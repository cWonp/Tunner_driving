
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"
#include <nav_msgs/MapMetaData.h>
#include "nav_msgs/Odometry.h"
#include "../include/Interrupt.h"

#include "time.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <list>

#include <leo_driving/charging_done.h>
#include <leo_driving/PlotMsg.h>
#include <leo_driving/UpdateOdom.h>

// Set time for STOP
#define STOP_MAX_CNT 50 // 5s
#define STOP_DOCKING 30 // 3s

// Set debug mode
#define QR 0
#define TUNNELPOSE 0
#define LATER 0
#define USING_CSV 0

// Robot heading
#define FORWARD 0
#define BACKWARD 1

// Only in old version = can be erased//////////////
#define STOP_MAX 30
#define FIRST_WAIT_MAX 150
#define TUNNEL_ANGLE_MAX 3.13

#define TorotateAtHome 0.3 // m
#define QR_DISTANCE 2000
#define QR_reset 100               // To initialize Docking out
#define QR_home 3                  // 101 To rotate at home
#define QR_end 4                   // 102 To rotate to go home
#define turning_angle_encoder 3.05 // 170 degree

#define TURNING_STOP 10

#define MODE_LENGTH 8
#define GOAL_LENGTH 3
/////////////////////////////////////////////////////

enum MODE_
{
    CHARGE_MODE = 0,
    STANDBY_MODE,
    DOCKING_MODE,
    DOCK_IN_MODE,
    DOCK_OUT_MODE,
    DOCK_OUT_PRE_MODE,
    DOCK_IN_PRE_MODE,
    AUTO_PRE_IMAGE_MODE,
    AUTO_PRE_LIDAR_MODE,
    AUTO_IMAGE_MODE,
    AUTO_LIDAR_MODE,
    MANUAL_MODE,
    STOP_MODE,
    TURN_MODE,
};

enum ACTION_MODE_
{
    STOP = 0,
    TURN,
    GOAL_STATUS_CHANGED,
    DOCKING_IN,
    WAITING_CHARGE,
    DOCKING_OUT,
    NEAR_GOAL,
    RUN,
};

/* What ...
 * mobile_direction
 * initial_start
 *
 *
 *
 *
 *
 * */

using namespace std;

/////////////////////////////////////////////////////////////////////////////
namespace auto_driving
{

    class LocalizationNode : public nodelet::Nodelet
    {
    public:
        LocalizationNode() = default;

        struct goal
        {
            geometry_msgs::PoseStamped pos;
            bool docking_station;
        };

    private:
        virtual void onInit()
        {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle nhp = getPrivateNodeHandle();

            // Configuration //
            nhp.param("global_dist_boundary", config_.global_dist_boundary_, 0.3);
            nhp.param("global_angle_boundary", config_.global_angle_boundary_, 0.05);
            nhp.param("HJ_MODE", config_.HJ_MODE_, 0);
            nhp.param("Without_QR_move", config_.Without_QR_move_, false);
            nhp.param("Main_start_x", config_.Main_start_x_, 0.0);
            nhp.param("Main_start_y", config_.Main_start_y_, 0.0);
            nhp.param("Main_goal_x", config_.Main_goal_x_, 0.0);
            nhp.param("Main_goal_y", config_.Main_goal_y_, 0.0);
            nhp.param("tunnel_start", config_.tunnel_start_, 0.0);
            nhp.param("tunnel_goal", config_.tunnel_goal_, 0.0);
            nhp.param("amcl_driving", config_.amcl_driving_, true);

            sub_joy_ = nhp.subscribe<std_msgs::Int32>("/joy_from_cmd", 10, &LocalizationNode::JoyCmdCallback, this);                           // Joystick data from cmd_node
            sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::UpdatePosCallback, this); // Pose callback from amcl node
            sub_pose_lio_ = nhp.subscribe<nav_msgs::Odometry>("/localization", 10, &LocalizationNode::Update3dPosCallback, this);              // fast-lio pose
            sub_camera_line_end_ = nhp.subscribe<std_msgs::Empty>("/camera_noline", 1, &LocalizationNode::CameraEndCallback, this);

            /* role as main function */
            // sub_pose_driving_ = nhp.subscribe<std_msgs::Int32> ("/cmd_publish", 10, &LocalizationNode::Run, this);
            sub_pose_driving_ =
                nhp.createTimer(ros::Duration(0.1),
                                &LocalizationNode::Run, this);

            sub_goal_ = nhp.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::SetGoalfromRviz, this); // Clicked point from RVIZ
            sub_event_end_ = nhp.subscribe<std_msgs::Empty>("/event_end", 1, &LocalizationNode::EventEndCallback, this);                  // Event : pcw
            sub_mode_decision_ = nhp.subscribe<std_msgs::Int32>("/Mode_Decision", 1, &LocalizationNode::ModedecisionCallback, this);      // To jump behavior_cnd
            sub_docking_done_ = nhp.advertiseService("charge_done", &LocalizationNode::docking_done, this);                               // TODO Docking done from robot-station
            sub_interrupt_ = nhp.subscribe<leo_driving::Interrupt>("/Interrupt", 1, &LocalizationNode::InterruptCallback, this);          // To jump behavior_cnd

            /* output primitive_mode to the cmd_publish_node */
            pub_localization_ = nhp.advertise<std_msgs::Float32MultiArray>("/localization_data", 10); // To communicate with cmd_node
            pub_action_ = nhp.advertise<std_msgs::Int32>("/action", 10);                              // Monitor current action

            /* onoff fast_lio */
            pub_3dlocal_start_ = nhp.advertise<std_msgs::Bool>("/localization_start", 1);
            pub_3dlocal_stop_ = nhp.advertise<std_msgs::Bool>("/localization_stop", 1);

            /* not using now */
            sub_mode_ = nhp.subscribe("/mode/low", 10, &LocalizationNode::DecisionpublishCmd, this); // To get a mode/low from Hanjeon
            pub_robot_pose_ = nhp.advertise<geometry_msgs::PoseStamped>("/state/pose", 10);          // To send to robot-station or Hanjeon
            pub_mode_call_ = nhp.advertise<std_msgs::Int32>("/mode/low", 10);                        // To use Dock in wiht Hanjeon

            /* topics for decision module */
            // oper_mode = nhp.subscribe<std_msgs::Int32>("/robot_mode", 1, &LocalizationNode::Set_Mode, this);

            /* unknown.. */
            sub_predone_ = nhp.subscribe("/auto_pre_lidar_mode/end", 1, &LocalizationNode::predoneCallback, this); // After finishing AUTO_PRE_LIDAR MODE

            /* Used for Aruco QR */
            // sub_area_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/fiducial_area_d", 1, &LocalizationNode::areaDataCallback, this);//QR fiducital_area detection
            // sub_QRinit_ = nhp.subscribe("/QR_TEST", 1, &LocalizationNode::QRtestCallback, this); //While HJ_mode==1 or 2, the mode is changed to another HJ_mode ==2 or 1
            // pub_QR_= nhp.advertise<std_msgs::Int32>("/QR_mode", 10); //To send to robot-station or Hanjeon --> not gonna use

            /* for visualization. Unconnected to the other node */
            pub_log_data_ = nhp.advertise<leo_driving::PlotMsg>("/PlotMsg_data", 10); // To save the data to plot
            pub_tunnel_pose = nhp.advertise<geometry_msgs::Point>("Tunnel_pose", 10); // pcw tunnel pose
        }

        void JoyCmdCallback(const std_msgs::Int32::ConstPtr &joy_msg)
        {
            if (config_.HJ_MODE_ != 0)
            {
                switch (joy_msg->data)
                {
                case 0: // A
                    //                Next_step = true;
                    ROS_INFO("Joy A: Behavior ++");
                    break;
                case 1: // B
                    Joy_mode = true;
                    break;

                case 3: // X
                    // behavior_cnt=0;
                    ROS_INFO("Joy X: Behavior 0");

                    Next_step = true;

                    // pcw0331 eventend
                    //{
                    /*
                                    goal_status_changed = true;
                                    event_end_flag = true;
                                    idx_start_goal = 0;
                                    cout<<"#####################End event!!!!!!####################"<<endl;
                    */
                    //}

                    break;
                case 4: // Y
                    ROS_INFO("Joy Y: Behavior 0");

                    // pcw0331 eventstart
                    //{
                    /*
                                    goal_count_ = 0;
                                    event_flag=true; //TODO When the goal is set to basic goal, this parameter should be 'false'
                                    goal_status_changed = true;

                                    event_goals[0].pose.position.x = 2.515;
                                    event_goals[0].pose.position.y = 0.059;
                                    event_goals[1].pose.position.x = 4.967;
                                    event_goals[1].pose.position.y = 0.153;
                    */
                    //}
                    break;

                case 100:
                    Joy_mode = false;
                    break;
                }
            }
        }
        void UpdatePosCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose_msg)
        {
            pub_robot_pose_.publish(amcl_pose_msg);
            current_pose = *amcl_pose_msg;
            // system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
        }
        void Update3dPosCallback(const nav_msgs::Odometry::ConstPtr &lio_pose_msg)
        {
            geometry_msgs::PoseWithCovarianceStamped pose_stamped;
            pose_stamped.header = lio_pose_msg->header;
            pose_stamped.pose = lio_pose_msg->pose;
            pub_robot_pose_.publish(pose_stamped);
            current_pose = pose_stamped;
            // system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
        }
        void InterruptCallback(const leo_driving::Interrupt::ConstPtr &msg)
        {
            geometry_msgs::PoseStamped temp;

            if (msg->enableEvent == 1) // Start event interrupt
            {
                event_goals[0].pos.pose.position.x = msg->event_x1;
                event_goals[0].pos.pose.position.y = msg->event_y1;
                event_goals[0].pos.pose.position.z = msg->event_z1;
                event_goals[1].pos.pose.position.x = msg->event_x2;
                event_goals[1].pos.pose.position.y = msg->event_y2;
                event_goals[1].pos.pose.position.z = msg->event_z2;

                event_flag = true;
                goal_status_changed = true;
                cout << "####################Start event!!!!!!###################" << endl;
            }
            else if(msg->enableEvent == 2){ // End event interrupt
                goal_status_changed = true;
                event_end_flag = true;
                idx_start_goal = 0;
                cout << "#####################End event!!!!!!####################" << endl;
            }
            if(msg->enableDocking == 1){ // Start docking Interrupt
                
            }
        }

        // void Run(const std_msgs::Int32::ConstPtr &empty_pose_msg){
        void Run(const ros::TimerEvent &event)
        {

            goal current_goal;

            if (FIRST_START_FLAG == true)
            {
                primitive_mode = STOP_MODE;
                FIRST_START_FLAG = false;

                // ReadCSV(); // Get map path

                // Set main goal from config
                goal main_start;
                main_start.pos.pose.position.x = config_.Main_start_x_;
                main_start.pos.pose.position.y = config_.Main_start_y_;
                main_start.docking_station = true;

                goal main_end;
                main_end.pos.pose.position.x = config_.Main_goal_x_;
                main_end.pos.pose.position.y = config_.Main_goal_y_;
                main_start.docking_station = false;

                // dummy position, need to change
                goal dock;
                dock.pos.pose.position.x = (main_end.pos.pose.position.x + main_start.pos.pose.position.x) / 2.0;
                dock.pos.pose.position.y = (main_end.pos.pose.position.y + main_start.pos.pose.position.y) / 2.0;
                dock.docking_station = true;

                auto_goals[0] = main_start;
                auto_goals[1] = dock;
                auto_goals[2] = main_end;

                for (int i = 0; i < GOAL_LENGTH; i++)
                {
                    current_goals[i] = auto_goals[i];
                    cout << "goal_list[" << i << "] = "
                         << "( " << auto_goals[i].pos.pose.position.x << ", " << auto_goals[i].pos.pose.position.y << " )"
                         << ", docking_station = " << auto_goals[i].docking_station << endl;
                }

                // Set main goal as current goal
                current_goal = auto_goals[0];

                heading = FORWARD;
            }
            current_goal = current_goals[auto_goal_num];

            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg = current_pose;
            std_msgs::Int32 mode_dockin;
            std_msgs::Float32MultiArray localization_msgs;
            localization_msgs.data.clear();
            std_msgs::Int32 current_action;

            if (start_wait < 150)
            {
                start_wait++;
                ROS_INFO_ONCE("Waiting for inserting goal... ");

                localization_msgs.data.push_back(0); // Postech mode
                localization_msgs.data.push_back(0); // g_err.x
                localization_msgs.data.push_back(0); // g_err.y
                localization_msgs.data.push_back(0); // global_theta_err
                localization_msgs.data.push_back(0); // global_c_theta_err
                localization_msgs.data.push_back(0); // initcall
                localization_msgs.data.push_back(0); // dokcking_cmd
                localization_msgs.data.push_back(0); // global x
                localization_msgs.data.push_back(0); // global y

                pub_localization_.publish(localization_msgs);
                return;
            }
            else
            {
                ROS_INFO_ONCE("Goal is set");
                cout << endl
                     << "1) goal_count_ = " << goal_count_ << endl;
                cout << "2) goal_status_changed = " << goal_status_changed << endl;
                cout << "3) idx                 = " << idx_start_goal << endl;

                ////--------------------------pcw event goal
            }

            // 1. Calculate Global Error~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            global_err g_err;
            g_err.x = current_goal.pos.pose.position.x - pose_msg.pose.pose.position.x;
            g_err.y = current_goal.pos.pose.position.y - pose_msg.pose.pose.position.y;
            g_err.dist = sqrt(g_err.x * g_err.x + g_err.y * g_err.y);

            std::cout << "4) goal (x,y) : "
                      << "(" << goal_current.pose.position.x << ", " << goal_current.pose.position.y << ")" << std::endl;
            std::cout << "5) curr (x,y) : "
                      << "(" << pose_msg.pose.pose.position.x << ", " << pose_msg.pose.pose.position.y << ")" << std::endl;
            std::cout << "6) distance   : " << g_err.dist << std::endl;
            std::cout << "9) start_wait : " << start_wait << std::endl;
            std::cout << "7) event_goal[0] = " << event_goals[0].pos.pose.position.x << " , " << event_goals[0].pos.pose.position.y << std::endl;
            std::cout << "8) event_goal[1] = " << event_goals[1].pos.pose.position.x << " , " << event_goals[1].pos.pose.position.y << std::endl;

            tf2::Quaternion orientation(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);
            tf2::Matrix3x3 m(orientation);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            g_x_err = g_err.x;
            g_y_err = g_err.y;

            g_ctheta = (float)yaw; // TODO for straight control, have to change atan2

            std_msgs::Int32 QR_msg;
            std_msgs::Bool localization_pub;
            localization_pub.data = true;

            // WAITING_CHARGE,DOCKING_OUT,STOP,RUN,NEAR_GOAL,STOP,DOCKING_IN,TURN
            // With docking postech mode
            static bool change_mode = false;
            static bool event_drive = false;
            static bool near_goal = false;
            static bool init = true;

            static int mode_num = 0;
            static int dir = 1;

            if (goal_status_changed)
            {
                current_action.data = GOAL_STATUS_CHANGED;
                cout << "[1] Motion : @@@ goal_status_changed @@@" << endl;
                current_pos.pose.position.x = pose_msg.pose.pose.position.x;
                current_pos.pose.position.y = pose_msg.pose.pose.position.y;

                if (event_flag)
                {                     // enter event mode
                    goal_idx_cur = 0; // need to change, find nearest goal
                    dir = 1;
                    for (int i = 0; i < event_goal_num; i++)
                    {
                        current_goals[i] = event_goals[i];
                    }
                    event_flag = false;
                    change_mode = true;
                    event_drive = true;
                    mode_num = 0;
                    cout << "[2] Event mode start" << endl;
                }
                else if (event_end_flag)
                {                     // finish event mode
                    goal_idx_cur = 0; // need to change, find nearest goal if docking is not necessary
                    dir = 1;
                    for (int i = 0; i < GOAL_LENGTH; i++)
                    {
                        current_goals[i] = auto_goals[i];
                    }
                    event_end_flag = false;
                    current_mode = STOP;
                    event_drive = false;
                    event_goal_num = 0;
                    cout << "[2] Event mode finished" << endl;
                }
                current_goal = current_goals[goal_idx_cur];
                goal_idx_cur += dir;
                if (goal_idx_cur == 0 || goal_idx_cur == GOAL_LENGTH - 1 && !event_drive || goal_idx_cur == event_goal_num - 1 && event_drive)
                {
                    dir *= (-1);
                }
                goal_status_changed = false;
                cout << "[3] goal_current = " << goal_current.pose.position.x << endl;
            }

            // change to next mode
            if (change_mode)
            {
                current_mode = auto_modes[mode_num];
                mode_num = mode_num % MODE_LENGTH;
                change_mode = false;
            }

            switch (current_mode)
            {
            case STOP:
                current_action.data = current_mode;
                if (STOP_cnt < STOP_MAX_CNT)
                { // Stop before turning
                    STOP_cnt++;
                    primitive_mode = STOP_MODE;
                }
                else
                {
                    STOP_cnt = 0;
                    change_mode = true;
                    mode_num++;
                }
                break;

            case TURN:
                current_action.data = current_mode;
                Camera_noline_flag = false; // To make sure if there is flag sign
                current_pos.pose.position.x = pose_msg.pose.pose.position.x;
                current_pos.pose.position.y = pose_msg.pose.pose.position.y;

                // if docking station, skip turning
                if (current_goal.docking_station && need_docking)
                {
                    primitive_mode = STOP_MODE;
                    is_rotating_ = false;
                    mode_num++;
                    change_mode = true;
                    cout << "[1] Motion : @@@ Turn motion @@@" << endl;
                    cout << "[2] Need to charge, Skip Turning" << endl;
                }
                else if (!NeedTurn(current_goal, current_pos))
                {
                    primitive_mode = STOP_MODE;
                    is_rotating_ = false;
                    mode_num++;
                    change_mode = true;
                    cout << "[1] Motion : @@@ Turn motion @@@" << endl;
                    cout << "[2] No need to Turn" << endl;
                }
                else
                {
                    primitive_mode = TURN_MODE;
                    if (!is_rotating_)
                    { // Initialize turn motion
                        is_rotating_ = true;

                        // Save current state before turn
                        static_x = pose_msg.pose.pose.position.x;
                        static_y = pose_msg.pose.pose.position.y;
                        goal_yaw = yaw + M_PI;

                        // Scaling goal_yaw
                        if (goal_yaw > M_PI)
                            goal_yaw -= 2 * M_PI;
                        else if (goal_yaw < -M_PI)
                            goal_yaw += 2 * M_PI;
                    }

                    g_err.ang = goal_yaw - yaw;
                    // Scaling g_err.ang
                    if (g_err.ang > M_PI)
                        g_err.ang -= 2 * M_PI;
                    else if (g_err.ang < -M_PI)
                        g_err.ang += 2 * M_PI;

                    std::cout << "start yaw: " << goal_yaw << ", cur yaw: " << yaw << ", yaw err:" << g_err.ang << std::endl;
#if TUNNELPOSE
                    if (fabs(encoder_angle - 3.05) < config_.global_angle_boundary_)
                    {
                        Next_step = true;
                        ROS_INFO("Tunnel POse STOP !!!!");
                    }
#endif
                    if (Next_step || abs(g_err.ang) < config_.global_angle_boundary_)
                    { // Done turning
                        Next_step = false;
                        primitive_mode = STOP_MODE;
                        is_rotating_ = false;
                        // heading
                        if (heading == FORWARD)
                            heading = BACKWARD;
                        else if (heading == BACKWARD)
                            heading = FORWARD;

                        mode_num++;
                        change_mode = true;
                    }
                    g_err.static_x_err = static_x - pose_msg.pose.pose.position.x;
                    g_err.static_y_err = static_y - pose_msg.pose.pose.position.y;
                    static_t = goal_yaw;
                    static_ct = yaw;
                    cout << "[1] Motion : @@@ Turn motion @@@" << endl;
                    cout << "[2] global angle err = " << abs(g_err.ang) << endl;
                }
                break;

            case DOCKING_IN:
                current_action.data = current_mode;
                if (current_goal.docking_station && need_docking && near_goal)
                {
                    if (STOP_cnt < STOP_DOCKING)
                    { // Stop before docking
                        STOP_cnt++;
                        primitive_mode = STOP_MODE;
                    }
                    else
                    {
                        STOP_cnt = 0;
                        primitive_mode = DOCK_IN_MODE;

                        if (init)
                        {
                            pub_3dlocal_stop_.publish(localization_pub);
                            init = false;
                        }

                        if (!Joy_mode)
                        {
                            mode_dockin.data = 4;
                            pub_mode_call_.publish(mode_dockin);
                        }
                        else if (Joy_mode)
                        { // To stop dock in node
                            mode_dockin.data = 0;
                            pub_mode_call_.publish(mode_dockin);
                        }

                        if (Next_step || Camera_noline_flag)
                        {
                            mode_dockin.data = 0;                // mode/low : stop_mode
                            pub_mode_call_.publish(mode_dockin); // To stop dock in with camera.

                            Camera_noline_flag = false;
                            Next_step = false;
                            primitive_mode = STOP_MODE;

                            change_mode = true;
                            mode_num++;
                            init = true;
                        }
                        cout << "[1] Motion : @@@ Docking in motion @@@" << endl;
                    }
                }
                else
                {
                    change_mode = true;
                    mode_num++;
                    init = true;
                    cout << "[1] Motion : @@@ Goal is not Docking Station, skip docking @@@" << endl;
                }

                break;

            case WAITING_CHARGE:
                Camera_noline_flag = false; // To make sure if there is flag sign
                current_action.data = current_mode;
                primitive_mode = STOP_MODE;
                if (current_goal.docking_station && need_docking && near_goal)
                {
                    if (Next_step || Charging_done_flag)
                    { // rosserive call
                        Next_step = false;
                        primitive_mode = STOP_MODE;
                        Charging_done_flag = 0;
                        change_mode = true;
                        mode_num++;
                    }
                    cout << "[1] Motion : @@@ do_waiting_charge @@@" << endl;
                }
                else
                {
                    cout << "[1] Motion : @@@ Skip do_waiting charge @@@" << endl;
                    change_mode = true;
                    mode_num++;
                }
                break;

            case DOCKING_OUT:
                current_action.data = current_mode;
                if (current_goal.docking_station && need_docking && near_goal)
                {
                    static int end_flag = false;

                    if (!end_flag)
                    {
                        primitive_mode = DOCK_OUT_MODE; // What the,,, Originally it was cancled... [check]

                        if (!Joy_mode)
                        {
                            mode_dockin.data = 5;
                            pub_mode_call_.publish(mode_dockin);
                        }
                        else if (Joy_mode)
                        { // To stop dock out node
                            mode_dockin.data = 0;
                            pub_mode_call_.publish(mode_dockin);
                        }

                        if (Next_step || Camera_noline_flag)
                        { // To use AMCL pose probably
                            end_flag = true;
                            mode_dockin.data = 0;
                            pub_mode_call_.publish(mode_dockin); // To stop dock out

                            pub_3dlocal_start_.publish(localization_pub);
                        }
                    }
                    else
                    {
                        if (STOP_cnt < STOP_DOCKING)
                        { // Stop before docking_out
                            STOP_cnt++;
                            primitive_mode = STOP_MODE;
                        }
                        else
                        {
                            STOP_cnt = 0;
                            end_flag = false;

                            Camera_noline_flag = false;
                            Next_step = false;
                            primitive_mode = STOP_MODE;
                        }
                    }
                    // need to fix, add change mode to the end of docking_out
                    // need_docking=false;
                    cout << "[1] Motion : @@@ Docking out @@@" << endl;
                }
                else
                {
                    cout << "[1] Motion : @@@ Skip docking out @@@" << endl;
                    near_goal = false;
                    mode_num++;
                    change_mode = true;
                }
                break;

            case NEAR_GOAL:
                /*close to the goal position have to do other motion*/
                current_action.data = current_mode;
                primitive_mode = STOP_MODE;
                Next_step = false;
                goal_status_changed = true;
                near_goal = true;
                change_mode = true;
                mode_num++;
                cout << "[1] Motion : @@@ Arrive to the goal position @@@" << endl;
                break;

            case RUN:
                /*Far from the goal position
            Have to run to goal position*/
                current_action.data = current_mode;
                primitive_mode = AUTO_LIDAR_MODE;
                Camera_noline_flag = false; // To make sure if there is flag sign
                if (Next_step || g_err.dist < config_.global_dist_boundary_)
                {
                    change_mode = true;
                    mode_num++;
                }
                break;

            default:
                break;
            }

            float send_x, send_y, send_ref_yaw, send_yaw;

            if (primitive_mode == TURN_MODE)
            {
                send_x = g_err.static_x_err;
                send_y = g_err.static_y_err;
                send_ref_yaw = static_t;
                send_yaw = static_ct;
            }
            else
            {
                send_x = g_err.x;
                send_y = g_err.y;
                send_ref_yaw = g_rtheta;
                send_yaw = g_ctheta;
            }

            localization_msgs.data.push_back(primitive_mode);
            localization_msgs.data.push_back(send_x);
            localization_msgs.data.push_back(send_y);
            localization_msgs.data.push_back(send_ref_yaw);
            localization_msgs.data.push_back(send_yaw);
            localization_msgs.data.push_back(init_start_);
            localization_msgs.data.push_back(Docking_out_cmd);
            localization_msgs.data.push_back(pose_msg.pose.pose.position.x);
            localization_msgs.data.push_back(pose_msg.pose.pose.position.y);

            pub_localization_.publish(localization_msgs);
            pub_action_.publish(current_action);

            // Reset variables
            fid_area = 0; // To reset for the fid_area. because if the ID is not detected the previous data is still in.
                          /*
                                  // For data logging ----------------------------------------------------------
                                  leo_driving::PlotMsg Save_log;
                                  Save_log.x = pose_msg.pose.pose.position.x;
                                  Save_log.y = pose_msg.pose.pose.position.y;
              
                                  if(primitive_mode==TURN_MODE){
                                      std::cout <<"static x: " << g_err.static_x_err *cos(static_ct) + g_err.static_y_err *sin(static_ct)<< ", static y: "<<  -g_err.static_x_err *sin(static_ct) + g_err.static_y_err *cos(static_ct) << std::endl;
                                      Save_log.r_x = static_x;
                                      Save_log.r_y = static_y;
                                      Save_log.s_x = g_err.static_x_err *cos(static_ct) + g_err.static_y_err *sin(static_ct);
                                      Save_log.s_y = -g_err.static_x_err *sin(static_ct) + g_err.static_y_err *cos(static_ct);
                                      Save_log.rt = goal_yaw;
                                      Save_log.ct = yaw;
                                  }
                                  else{
                                      Save_log.r_x = goal_current.pose.position.x;
                                      Save_log.r_y = goal_current.pose.position.y;
                                  }
                                  pub_log_data_.publish(Save_log);
                          */
        }
        //*** Read CSV **************************************************************//
        void remove_spaces(char *s)
        {
            const char *d = s;
            do
            {
                while (*d == ' ')
                {
                    ++d;
                }
            } while (*s++ = *d++);
        }
        struct csv_data
        {
            char s[2][1024];
        };
        void getfield(char *line, csv_data *d, int end_idx)
        {
            int idx = 0;
            char *token = strtok(line, ",");
            do
            {
                strcpy(d->s[idx++], token);
            } while (idx != end_idx && (token = strtok(NULL, ",")));
        }
        void ReadCSV()
        {
            FILE *stream = fopen("/home/cocel/mapping_ws/src/FAST_LIO/PCD/path011922_14_54.csv", "r"); // have to change current map csv file

            char line[1024];
            csv_data d;
            while (fgets(line, 1024, stream))
            {
                remove_spaces(line);

                char *tmp = strdup(line);
                getfield(tmp, &d, 2);

                geometry_msgs::PoseStamped position;
                position.pose.position.x = stof(d.s[0]);
                position.pose.position.y = stof(d.s[1]);
                map_data.push_back(position);
                free(tmp);
            }
            /* cout map_data */
            //        for(int i = 0; i < map_data.size(); i++){
            //            cout<<"map_data["<<i<<"] = "<<map_data[i]<<endl;
            //        }
        }

        //*** Event *****************************************************************//
        double FindIdx(geometry_msgs::PoseStamped pos_)
        {
            vector<double> idx;
            double smallest_idx = 0;
            double x = pos_.pose.position.x;
            double y = pos_.pose.position.y;
#if USING_CSV
            for (int i = 0; i < map_data.size() - 1; i++)
            {
                if ((x <= map_data[i].pose.position.x && x > map_data[i + 1].pose.position.x) ||
                    (x > map_data[i].pose.position.x && x <= map_data[i + 1].pose.position.x))
                {
                    idx.push_back(i);
                }
            }
            for (int i = 0; i < idx.size(); i++)
            {
                double err = 1000;
                double temp_err = fabs(y - map_data[idx[i]].pose.position.y);
                if (temp_err < err)
                {
                    err = temp_err;
                    smallest_idx = idx[i];
                }
            }
            //        if(idx == 0){
            //            double CalcPosErr(map_data[0],pos_)
            //        }
            return smallest_idx;
#else
            // Just compare using x position
            return x;
#endif
        }
        double CalcPosErr(geometry_msgs::PoseStamped pos1, geometry_msgs::PoseStamped pos2)
        {
            double err_x = pos1.pose.position.x - pos2.pose.position.x;
            double err_y = pos1.pose.position.y - pos2.pose.position.y;

            return sqrt(pow(err_x, 2) + pow(err_y, 2));
        }

        // Need to fix
        // Decide turning using current pos, yaw & changed goal pos
        bool NeedTurn(goal next_goal, geometry_msgs::PoseStamped current_pos)
        {
            /*
            bool need_turn = false;
            double idx_event_goal = FindIdx(event_goal_pos);
            double idx_previous_goal = FindIdx(previous_goal_pos);
            double idx_current_pos = FindIdx(current_pos);

            //need to erase
            //        cout<<"idx_event_goal    = "<<idx_event_goal<<endl;
            //        cout<<"idx_previous_goal = "<<idx_previous_goal<<endl;
            //        cout<<"idx_current_pos   = "<<idx_current_pos<<endl;

            // Not using heading===
            // if(((idx_current_pos<idx_previous_goal)&&(idx_current_pos>idx_event_goal))||((idx_current_pos>idx_previous_goal)&&(idx_current_pos<idx_event_goal))){
            //     need_turn = true;
            // }
            // else if(idx_event_goal == idx_current_pos){
            //     if(CalcPosErr(event_goal_pos, previous_goal_pos) > CalcPosErr(current_pos, previous_goal_pos)){
            //         need_turn = true;
            //     }
            //     else{
            //         need_turn = false;
            //     }
            // }
            // else{
            //     need_turn = false;
            // }

            //Using heading===
            if(idx_event_goal < idx_current_pos){
                if(heading == FORWARD)
                    need_turn = true;
                else if(heading == BACKWARD)
                    need_turn = false;
            }
            else if(idx_event_goal > idx_current_pos){
                if(heading == FORWARD)
                    need_turn = false;
                else if(heading == BACKWARD)
                    need_turn = true;
            }

            cout<<"NeedTurn~~~~~~~~~~~~~~~~~~~~~"<<endl;
            cout<<"heading = "<<heading<<endl;
            cout<<"idx_event_goal = "<<idx_event_goal<<endl;
            cout<<"idx_previous_goal = "<<idx_previous_goal<<endl;
            cout<<"idx_current_pos = "<<idx_current_pos<<endl;
            cout<<"turn     = "<<need_turn<<endl;
            return need_turn;
            */
            return false;
        }

        //*** Tunnel Pose ***********************************************************//
        void SetTunnelPoseUsingQR(int id)
        { // pcw for QR local
            // just for test have to modify to real value
            // if(id <100)
            if (id == 1 || id == 2)
                tunnel_pose = id * 100;
        }

        //*** Docking ***************************************************************//
        void CameraEndCallback(const std_msgs::Empty::ConstPtr &msg)
        {
            Camera_noline_flag = true;
        }
        bool docking_done(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
        {
            Charging_done_flag = true;
            return true;
        }

        //*** Goal position *********************************************************//
        void EventEndCallback(const std_msgs::Empty::ConstPtr &msg)
        { // Event : pcw
            goal_status_changed = true;
            event_end_flag = true;
            idx_start_goal = 0;
            cout << "#####################End event!!!!!!####################" << endl;
        }
        void SetGoalfromRviz(const geometry_msgs::PoseStamped::ConstPtr &click_msg)
        {
            geometry_msgs::PoseStamped temp;

            event_goals[event_goal_num].pos = *click_msg;
            event_goals[event_goal_num].docking_station = false;
            event_goal_num++;
            ROS_INFO("%d th goal is set", goal_count_);

            if (event_goal_num = 2)
            {
                goal_count_ = 0;
                event_flag = true; // TODO When the goal is set to basic goal, this parameter should be 'false'
                goal_status_changed = true;

                double Idx[2] = {
                    0,
                };
                for (int i = 0; i < 2; i++)
                {
                    Idx[i] = FindIdx(event_goals[i].pos);
                }
                double ind_dock = FindIdx(auto_goals[1].pos);
                if (Idx[0] > Idx[1])
                { // have to sort
                    goal temp = event_goals[0];
                    event_goals[0] = event_goals[1];
                    event_goals[1] = temp;
                }

                // docking station between event goals
                if (Idx[1] >= ind_dock && Idx[0] <= ind_dock)
                {
                    event_goals[2] = event_goals[1];
                    event_goals[1] = auto_goals[1];
                    event_goal_num++;
                }
                // TODO : else if(Idx[0] == Idx[1])

                // event for test
                //  event_goals[0].pose.position.x = 1.0;
                //  event_goals[0].pose.position.y = 0.0;
                //  event_goals[1].pose.position.x = 2.0;
                //  event_goals[1].pose.position.y = 0.0;
            }
        }
        int findNearestGoal()
        {
            return 0;
        }

        //*** External ***********************************************************//
        void ModedecisionCallback(const std_msgs::Int32::ConstPtr &msg_cnt) // To jump the behavior of user
        {
            behavior_cnt = msg_cnt->data;
        }

        void Set_Mode(const std_msgs::Int32::ConstPtr &msg_cnt)
        {
            robot_mode = msg_cnt->data;
        }

        //*** QR *****************************************************************//
        void QRtestCallback(const std_msgs::Float32::ConstPtr &QR_flag_msgs);
        void areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr &area_msgs);

        //*** For future *********************************************************//
        bool odominit(leo_driving::UpdateOdom::Request &req, leo_driving::UpdateOdom::Response &res);
        void predoneCallback(const std_msgs::Empty::ConstPtr &msg_empty);
        void DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call);
        void RobotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs);

    private:
        ros::Subscriber sub_joy_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_pose_lio_;
        // ros::Subscriber sub_pose_driving_;
        ros::Timer sub_pose_driving_;
        ros::Subscriber sub_goal_;
        ros::Subscriber sub_camera_line_end_;

        ros::Subscriber sub_area_;
        ros::Subscriber sub_mode_;
        ros::Subscriber sub_QRinit_;

        ros::Subscriber sub_predone_;
        ros::Subscriber sub_mode_decision_;
        ros::Subscriber sub_pose_dt_from_linearvel; // pcw for QR local
        ros::Subscriber sub_event_end_;             // Event : pcw
        ros::Subscriber sub_interrupt_;

        ros::ServiceServer sub_docking_done_;

        ros::Subscriber oper_mode;

        ros::Publisher pub_localization_;
        ros::Publisher pub_action_;
        ros::Publisher pub_robot_pose_;
        ros::Publisher pub_QR_;
        ros::Publisher pub_log_data_;
        ros::Publisher pub_mode_call_;
        ros::Publisher pub_tunnel_pose; // pcw for Tunnel pose

        ros::Publisher pub_3dlocal_start_;
        ros::Publisher pub_3dlocal_stop_;

        /* Run */
        bool heading = 0;
        bool FIRST_START_FLAG = true; // To set a behavior_cnt at first

        bool is_rotating_ = false;
        bool init_start_ = false; // To initial odom and IMU
        bool g_rtheta_flag = true;

        bool event_flag = false; // For a event goal
        bool event_end_flag = false;
        int idx_start_goal = 0;

        bool Joy_mode = false;

        float static_x = 0.0, static_y = 0.0, static_t = 0.0, static_ct = 0.0;
        float g_x_err, g_y_err, g_rtheta, g_ctheta;
        int primitive_mode;
        float line_y_pose = 0; // not necessary

        struct global_err
        {
            float x, y;
            float dist;
            float ang;
            float static_x_err, static_y_err;
        };

        // robot mode
        unsigned int robot_mode = 0;
        unsigned int current_mode = STOP_MODE;
        unsigned int mode_num = 0;
        unsigned int auto_modes[MODE_LENGTH] = {WAITING_CHARGE, DOCKING_OUT, STOP, RUN, NEAR_GOAL, STOP, DOCKING_IN, TURN};

        unsigned int auto_goal_num = 0;
        goal auto_goals[GOAL_LENGTH];
        unsigned int event_goal_num = 0;
        goal event_goals[GOAL_LENGTH];
        goal current_goals[GOAL_LENGTH];

        bool event_docking = false;

        bool need_docking = false;

        int goal_idx_cur = 1;

        // GOAL
        unsigned int start_wait = 0;
        int goal_index_ = 0;
        int goal_count_ = 0;
        float goal_yaw = M_PI;

        // Camera
        int fid_ID = 0;
        float fid_area = 0;
        bool Camera_noline_flag = false;

        // Linear vel pose from roverroboics_ros_drier
        // pcw for QR local
        double dt_x_by_linearvel = 0;
        double dt_y_by_linearvel = 0;
        double pos_x_by_linearvel = 0;
        double pos_y_by_linearvel = 0;
        //

        // Decision
        unsigned int behavior_cnt = 6; // For case 5 or case 6
        bool Charging_done_flag = false;
        int HJ_mode_low = STOP;
        bool Next_step = false;

        unsigned int STOP_cnt = 0;

        // To recognize only one press for joystick
        unsigned int switch_flag0 = 0;
        unsigned int switch_flag1 = 0;

        double past_time = 0;
        double tunnel_pose = 0, current_tunnel_pose = 0;
        double Tunnel_reference = 0;
        int mobile_direction = 1;

        bool home_arrival_flag = true;
        float Docking_out_cmd = 0;

        double encoder_angle = 0;
        double encoder_angular = 0;

        // odom publish
        double odom_x = 0;
        double odom_y = 0;
        double odom_theta = 0;
        double update_x = 0, update_y = 0, update_theta = 0;
        bool odom_update = false, odom_init_flag = false;

        geometry_msgs::PoseStamped goal_current;
        geometry_msgs::PoseStamped goal_previous;
        geometry_msgs::PoseStamped current_pos;

        geometry_msgs::PoseWithCovarianceStamped current_pose;

        // Event : pcw
        bool goal_status_changed = true;
        vector<geometry_msgs::PoseStamped> map_data;
        // end Event : pcw

        /** configuration parameters */
        typedef struct
        {
            double global_dist_boundary_;
            double global_angle_boundary_;
            bool amcl_driving_;
            int HJ_MODE_;
            bool Without_QR_move_;
            double Main_start_x_;
            double Main_start_y_;
            double Main_goal_x_;
            double Main_goal_y_;
            double tunnel_start_;
            double tunnel_goal_;

        } Config;
        Config config_;
    };

    void LocalizationNode::areaDataCallback(const std_msgs::Float32MultiArray::ConstPtr &area_msgs) // QR detection Aurco realsense for a front camera.
    {
        fid_ID = (int)area_msgs->data[0];
        fid_area = area_msgs->data[1];
        // ROS_INFO("fid_ID: %d, are: %f",fid_ID, fid_area);

        SetTunnelPoseUsingQR(fid_ID);
    }
    void LocalizationNode::QRtestCallback(const std_msgs::Float32::ConstPtr &QR_flag_msgs)
    {
        if (QR_flag_msgs->data == 0)
        {
            config_.Without_QR_move_ = false;
            ROS_INFO("When HJ_MODE==2, with QR initialzation");
        }
        else if (QR_flag_msgs->data == 1)
        {
            config_.Without_QR_move_ = true;
            ROS_INFO("When HJ_MODE==2, without QR initialzation");
        }
    }

    bool LocalizationNode::odominit(leo_driving::UpdateOdom::Request &req, leo_driving::UpdateOdom::Response &res)
    {
        odom_init_flag = true;
        update_x = req.odom_x;
        update_y = req.odom_y;
        update_theta = req.odom_theta;
    }
    void LocalizationNode::DecisionpublishCmd(const std_msgs::Int32::ConstPtr &mode_call)
    {
        HJ_mode_low = mode_call->data;
        // ROS_INFO("Mode: %d",HJ_mode_low);
    }
    void LocalizationNode::RobotOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msgs)
    {
        double dt = 0;
        ros::Time ros_now_time = ros::Time::now();
        double now_time = ros_now_time.toSec();

        dt = now_time - past_time;
        past_time = now_time;

        tunnel_pose += mobile_direction * odom_msgs->twist.twist.linear.x * dt;
        encoder_angular = odom_msgs->twist.twist.angular.z;
        if (behavior_cnt == 1 || behavior_cnt == 3)
        {
            //        encoder_angle += encoder_angular* dt*2/3;
            encoder_angle += encoder_angular * dt;
        }
        else
        {
            encoder_angle = 0;
        }
        geometry_msgs::Point tunnel_pos_pub;
        tunnel_pos_pub.x = tunnel_pose;
        tunnel_pos_pub.z = encoder_angle;
        pub_tunnel_pose.publish(tunnel_pos_pub);
    }
    void LocalizationNode::predoneCallback(const std_msgs::Empty::ConstPtr &msg_empty)
    {
        if (behavior_cnt == 1)
        {
            behavior_cnt = 2; // The pre lidar is finished; thus next step
            primitive_mode = STOP_MODE;

            is_rotating_ = false; // Those two parameters for waiting before rotating
            STOP_cnt = 0;
        }
        else if (behavior_cnt == 3)
        {
            behavior_cnt = 4; // The pre lidar is finished; thus next step
            primitive_mode = STOP_MODE;

            is_rotating_ = false;
            STOP_cnt = 0;
        }
    }
}
PLUGINLIB_EXPORT_CLASS(auto_driving::LocalizationNode, nodelet::Nodelet);