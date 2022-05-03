
#include <laser_geometry/laser_geometry.h>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>

//CMD
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
// #include <leo_driving/Mode.h>

#define INIT_WAIT 10
#define RATIO_VEL 1.0
#define TIMER_SAMPLE 0.1
#define ENABLE_STRAIGHT_CONTROL 1
#define NUM_JUNCTION 9
#define JUNCTION_STOP_SIZE 100

#define FORWARD_DRIECTION 1
#define BACKWARD_DRIECTION 2


using namespace message_filters;
using namespace std;

enum MODE_{
    CHARGE_MODE=0,
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
struct csv_data{
    char s[2][1024];
};
namespace auto_driving {

class AisleDetectNode : public nodelet::Nodelet {

public:
    AisleDetectNode() = default;

private:
    virtual void onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle nhp = getPrivateNodeHandle();
        
        // Configuration //
        nhp.param("line_thresh", config_.line_thresh_, 0.5);
        nhp.param("aisle_width", config_.aisle_width_, 0.6);

        //cmd
        nhp.param("Kpx_param", config_.Kpx_param_, 2.0);
        nhp.param("Kpy_param", config_.Kpy_param_, 1.1);
        nhp.param("Kpy_param_rot", config_.Kpy_param_rot_, 0.01);
        nhp.param("Kpx_str", config_.Kpx_str_, 2.0);
        nhp.param("Kpy_str", config_.Kpy_str_, 1.1);
        nhp.param("Kpy_str_rot", config_.Kpy_str_rot_, 0.01);
       
	nhp.param("tunnel_gain_boundary", config_.tunnel_gain_boundary_, 0.01);
        nhp.param("Kpy_param_boundary_gain", config_.Kpy_param_boundary_gain_, 0.05);
        nhp.param("theta_ratio", config_.theta_ratio_, 0.8);

        nhp.param("linear_vel", config_.linear_vel_, 0.0);
        nhp.param("obs_vel_ratio", config_.obs_vel_ratio_, 1.0);
        nhp.param("robot_width", config_.robot_width_, 0.45);
        nhp.param("front_obs", config_.front_obs_, 0.6);
        nhp.param("front_obs_2", config_.front_obs_2_, 0.5);
        nhp.param("spare_length", config_.spare_length_, 0.3);
        nhp.param("check_obstacles", config_.check_obstacles_, true);
        nhp.param("obs_avoidance_distance", config_.obs_avoidance_distance_, 0.1);
        nhp.param("obs_double_check", config_.obs_double_check_, true);
        nhp.param("obs_double_check_dist", config_.obs_double_check_dist_, 0.5);
        nhp.param("backward_length", config_.backward_length_, -0.3);


        nhp.param("amcl_driving", config_.amcl_driving_, true);
        nhp.param("rot_kx", config_.rot_kx_, 0.1);
        nhp.param("rot_ky", config_.rot_ky_, 0.1);
        nhp.param("rot_kt", config_.rot_kt_, 0.3);
        nhp.param("min_vel", config_.min_vel_, 0.05);
        nhp.param("min_rot", config_.min_rot_, 0.05);
        nhp.param("max_vel", config_.max_vel_, 0.5);
        nhp.param("max_rot", config_.max_rot_, 1.0);
        nhp.param("Postech_code", config_.Postech_code_, false);

        // Subscriber & Publisher
        sub_scan_ = nhp.subscribe("/rp/scan", 20, &AisleDetectNode::scanCallback, this);
        
        pub_line_ = nhp.advertise<sensor_msgs::PointCloud2>("/cluster_line", 10);
        pub_points_ = nhp.advertise<sensor_msgs::PointCloud2> ("/aisle_points", 10);

        pub_prelidar_fail_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/fail", 10);
        pub_lidar_fail_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/fail", 10);


        //cmd
        sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 10, &AisleDetectNode::joyCallback, this);
        sub_localization_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/localization_data", 10, &AisleDetectNode::localDataCallback, this);
        sub_mode_call_ = nhp.subscribe<std_msgs::Int32> ("/mode/low", 10, &AisleDetectNode::modeCallback, this);
        sub_speed_ = nhp.subscribe("/mission/setspeed", 1, &AisleDetectNode::SpeedCallback, this);
        sub_obs_dists_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/obs_dists", 10, &AisleDetectNode::obsCallback, this);


        pub_cmd_ = nhp.advertise<geometry_msgs::Twist> ("/cmd_vel", 20);
        pub_docking_end_ = nhp.advertise<std_msgs::Int32> ("/joy_from_cmd", 1); // Temperary To finish with joystick
        pub_prelidar_end_ = nhp.advertise<std_msgs::Empty> ("/auto_pre_lidar_mode/end", 10);

        timer_cmd_node = nhp.createTimer(ros::Duration(TIMER_SAMPLE), &AisleDetectNode::cmdtimercallback, this);
    };

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // double dt = 0;
        // ros::Time ros_now_time = ros::Time::now();
        // double now_time = ros_now_time.toSec();

        

        std_msgs::Empty EmptyMsg;
        // 1. Data type conversions (laser scan -> pointcloud2)
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud_msg;
        projector.projectLaser(*scan_msg, cloud_msg);

        pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(cloud_msg, *temp_cloud); // save cloud message to cloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

        // 2. Crop Point Cloud
        pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inrange(new pcl::PointCloud<pcl::PointXYZ>); // <- cropped cloud
        // set condition
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -config_.aisle_width_)));
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, config_.aisle_width_)));
        range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
        // conditional removal
        pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
        condrem.setInputCloud(cloud);
        condrem.setCondition(range_condition);
        condrem.setKeepOrganized(true);
        condrem.filter(*cloud_inrange);
        if (cloud_inrange->size() == 0)
        {
            this->pub_lidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            this->pub_prelidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            ROS_WARN("all points are cropped");
            return;
        }

        // 3. EXTRACT LINE (RANSAC ALGORITHM)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(config_.line_thresh_); // <- threshold (line width) // 0.5
        seg.setInputCloud(cloud_inrange);
        seg.segment(*inliers, *coefficients);
        extract.setInputCloud(cloud_inrange);
        extract.setIndices(inliers);
        extract.setNegative(false); //<- if true, it returns point cloud except the line.
        extract.filter(*cloud_inrange);

        // 4. Extract Line Cluster
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
        tree_cluster->setInputCloud(cloud_inrange);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(cloud_inrange);
        ec.setClusterTolerance(0.05); // <- If the two points have distance bigger than this tolerance, then points go to different clusters.
        ec.setMinClusterSize(30);
        ec.setMaxClusterSize(800);;
        ec.setSearchMethod(tree_cluster);
        ec.extract(cluster_indices);

        // extract first clustering (center cluster)
        int j = 0;
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            if (j == 0) {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                {
                    cloud_cluster->points.push_back (cloud_inrange->points[*pit]);
                }
                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
            }
            j++;
        }
        if((*cloud_cluster).size() == 0)
        {
            this->pub_lidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            this->pub_prelidar_fail_.publish(EmptyMsg); //TODO FAIL Pub
            ROS_WARN("Not enough points!");
            return;
        }

        // 5. FIND NEAREST POINT FROM THE ORIGIN ( => /nearest_point)
        pcl::PointCloud<pcl::PointXYZ> point_set; // Data to be published

        pcl::PointXYZ origin(0, 0, 0);
        pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud_cluster);
        std::vector<int> nn_indices(1);
        std::vector<float> nn_dists(1);
        tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
        point_set.push_back(cloud_cluster->points[nn_indices[0]]); // [0]: closest

        // 6. Calculate the Reference point
        float sum_x = 0;
        int num_points = 0;

        // Update Line min & Line max
        float line_start_y = 0;
        float line_end_y = 1000;
        pcl::PointCloud<pcl::PointXYZ> line_cloud;

        for (int i = 0; i < (*cloud_cluster).size(); i++)
        {
            sum_x += cloud_cluster->points[i].x;
            num_points ++;
            if (line_end_y > cloud_cluster->points[i].y)
                line_end_y = cloud_cluster->points[i].y;
            if (line_start_y < cloud_cluster->points[i].y)
                line_start_y = cloud_cluster->points[i].y;
        }
        pcl::PointXYZ reference (sum_x / (float)num_points, (line_start_y + line_end_y)/2, 0);//Jinsuk
        point_set.push_back(reference); // [1]: reference

        pcl::PointXYZ line_start_point (sum_x / (float)num_points, line_start_y, 0);
        pcl::PointXYZ line_end_point (sum_x / (float)num_points, line_end_y, 0);
        point_set.push_back(line_start_point); //[2]: line start point
        point_set.push_back(line_end_point); //[3]: line end point



        // Publish ROS Topics
        sensor_msgs::PointCloud2 points_msg;
        sensor_msgs::PointCloud2 points_line;

        pcl::toROSMsg((*cloud_cluster), points_line);
        pcl::toROSMsg(point_set, points_msg);
        points_line.header.frame_id = scan_msg->header.frame_id;
        points_msg.header.frame_id = scan_msg->header.frame_id;
        this->pub_points_.publish(points_msg);
        this->pub_line_.publish(points_line);

        //cmd
        near_y_ = cloud_cluster->points[nn_indices[0]].y;
        ref_y_ = reference.y;
        ref_x_ = reference.x;
        line_start_y_ = line_start_point.y;
        line_end_y_ = line_end_point.y;
    }
    void cmdtimercallback(const ros::TimerEvent &event)
    {
        std_msgs::Empty EmptyMsg;
        geometry_msgs::Twist cmd_vel;
        int Mode_type;
        HJ_mode_cnt++;

        //To stop when the communication is delayed or failed
        if(config_.Postech_code_)
            Mode_type  = postech_mode_;
        else
        {
//            if(HJ_mode_cnt >=10) //If there is no mode subscribe, the mode stops. It should be deleted
//                HJ_mode_low = STOP_MODE;
            Mode_type = HJ_mode_low;
        }
        //To interrupt mode_type, program here->.
        //To operate the gmapping

	
        // minwook
        if(config_.amcl_driving_ ==false)
        {
            Mode_type = AUTO_LIDAR_MODE;
            if(!gmapping_go)
                return;
        }
        //minwook ////////////////////////////////////////////////
        //Mode_type = AUTO_LIDAR_MODE;
        //minwook ////////////////////////////////////////////////
        
        if(joy_driving_ || HJ_mode_low == MANUAL_MODE)
            Mode_type = MANUAL_MODE;

        if(init_call || init_cnt !=0)
        {
            init_call = false; //Without this, it keeps initializing due to delayed subscribe from loclizaiton node.
            init_cnt++;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.z = 0.0;
            pub_cmd_.publish(cmd_vel);
            if(init_cnt==INIT_WAIT)
            {
                system("rosservice call /pose_update 0.0 0.0 0.0"); //Intialize AMCL
            }
            else if(init_cnt==INIT_WAIT*3)
            {
                system("rosservice call /reset_odom"); //Intialize IMU
            }
            else if(init_cnt==INIT_WAIT*6)
            {
                system("rosservice call /odom_init 0.0 0.0 0.0"); //Intialize Encoder
            }
            if(init_cnt==INIT_WAIT*12)
                init_cnt =0;
            return;
        }

        //// 2. Autonomous Driving
        float y_err_local = ref_y_ - near_y_;
        cmd_vel.linear.x = config_.linear_vel_;
	    double temp_angular_max;
        // 2.1 Check Obstacles
        if (config_.check_obstacles_ && (Mode_type == AUTO_LIDAR_MODE || Mode_type == AUTO_IMAGE_MODE))
        {
            float line_length = line_end_y_ - line_start_y_;
            //bool is_obs_in_aisle = obs_y_ > line_end_y_ && obs_y_ < line_start_y_;
            // (0) Front Obstacle Update

           // std::cout << "Obs_x : " << obs_x_ << ", Obs_y : " << obs_y_  << "Line_start : "<<line_start_y_<<", Line_end : "<<line_end_y_<<std::endl; 

            if ((obs_x_ < config_.front_obs_ && abs(obs_y_) < config_.robot_width_/3)
                ||(obs_x_ < config_.front_obs_2_ && abs(obs_y_) < config_.robot_width_/2))
            {
                if(obs_x_ <config_.obs_double_check_dist_){
                    double_check = true;
                }
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
		        was_obs_in_aisle = false;
                pub_cmd_.publish(cmd_vel);
                std::cout<<"Front obstacle is deteced"<<std::endl;
                return;
            } 
            else if (config_.obs_double_check_ && double_check == true){ //when obstacle disappear, slightly go back to find obstacle
                cmd_vel.linear.x = -0.1;
		        cmd_vel.angular.z = 0.0;
                pub_cmd_.publish(cmd_vel);
                backward_length += cmd_vel.linear.x * TIMER_SAMPLE;
		        std::cout<<"backward: "<<backward_length<<std::endl;
                if (obs_x_ < config_.front_obs_ || backward_length <config_.backward_length_){
                    std::cout<<"-------------backward_init------------"<<std::endl;
			        double_check =false;
                    backward_length =0;
		        }
		        return;
            }
            // (1) Right Obstacle Update	(y:오른쪽이 음수)
            else if(obs_y_ < 0 && obs_y_ > -1 && obs_x_< 1)
            {
                cmd_vel.linear.x = config_.obs_vel_ratio_*config_.linear_vel_;
                std::cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                y_err_local = ref_y_-config_.obs_avoidance_distance_ - near_y_;

                temp_y_err_local = 1;
                was_obs_in_aisle = true;
                spare_length = 0;
            }
            // (2) Left Obstacle Update (y:왼쪽이 양수)
            else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_< 1)
            {
                cmd_vel.linear.x =config_.obs_vel_ratio_*config_.linear_vel_;
                std::cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                y_err_local = ref_y_+config_.obs_avoidance_distance_ - near_y_;
                
                temp_y_err_local = -1;
                was_obs_in_aisle = true;
                spare_length = 0;
            }
            
            // (3) After obs disappear, go further 'spare_length'
            else if(was_obs_in_aisle)
            {
                cmd_vel.linear.x =config_.obs_vel_ratio_*config_.linear_vel_;
                spare_length += cmd_vel.linear.x * TIMER_SAMPLE;
                //y_err_local = temp_y_err_local;
                if(temp_y_err_local ==1)
                    y_err_local = ref_y_ - config_.obs_avoidance_distance_ - near_y_;
                else if (temp_y_err_local == -1)
                    y_err_local = ref_y_ + config_.obs_avoidance_distance_ - near_y_;
                std::cout<< "go straight for spare distance" <<std::endl;
                if(spare_length > config_.spare_length_)
                {
                    spare_length = 0;
                    was_obs_in_aisle = false;
                    std::cout<<"spare finish"<<std::endl;
                }
            }
        }

        float straight_start_x = 1086.27382;
        float straight_start_y = 13.8423;
        float straight_end_x = 1103.68392;
        float straight_end_y = 15.78658;
        int state_of_straight = DecisionStraight(straight_start_x,straight_start_y,straight_end_x,straight_end_y,global_x,global_y);

        switch(Mode_type)
        {
        case MANUAL_MODE:
            break;
        case STOP_MODE:
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.z = 0.0;
            pub_cmd_.publish(cmd_vel);
            break;

        case AUTO_LIDAR_MODE:
            //RP driving
            cmd_vel.linear.x = StopControl(cmd_vel.linear.x,global_x,global_y,global_c_theta_);
            cmd_vel = RPLidarControl(y_err_local,cmd_vel.linear.x);

            //straight moving            
#if ENABLE_STRAIGHT_CONTROL
            if(state_of_straight==1)
		    {
                cmd_vel = PointToPointControl(straight_end_x,straight_end_y,global_x,global_y,global_c_theta_);
		        ROS_INFO("Straight control forward");
		    }
            else if(state_of_straight ==2)
            {
                cmd_vel = PointToPointControl(straight_start_x,straight_start_y,global_x,global_y,global_c_theta_);
		        ROS_INFO("Straight control forward");
            }
#endif
            pub_cmd_.publish(cmd_vel);
            break;

        case TURN_MODE:
            cmd_vel = TurningControl(global_x_err_,global_y_err_,global_r_theta_,global_c_theta_);
            pub_cmd_.publish(cmd_vel);
            break;

        case AUTO_PRE_LIDAR_MODE:
            align_cnt++;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local;
            if(cmd_vel.angular.z< config_.min_rot_ && cmd_vel.angular.z>0)//To rotate minimum speed at cw
                cmd_vel.angular.z = config_.min_rot_;
            else if(cmd_vel.angular.z> -config_.min_rot_ && cmd_vel.angular.z<0) //To rotate minimum speed at ccw
                cmd_vel.angular.z = -config_.min_rot_;
            if(cmd_vel.angular.z> config_.max_rot_)
                cmd_vel.angular.z = config_.max_rot_;
            else if(cmd_vel.angular.z< -config_.max_rot_)
                cmd_vel.angular.z = -config_.max_rot_;

            pub_cmd_.publish(cmd_vel);

            if(y_err_local < 0.02 || align_cnt >=70 )//0.05 cm
            {
                align_cnt=0;
                pub_prelidar_end_.publish(EmptyMsg);
            }
            break;

        case DOCK_IN_MODE: //Hanjeon gives us the err of y
            //            cmd_vel.linear.x = -0.1;
            //            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local;

            //            if(cmd_vel.linear.x> config_.max_vel_)
            //                cmd_vel.linear.x = config_.max_vel_;
            //            else if(cmd_vel.linear.x< -config_.max_vel_)
            //                cmd_vel.linear.x = -config_.max_vel_;

            //            if(cmd_vel.angular.z> config_.max_rot_)
            //                cmd_vel.angular.z = config_.max_rot_;
            //            else if(cmd_vel.angular.z< -config_.max_rot_)
            //                cmd_vel.angular.z = -config_.max_rot_;

            //pub_cmd_.publish(cmd_vel);
            break;
        case DOCK_OUT_MODE: // with RPlidar
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local/0.6;

            if(cmd_vel.linear.x> config_.max_vel_)
                cmd_vel.linear.x = config_.max_vel_;
            else if(cmd_vel.linear.x< -config_.max_vel_)
                cmd_vel.linear.x = -config_.max_vel_;

            if(cmd_vel.angular.z> config_.max_rot_)
                cmd_vel.angular.z = config_.max_rot_;
            else if(cmd_vel.angular.z< -config_.max_rot_)
                cmd_vel.angular.z = -config_.max_rot_;

            //pub_cmd_.publish(cmd_vel);


            break;

        default:
            break;
        }
        //cout<<"3. cmd_vel = "<<cmd_vel<<endl;
        // ros::Time ros_end_time = ros::Time::now();
        // double end_time = ros_now_time.toSec();
        // dt = end_time - now_time;
        // ROS_INFO("Current, during TIME: %f, %f",now_time,dt);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        int temp_value;
        std_msgs::Int32 joy_msg_to_node;
        //B
        if (joy_msg->buttons[1] == 1 && switch_flag1 ==0)
        {
            switch_flag1 = 1;
            std::cout<<"(push) B "<<std::endl;
            joy_driving_ = !joy_driving_;
            gmapping_go = true; //Keep TRUE

            if(joy_driving_==true)
            {
                ROS_INFO("Joystick mode");
                joy_msg_to_node.data=1;
                pub_docking_end_.publish(joy_msg_to_node);
            }
            else if(joy_driving_==false)
            {
                joy_msg_to_node.data=100;
                pub_docking_end_.publish(joy_msg_to_node);
            }
        }
        else if(joy_msg->buttons[1] == 0 && switch_flag1 ==1)
        {
            switch_flag1 =0;
        }

        //A
        if (joy_msg->buttons[0] == 1 && switch_flag0 ==0)
        {
            switch_flag0 = 1;
            std::cout<<"(push) A "<<std::endl;
		rplidar_max_on = !rplidar_max_on; 

            joy_msg_to_node.data=0;
            pub_docking_end_.publish(joy_msg_to_node);
        }
        else if(joy_msg->buttons[0] == 0 && switch_flag0 ==1)
        {
            switch_flag0 =0;
        }
        
        //X
        temp_value = 3;
        if(joy_msg->buttons[temp_value] == 1 && switch_flag[temp_value] ==0)	 //
        {
            switch_flag[temp_value] = 1;
            std::cout<<"(push) X "<<std::endl;

            joy_msg_to_node.data=temp_value;
            pub_docking_end_.publish(joy_msg_to_node);
        }
        else if(joy_msg->buttons[temp_value] == 0 && switch_flag[temp_value] ==1)
        {
            switch_flag[temp_value] =0;
        }

        //Y
        temp_value = 4;
        if(joy_msg->buttons[temp_value] == 1&& switch_flag[temp_value] ==0)	 //
        {
            switch_flag[temp_value] = 1;
            std::cout<<"(push) Y "<<std::endl;

            joy_msg_to_node.data=temp_value;
            pub_docking_end_.publish(joy_msg_to_node);
        }
        else if(joy_msg->buttons[temp_value] == 0 && switch_flag[temp_value] ==1)
        {
            switch_flag[temp_value] =0;
        }
        //Velocity maximum
        // left right
        if ((joy_msg->axes[6] == 1 || joy_msg->axes[6] == -1) && axes_flag[6] ==0)
        {
            axes_flag[6] = 1;
            //std::cout<<"(push) axes up "<<std::endl;
            if(joy_msg->axes[6] == 1)
                manual_ratio += 0.1;
            else if(joy_msg->axes[6] == -1)
                manual_ratio -= 0.1;
            if(manual_ratio >= 1.5)
                manual_ratio = 1.5;
            else if(manual_ratio <= 0.5)
                manual_ratio = 0.5;
	    std::cout<<"manul velocity: "<< manual_ratio<<std::endl;
        }
        else if(joy_msg->axes[6] == 0 && axes_flag[6] ==1)
        {
            axes_flag[6] =0;
        }

        // up down
        if ((joy_msg->axes[7] == 1 || joy_msg->axes[7] == -1) && axes_flag[7] ==0)
        {
            axes_flag[7] = 1;
            //std::cout<<"(push) axes up "<<std::endl;
            if(joy_msg->axes[7] == 1)
                config_.linear_vel_ += 0.1;
            else if(joy_msg->axes[7] == -1)
                config_.linear_vel_ -= 0.1;
            if(config_.linear_vel_ >= 1.5)
                config_.linear_vel_ = 1.5;
            else if(config_.linear_vel_ <= 0.2)
                config_.linear_vel_ = 0.2;
	   std::cout<<"auto velocity: "<< config_.linear_vel_ <<std::endl;
        }
        else if(joy_msg->axes[7] == 0 && axes_flag[7] ==1)
        {
            axes_flag[7] =0;
        }

        if(joy_driving_)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = joy_msg -> axes[1] * manual_ratio;
            cmd_vel.angular.z = joy_msg -> axes[0] * manual_ratio;
            pub_cmd_.publish(cmd_vel);
            return;
        }

    }

    void obsCallback(const std_msgs::Float32MultiArray::ConstPtr& dists_msg)
    {
        if (dists_msg->data.size())
        {
            obs_x_ = dists_msg->data[0];
            obs_y_ = dists_msg->data[1];
        }
        else
        {
            obs_x_ = 1000000;
            obs_y_ = 1000000;
        }
    }

    void SpeedCallback(const std_msgs::Float32::ConstPtr& speed_msgs)
    {
        config_.linear_vel_ = speed_msgs->data;
    }


    void localDataCallback(const std_msgs::Float32MultiArray::ConstPtr& local_msgs)
    {
        postech_mode_ = local_msgs->data[0];

        global_x_err_ = local_msgs->data[1];
        global_y_err_ = local_msgs->data[2];
        global_r_theta_ = local_msgs->data[3];
        global_c_theta_ = local_msgs->data[4];

        init_call = local_msgs->data[5];

        docking_out_flag = local_msgs->data[6];

        local_data_receive =true;

        global_x = local_msgs->data[7];
        global_y = local_msgs->data[8];
    }

    void modeCallback(const std_msgs::Int32::ConstPtr &Mode_value)
    {
        HJ_mode_low = Mode_value->data;
        HJ_mode_cnt =0;
    }
    float StopControl(float current_vel, float current_x, float current_y, float ct);
    geometry_msgs::Twist RPLidarControl(float y_err_tunnel,float v_x);
    geometry_msgs::Twist PointToPointControl(float g_x, float g_y, float x, float y, float ct);
    geometry_msgs::Twist TurningControl(float turn_x_err_, float turn_y_err_, float turn_r_theta_, float turn_c_theta_);
    int DecisionStraight(float s_x, float s_y, float e_x, float e_y, float current_x, float current_y);
    float PointDistance(float x1, float y1, float x2, float y2);
    void getfield(char* line, csv_data *d, int end_idx);
    void remove_spaces(char* s);
    void ReadCSV(void);
    
private:
    // Publisher & Subscriber
    ros::Subscriber sub_scan_;
    ros::Timer timer_cmd_node;
    ros::Publisher pub_line_;
    ros::Publisher pub_points_;
    ros::Publisher pub_prelidar_fail_;
    ros::Publisher pub_lidar_fail_;

    ros::Subscriber sub_joy_;//Joystic
    ros::Subscriber sub_obs_dists_;//Obs
    ros::Subscriber sub_aisle_;//RPLidar for aisle
    ros::Subscriber sub_localization_;//From localization node
    ros::Subscriber sub_driving_;
    ros::Subscriber sub_mode_call_;
    ros::Subscriber sub_speed_;//For reference speed
    ros::Subscriber sub_gmapping;

    ros::Publisher pub_cmd_;
    ros::Publisher pub_prelidar_end_;
    ros::Publisher pub_docking_end_;

    bool joy_driving_ = false; // even: auto, odd: joy control
    // Obs
    float obs_x_ = 10000;
    float obs_y_ = 10000;
    bool was_obs_in_aisle = false;
    double spare_length = 0;
    float temp_y_err_local = 0;
    float pre_y_err = 0.0;
    float Max_speed = 0.5;
    unsigned int align_cnt=0;
    bool double_check = false;
    float backward_length = 0;

    // Amcl
    float global_dist_err_ = 0;
    float global_ang_err_ = 0;
    float global_x_err_ =0;
    float global_y_err_ =0,global_r_theta_=0.0,global_c_theta_=0.0;
    float global_x=0.0, global_y=0.0;


    float liney_pose=0;
    float Docking_speed=0;

    bool init_call = false;
    unsigned int init_cnt=0;
    bool once_flag = false;
    bool RP_MODE= true;
    bool local_data_receive = false;
    unsigned int gmapping_cnt=0;
    unsigned int gmapping_start_cnt=0;

    // Aisle
    float line_start_y_ = -30;
    float line_end_y_ = 30;
    float ref_y_ = 0;
    float ref_x_ = 0;
    float near_y_ = 0;

    vector<geometry_msgs::PoseStamped> map_data;
    //int fid_ID=0;
    //float fid_area=0;
    int postech_mode_=0;
    int HJ_mode_low=STOP_MODE;
    unsigned int HJ_mode_cnt =0;

    //
    bool is_rotating_ = false;
    bool is_arrived_ = true;
    //bool is_linetracking = false;
    bool gmapping_go =false;

	bool rplidar_max_on = true;

    unsigned int switch_flag0 = true;
    unsigned int switch_flag1 = true;
    unsigned int switch_flag[10] ={true};
    bool axes_flag[7] ={true};
    float manual_ratio = 1.0;
    unsigned int odom_update_cnt=0;//To update encoder odom while mapping
    float docking_out_flag=0;
    /** configuration parameters */
    typedef struct
    {
        double line_thresh_;
        double aisle_width_;

        double Kpx_param_;
        double Kpy_param_;
        double Kpy_param_rot_;
	double Kpx_str_;
	double Kpy_str_;
	double Kpy_str_rot_;

        double tunnel_gain_boundary_;
        double Kpy_param_boundary_gain_;
        double theta_ratio_;

        double linear_vel_;
        double obs_vel_ratio_;
        double robot_width_;
        double front_obs_;
        double front_obs_2_;
        double spare_length_;
        bool check_obstacles_;
        double obs_avoidance_distance_;
        bool obs_double_check_;
        double obs_double_check_dist_;
        double backward_length_;

        bool amcl_driving_;
        double rot_kx_;
        double rot_ky_;
        double rot_kt_;
        double min_vel_;
        double min_rot_;
        double max_vel_;
        double max_rot_;
        bool Postech_code_;
        bool ENABLE_STRAIGHT_CONTROL_;
    } Config;
    Config config_;
vector<geometry_msgs::PoseStamped> pos_junction_box;
};
float AisleDetectNode::StopControl(float current_vel, float current_x, float current_y, float ct)
{
	static bool only_one=true;
	if(only_one)
	{
		only_one = false;

    
    geometry_msgs::PoseStamped pos_junction_box_;
    pos_junction_box_.pose.position.x = 50.55;
    pos_junction_box_.pose.position.y = 0.41;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 1 postion



    pos_junction_box_.pose.position.x = 59.55;
    pos_junction_box_.pose.position.y = 0.56;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 2 postion
    
    pos_junction_box_.pose.position.x = 68.53;
    pos_junction_box_.pose.position.y = 0.71;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 3 postion

    pos_junction_box_.pose.position.x = 80.48;
    pos_junction_box_.pose.position.y = 0.96;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 4 postion

    pos_junction_box_.pose.position.x = 85.00;
    pos_junction_box_.pose.position.y = 1.01;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 5 postion

    pos_junction_box_.pose.position.x = 89.44;
    pos_junction_box_.pose.position.y = 1.10;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 6 postion

    pos_junction_box_.pose.position.x = 93.97;
    pos_junction_box_.pose.position.y = 1.23;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 7 postion

    pos_junction_box_.pose.position.x = 98.46;
    pos_junction_box_.pose.position.y = 1.31;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 8 postion

    pos_junction_box_.pose.position.x = 102.99;
    pos_junction_box_.pose.position.y = 1.43;
    pos_junction_box.push_back(pos_junction_box_); //Junction box 9 postion
}
    cout<<"pos_junction_box.size() = "<<pos_junction_box.size()<<endl;
    //for(int i = 0; i < pos_junction_box.size(); i++){
    static unsigned int i;
    
    double junction_err_x = -current_x + pos_junction_box[i].pose.position.x;
    double junction_err_y = -current_y + pos_junction_box[i].pose.position.y;
    double junction_err_dist = sqrt(junction_err_x*junction_err_x + junction_err_y*junction_err_y);

    //cout<<"pos_junction_box[i].pose.position.x = "<<pos_junction_box[i].pose.position.x<<endl;
    //cout<<"pos_junction_box[i].pose.position.y = "<<pos_junction_box[i].pose.position.y<<endl;
    //cout<<"junction_err_dist = "<<junction_err_dist<<endl;

    float temp_vel=0.0;
	static unsigned int stop_cnt_l=0;
	static bool stop_flag_l =false;

    float straight_l_xerr= junction_err_x *cos(ct) + junction_err_y *sin(ct);
    float straight_l_yerr= -junction_err_x *sin(ct) + junction_err_y *cos(ct);


	if(stop_flag_l)
	{
		if(stop_cnt_l<JUNCTION_STOP_SIZE)
		{
			stop_cnt_l++;
			cout<<"Next Junction stop : "<<i<<endl;
			return 0.0;
		}
		else
		{
			stop_cnt_l=0;
			stop_flag_l = false;
		}
	}
    if(junction_err_dist < 0.05)
    {
        cout<<"[1] Motion : @@@ Stop for Juction Box Number : ["<<i<<"] @@@"<<endl;
        i++;
        i%=NUM_JUNCTION;
	stop_flag_l = true;
        return 0.0;
    }    
    else if(junction_err_dist < 2.0)
    {
        //temp_vel = junction_err_dist*current_vel/2;
        temp_vel = straight_l_xerr*current_vel/2;
	if(temp_vel >= current_vel)
            temp_vel = current_vel;
        else if(temp_vel <= -current_vel)
            temp_vel = -current_vel;

        return temp_vel;
    }
	else
		return current_vel;
}
geometry_msgs::Twist AisleDetectNode::RPLidarControl(float y_err_tunnel,float v_x)
{
    static float prev_yerr= 0.0;
    float temp_angular_max=0.0;
    float tunnel_angle_diff;
    geometry_msgs::Twist tmp_cmd_vel;

    tunnel_angle_diff = atan2(y_err_tunnel,0.2);
    tmp_cmd_vel.angular.z = -config_.Kpy_param_ * tunnel_angle_diff - config_.Kpy_param_rot_*(tunnel_angle_diff - prev_yerr)/TIMER_SAMPLE;
    prev_yerr = tunnel_angle_diff;

    //Saturation parts due to Zero's deadline from VESC
    tmp_cmd_vel.linear.x = v_x;
    if(tmp_cmd_vel.linear.x< config_.min_vel_ && tmp_cmd_vel.linear.x>0)
        tmp_cmd_vel.linear.x = config_.min_vel_;


    if(tmp_cmd_vel.linear.x> config_.max_vel_)
        tmp_cmd_vel.linear.x = config_.max_vel_;
    else if(tmp_cmd_vel.linear.x< -config_.max_vel_)
        tmp_cmd_vel.linear.x = -config_.max_vel_;

    if(tmp_cmd_vel.angular.z> config_.max_rot_)
        tmp_cmd_vel.angular.z = config_.max_rot_;
    else if(tmp_cmd_vel.angular.z< -config_.max_rot_)
        tmp_cmd_vel.angular.z = -config_.max_rot_;
/*    if(fabs(cmd_vel.angular.z) >RATIO_VEL)
        temp_angular_max = RATIO_VEL;
    else
        temp_angular_max = fabs(cmd_vel.angular.z);
*/
	if(rplidar_max_on)
	{
    temp_angular_max = fabs(tmp_cmd_vel.angular.z)/config_.max_rot_/3;
    tmp_cmd_vel.linear.x = tmp_cmd_vel.linear.x*(1.0- temp_angular_max);
	}
    //std::cout<<"x: "<<cmd_vel.linear.x<< ", z: " << cmd_vel.angular.z<<std::endl;
    //Saturation of 'cmd_vel.linear.x' doesn't need because when the x fits, it's not necessary to move
    return tmp_cmd_vel;
}
geometry_msgs::Twist AisleDetectNode::PointToPointControl(float g_x, float g_y, float x, float y, float ct)
{ 
    geometry_msgs::Twist tmp_cmd_vel;
    const float Kpy_control =1.5, Kpt_control =2.5;
    float straight_l_xerr;
    float straight_l_yerr;
    float x_err = g_x -x;
    float y_err = g_y -y;
    float yaw_err = 0.0;
    float g_rt=0.0;
    
    straight_l_xerr= x_err *cos(ct) + y_err *sin(ct);
    straight_l_yerr= -x_err *sin(ct) + y_err *cos(ct);
    g_rt = atan2(y_err, x_err);
    yaw_err = g_rt-ct;
    if(yaw_err > M_PI)
        yaw_err -= 2*M_PI;
    else if(yaw_err < -M_PI)
        yaw_err += 2*M_PI;

    tmp_cmd_vel.linear.x = config_.linear_vel_*straight_l_xerr; 
    tmp_cmd_vel.angular.z = config_.Kpy_str_*straight_l_yerr + config_.Kpy_str_rot_*yaw_err;

    if(tmp_cmd_vel.linear.x< config_.min_vel_ && tmp_cmd_vel.linear.x>0)
        tmp_cmd_vel.linear.x = config_.min_vel_;



    if(tmp_cmd_vel.linear.x> 0.5)
        tmp_cmd_vel.linear.x = 0.5;
    else if(tmp_cmd_vel.linear.x< -0.5)
        tmp_cmd_vel.linear.x = -0.5;

    if(tmp_cmd_vel.angular.z> config_.max_rot_)
        tmp_cmd_vel.angular.z = config_.max_rot_;
    else if(tmp_cmd_vel.angular.z< -config_.max_rot_)
        tmp_cmd_vel.angular.z = -config_.max_rot_;



    return tmp_cmd_vel;
}
geometry_msgs::Twist AisleDetectNode::TurningControl(float turn_x_err_, float turn_y_err_, float turn_r_theta_, float turn_c_theta_)
{
    geometry_msgs::Twist tmp_cmd_vel;
    float l_xerr, l_yerr;
    l_xerr= turn_x_err_ *cos(turn_c_theta_) + turn_y_err_ *sin(turn_c_theta_);
    l_yerr= -turn_x_err_ *sin(turn_c_theta_) + turn_y_err_ *cos(turn_c_theta_);
    //std::cout<< "static_x: " <<l_xerr <<", static_y:" << l_yerr <<std::endl;
    l_xerr = (tan(turn_r_theta_)*-turn_x_err_ -turn_y_err_)/(sqrt(tan(turn_r_theta_)*tan(turn_r_theta_)+(float)1));//new l_xerr


    tmp_cmd_vel.linear.x = config_.rot_kx_;//config_.rot_kx_*l_xerr;
    tmp_cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(turn_r_theta_ - turn_c_theta_);

/*
    if(fabs(global_c_theta_ - global_c_theta_)< 0.4 || fabs(global_c_theta_ - global_c_theta_)> 2.7)
    {
        cmd_vel.linear.x = config_.rot_kx_*l_xerr;
        cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);
    }
    else
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = config_.rot_ky_ *l_yerr + config_.rot_kt_ *(global_r_theta_ - global_c_theta_);
    }
*/


    //Saturation parts due to Zero's deadline from VESC
    //Saturation of 'cmd_vel.linear.x' doesn't need because when the x fits, it's not necessary to move
    if(tmp_cmd_vel.angular.z< config_.min_rot_ && tmp_cmd_vel.angular.z>0)//To rotate minimum speed at cw
        tmp_cmd_vel.angular.z = config_.min_rot_;
    else if(tmp_cmd_vel.angular.z> -config_.min_rot_ && tmp_cmd_vel.angular.z<0) //To rotate minimum speed at ccw
        tmp_cmd_vel.angular.z = -config_.min_rot_;

    if(tmp_cmd_vel.angular.z> config_.max_rot_)
        tmp_cmd_vel.angular.z = config_.max_rot_;
    else if(tmp_cmd_vel.angular.z< -config_.max_rot_)
        tmp_cmd_vel.angular.z = -config_.max_rot_;
        
    return tmp_cmd_vel;
}
int AisleDetectNode::DecisionStraight(float s_x, float s_y, float e_x, float e_y, float current_x, float current_y)
{
    if(config_.amcl_driving_ ==false)
        return 0;

    float mid_x = (s_x + e_x)/2;
    float mid_y = (s_y + e_y)/2;
    float distance = PointDistance(mid_x,mid_y,s_x,s_y);
    float current_distance = PointDistance(mid_x,mid_y,current_x,current_y);
    float end_distance = PointDistance(e_x,e_y,current_x,current_y);
    static bool start_control = false;
    static int direction_straight = FORWARD_DRIECTION;
/*    
    // if(end_distance<0.1 || distance-current_distance<0)
    if(current_distance - distance > 0.1)
        return 0;
    else
        return 1;
*/
    if(current_distance - distance > 0.1)
    {
        start_control = true;
        return 0;
    }
    else
    {
        if(start_control)
        {
            start_control = false;
            if(distance < end_distance)
                direction_straight = FORWARD_DRIECTION;
            else
                direction_straight = BACKWARD_DRIECTION;;
        }
        return direction_straight;
    }
}
float AisleDetectNode::PointDistance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1 - x2)*(x1 - x2)+(y1 - y2)*(y1 - y2));
}
void remove_spaces(char* s) {
    const char* d = s;
    do {
        while (*d == ' ') {
            ++d;
        }
    } while (*s++ = *d++);
}
void getfield(char* line, csv_data *d, int end_idx){
    int idx = 0;
    char *token = strtok(line, ",");
    do{
        strcpy(d->s[idx++], token);
    }
    while ( idx != end_idx && (token = strtok(NULL, ",")));
}
void AisleDetectNode::ReadCSV(){
    FILE* stream = fopen("/home/cocel/mapping_ws/src/FAST_LIO_KEPLAR/PCD/path011922_14_54.csv", "r"); // have to change current map csv file

    char line[1024];
    csv_data d;
    while (fgets(line, 1024, stream)){
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
}
PLUGINLIB_EXPORT_CLASS(auto_driving::AisleDetectNode, nodelet::Nodelet);
