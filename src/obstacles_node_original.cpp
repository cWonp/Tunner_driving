#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


namespace auto_driving {

class ObstaclesNode : public nodelet::Nodelet {

public:
	ObstaclesNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("obstacle_coefficient", config_.obstacle_coefficient_, 0.005);
		nhp.param("front_obstacle_dist", config_.front_obstacle_dist_, 0.1);

		sub_pointcloud_ = nhp.subscribe("/velodyne_points", 10, &ObstaclesNode::cloudCallback, this);
		pub_obs_ = nhp.advertise<sensor_msgs::PointCloud2> ("/cropped_obs", 10);
        	pub_obs_dists_ = nhp.advertise<std_msgs::Float32MultiArray> ("/obs_dists", 10);
		pub_drain_line_ = nhp.advertise<sensor_msgs::PointCloud2> ("/drain_line", 10);
	};
	
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
	{
    	std_msgs::Float32MultiArray dists;
	dists.data.clear();
		
    	pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
		pcl_conversions::toPCL(*pc_msg, pcl_pc);
		// Convert point cloud to PCL native point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_ptr(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);
		// Create output point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_ptr(new pcl::PointCloud<pcl::PointXYZ>());   	    
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_left(new pcl::PointCloud<pcl::PointXYZ>());  	    
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_right(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_obs_selected(new pcl::PointCloud<pcl::PointXYZ>());

		//Drain line detection using Velodyne
		float h_velo = 0.40;  //0.34	//????????? ????????? ?????? ???????????? ?????? ????????? ????????????, 
		//point cloud?????? ????????? z?????? ???????????? ???????????? ??????, but ?????? ????????? ???????????? ?????? ????????? ?????? ??????????????? ?????? ?????? ????????? ?????? ???????????? ????????? ?????? ????????? ????????? ???????????????????????????
		//????????? ??????????????? ?????? ?????? ????????? ?????? ??? ?????? ????????? ????????? ????????? ?????? ????????? ??? ??????.
		float w_drain = 0.2;
		

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (input_ptr);         
		pass.setFilterFieldName ("z");         
		//pass.setFilterLimits (-(h_velo+0.3), -h_velo);    
		//?????????????????? ????????? ????????? ???????????? ????????? ??????
		//???????????? ????????? point cloud??? ????????????, ?????? ??????????????? ?????? ??? ??? ?????????
		//pass.setFilterLimits (-1, -h_velo);    
		pass.setFilterLimits (-h_velo+0.03,-h_velo+0.09 );
		pass.filter (*output_ptr);      

		// x<2 (front)
		pass.setInputCloud (output_ptr);         
		pass.setFilterFieldName ("x");         
		pass.setFilterLimits (0, 1.8); //2.0   
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_ptr);              

		// y>0 - left
		pass.setInputCloud (output_ptr);         
		pass.setFilterFieldName ("y");         
		pass.setFilterLimits (0, 1.0);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_left);              

		// y<0 - right
		pass.setInputCloud (output_ptr);         
		pass.setFilterFieldName ("y");         
		pass.setFilterLimits (-1.0, 0);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_right);      		

		//RANSAC -???????????????
		pcl::ModelCoefficients::Ptr coefficients_left (new pcl::ModelCoefficients ());
		pcl::ModelCoefficients::Ptr coefficients_right (new pcl::ModelCoefficients ());

		if(output_left->size() > 10 && output_right->size() > 10 ){
			
			//left
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
										
			pcl::SACSegmentation<pcl::PointXYZ> seg;
			seg.setOptimizeCoefficients (true);      
			seg.setInputCloud (output_left);                 
			seg.setModelType (pcl::SACMODEL_PLANE);    
			seg.setMethodType (pcl::SAC_RANSAC);      
			seg.setMaxIterations (1000);              
			seg.setDistanceThreshold (0.01);          
			seg.segment (*inliers, *coefficients_left);    

			pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (output_left);
			extract.setIndices (inliers);
			extract.setNegative (false);//false
			extract.filter (*output_left);
		
			//right		
			//pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
										
			//pcl::SACSegmentation<pcl::PointXYZ> seg;
			seg.setOptimizeCoefficients (true);      
			seg.setInputCloud (output_right);                 
			seg.setModelType (pcl::SACMODEL_PLANE);    
			seg.setMethodType (pcl::SAC_RANSAC);      
			seg.setMaxIterations (1000);              
			seg.setDistanceThreshold (0.01);          
			seg.segment (*inliers, *coefficients_right);    

			//pcl::ExtractIndices<pcl::PointXYZ> extract;
			extract.setInputCloud (output_right);
			extract.setIndices (inliers);
			extract.setNegative (false);//false
			extract.filter (*output_right);
		}
		//else{
		//???????????? ???????????? ?????? ???????????????? 
			//std::cerr << "Not enough point for finding drain line" << std::endl;
		//	return;
		//}

		//fail&safe (?????? ???????????? ?????? ???????????? , y??? 0.85?????? ?????????, ??? ?????? ???????????? ?????? ????????? fail
		/*if(abs(coefficients_left->values[1]) < 0.85 || abs(coefficients_right->values[1]) < 0.85){
			std::cerr << "Wrong direction plane for finding drain line" << std::endl;		
			return;
		}*/


		//????????? ?????? ???????????? ?????? (eg. ax + by + cz + d = 0 ).
	  	/*std::cerr << "Model coefficients_left: " << coefficients_left->values[0] << " " 
		                                  << coefficients_left->values[1] << " "
		                                  << coefficients_left->values[2] << " " 
		                                  << coefficients_left->values[3] << std::endl;

	  	std::cerr << "Model coefficients_right: " << coefficients_right->values[0] << " " 
		                                  << coefficients_right->values[1] << " "
		                                  << coefficients_right->values[2] << " " 
		                                  << coefficients_right->values[3] << std::endl;
		left_angle  = atan(-coefficients_left->values[0] / coefficients_left->values[1]) *180/3.141592;
		right_angle  = atan(-coefficients_right->values[0] / coefficients_right->values[1]) *180/3.141592;
		
		//????????? ?????? ?????? ?????? angle??? (??? ??? ????????????/??? ???) ?????????(ex-4???) ????????? ??? ???????????????
		//????????? angle ????????? 10??? ?????? ?????? return?(????????? ????????? ??????????????????
		std::cerr << "left line angle : " << left_angle <<std::endl;
		std::cerr << "right line angle : " << right_angle <<std::endl;		
		//????????? ????????? ????????? ???????????? ???????????? 1m?????? ??? ?????? ????????? ?????? ??????(1,0,-h_velo)
		left_distance = coefficients_left->values[0] + coefficients_left->values[2]*(-h_velo) + coefficients_left->values[3];
		right_distance = coefficients_right->values[0] + coefficients_right->values[2]*(-h_velo) + coefficients_right->values[3];
		//left dist>0, right >0, 
		//left+right>0??? ???????????????????????? ????????? ???????????? ??????????????? ?????????
		//left+right<0??? ??????????????????????????? ????????? ???????????? ???????????? ?????????
		//?????? wrong direction, not enough point ???????????? count?????????, 5??? ?????? ???????????? ????????? error flag ?????? ??????.
		//????????? ????????? ????????? difference ?????? ???????????? ?????? ?????? ?????? 
		std::cerr << "left line distance : " << left_distance <<std::endl;
		std::cerr << "right line distance : " << right_distance <<std::endl;
		std::cerr << "differnece btw lines : " << left_distance +right_distance <<std::endl;
*/
		//drain	line		
		*output = *output_left + *output_right;


		//OBSTACLE DETECTION

		//z?????? ??????????????? ???????????? xy 2?????? ?????? 2??? ?????????, ?????? ???????????? obs detection
		//obs ?????? ?????????, ?????? ????????? ??????????????? ?????? obs??? ??????
		pass.setInputCloud (input_ptr);         
		pass.setFilterFieldName ("z");         
		pass.setFilterLimits (-(h_velo-0.15), 0.2);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_obs);

		// x<2 (front)
		pass.setInputCloud (output_obs);         
		pass.setFilterFieldName ("x");         
		pass.setFilterLimits (0.5, 2.0);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_obs);  
		
		// |y|<0.3 (left,right)
		pass.setInputCloud (output_obs);         
		pass.setFilterFieldName ("y");         
		pass.setFilterLimits (-0.3, 0.3);    
		//pass.setFilterLimitsNegative (false);  
		pass.filter (*output_obs_selected);  

		/*float temp_x,temp_y;
		int temp_count = 0;
		for(int i = 0; i < output_obs->points.size(); ++i){
			temp_x = output_obs->points[i].x;
			temp_y = output_obs->points[i].y;
			if(coefficients_left->values[0]*temp_x + coefficients_left->values[1]*temp_y + coefficients_left->values[3] -w_drain >0 
			&& coefficients_right->values[0]*temp_x + coefficients_right->values[1]*temp_y + coefficients_right->values[3] +w_drain <0)
			{ 
				output_obs_selected->push_back(output_obs->points[i]);
			}
		}*/


	if(output_obs_selected->size() != 0)
        {
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (output_obs_selected);  
			sor.setMeanK (50);               
			sor.setStddevMulThresh (1.0);    
			sor.filter (*output_obs_selected);        
		    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	    
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (output_obs_selected);  
			std::vector<int> nn_indices(30);
			std::vector<float> nn_dists(30);
			pcl::PointXYZ origin(0, 0, 0);

			tree->nearestKSearch(origin, 30, nn_indices, nn_dists);

			float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
			int count_negative=0, count_positive = 0, count_center = 0;
			float point_x, point_y; 
			//front_obstacle_ = false;
			if(nn_indices.size() > 5)
            {
				for (int i = 0; i < nn_indices.size(); i++)
				{
					point_y = output_obs_selected->points[nn_indices[i]].y;
					point_x = output_obs_selected->points[nn_indices[i]].x;
					
					if(point_y > 0) //obstacles in left side
					{			
						if(min_positive >point_y)
						{
							min_positive = point_y;
							min_positive_point = i;
						}
						count_positive += 1;
					}
					else //obstacles in right side
					{
						if(min_negative < point_y)
						{	
							min_negative = point_y;
							min_negative_point = i;
						}
						count_negative += 1;
					}
				}

				if(count_positive > count_negative) //obstacles in left side
				{
					// shift_position_ = std::min(params_.obstacle_coefficient_/output_obs_selected->points[nn_indices[min_positive_point]].x, 0.01);
					//std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<abs(output_obs_selected->points[nn_indices[min_positive_point]].x) <<std::endl;	
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].x);
					dists.data.push_back(output_obs_selected->points[nn_indices[min_positive_point]].y);
				}
				else	//obstacles in right side
				{	
					// shift_position_ = std::max(-params_.obstacle_coefficient_/output_obs_selected->points[nn_indices[min_negative_point]].x, -0.01);
					//std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<abs(output_obs_selected->points[nn_indices[min_negative_point]].x) <<std::endl;
					dists.data.push_back(output_obs_selected->points[nn_indices[min_negative_point]].x);
					dists.data.push_back(output_obs_selected->points[nn_indices[min_negative_point]].y);
				}	
            
	        }
        }
        	// Convert data type PCL to ROS
		sensor_msgs::PointCloud2 ros_output;
		pcl::toPCLPointCloud2(*output_obs_selected, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, ros_output);

		sensor_msgs::PointCloud2 ros_line_output;
		pcl::toPCLPointCloud2(*output, pcl_pc);
		pcl_conversions::fromPCL(pcl_pc, ros_line_output);

		// Publish the data
		pub_obs_.publish(ros_output);
		pub_obs_dists_.publish(dists);
		pub_drain_line_.publish(ros_line_output);
    }

private:
	ros::Subscriber sub_pointcloud_;
	ros::Publisher pub_obs_;
	ros::Publisher pub_obs_dists_;
	ros::Publisher pub_drain_line_;
	 

	/** configuration parameters */
	typedef struct
	{
		double obstacle_coefficient_;
		double front_obstacle_dist_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::ObstaclesNode, nodelet::Nodelet);

