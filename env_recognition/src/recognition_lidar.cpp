#include <env_recognition/recognition_lidar.h>

namespace env_recognition
{ 

/*
*********************
*    Constructor    * 
*********************
*/

RecognitionLidar::RecognitionLidar (void)
{
	initParams();
	initValues();

	scan_sub_ = n_.subscribe("/scan", 10, &RecognitionLidar::laserCallback, this);
	env_pub_ = n_.advertise<std_msgs::String>("/environment", 10);
}

/*
********************
*    Destructor    *  
********************
*/

RecognitionLidar::~RecognitionLidar (void)
{
	ROS_INFO("[RecognitionLidar] Node destroyed.");
}

/*
*****************************************
*    Initialize the necessary values    *  
*****************************************
*/

void RecognitionLidar::initValues (void)
{
	counter_msgs_ = 0;
	sum_vars_ = 0;
	sum_features_ = 0;
	temp_env_ = ""; 
	index_ = 0;
	samples_ = 0;
	case_ = -1;

	for(int i; i<100; i++)
	{
		list_vars_[i] = 0;
		list_features_[i] = 0;
	}
}

/*
***********************************
*    Initialize the parameters    *  
***********************************
*/

void RecognitionLidar::initParams (void)
{
	if (n_.hasParam(ros::this_node::getName() + "/threshold_variance"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_variance", threshold_variance_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_variance not found.\
			Using Default");
		threshold_variance_ = 5;
	}

	if (n_.hasParam(ros::this_node::getName() + "/threshold_density"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_density", threshold_density_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_density not found.\
			Using Default");
		threshold_density_ = 40.0;
	}
}

/*
************************************
*    Get data from laser sensor    * 
************************************
*/

void RecognitionLidar::laserCallback (const sensor_msgs::LaserScan::ConstPtr &msg_laser)
{
	ranges_size_ = msg_laser->ranges.size();
	temp_env_ = ""; 
	temp_var_ = 0;
	temp_features_ = 0;
	counter_inf_ = 0;

	ROS_INFO("*************************************************");
	for(int i=1; i<ranges_size_; i++)
	{
		//! Search for big changes between 2 ranges, where a wall,     
	    //! a feature or a door could end/start. If one is detected,    
	    //! then raise temp_var (temporary variance) by 1		 
		if((msg_laser->ranges[i] > (1.35)*msg_laser->ranges[i-1]) || (msg_laser->ranges[i] < (0.65)*msg_laser->ranges[i-1]))
		{			
			if((msg_laser->ranges[i-1] > msg_laser->range_max) && (msg_laser->ranges[i] > msg_laser->range_max))
			{
				; //do nothing
			}else
			{
				ROS_INFO("*ranges[%d]: %.2f --- ranges[%d]: %.2f", i, msg_laser->ranges[i], i-1, msg_laser->ranges[i-1]);
				temp_var_++;
			}
		}

		//! Count infinite values
		if(msg_laser->ranges[i] > msg_laser->range_max)
		{
			counter_inf_++;
		}
	}

	//! If the robot sees only inf values, then it sees nothing
	if(counter_inf_ == ranges_size_-1)
	{
		if(case_ != 0)
		{
			case_ = 0;
			temp_env_ = "|| Walls: No    || Features: No  || Dense/Sparse: N/A ||";
			ROS_INFO("[counter_inf_] %d", counter_inf_);
			msg_env_.data = temp_env_.c_str();
			env_pub_.publish(msg_env_);
		}
	}else
	{
		temp_features_ = ranges_size_- 1 - counter_inf_; 
		averageValues();
	}
}

/*
************************************************
*    Calculate the average values until now    *  
************************************************
*/

void RecognitionLidar::averageValues (void)
{
	//! For the first 100 messages, raise the samples_ by 1. Then, keep it to 100
	//! Also, reset the index of the list (index_), everytime it reaches 100
	if(counter_msgs_%100 == 0)
		index_ = 0;
	if(counter_msgs_ < 100)
		samples_++;
	counter_msgs_++;

	//! This variable to store the element of the list that is going to be overriden
	int previous;

	//! Calculate average variance so far
	previous = list_vars_[index_];
	list_vars_[index_] = temp_var_;
	sum_vars_ += list_vars_[index_] - (float)previous;
	average_var_ = (float)sum_vars_ / (float)samples_;

	//! Calculate average features' sizes so far
	previous = list_features_[index_];
	list_features_[index_] = temp_features_;
	sum_features_ += list_features_[index_] - (float)previous;
	average_features_ = (float)sum_features_ / (float)samples_;
	density_ = 100*(average_features_/ ((float)ranges_size_-1));

	ROS_INFO("*[threshold_variance_] %.2f", threshold_variance_);
	ROS_INFO("*[threshold_density_] %.2f%%", threshold_density_);
	ROS_INFO("*[index_] %d", index_);
	ROS_INFO("*[samples_] %d", samples_);
	ROS_INFO("*[temp_var_] %d", temp_var_);
	ROS_INFO("*[average_var_] %.2f", average_var_);
	ROS_INFO("*[temp_features_] %d", temp_features_);
	ROS_INFO("*[density_] %.2f%%", density_);
	ROS_INFO("*************************************************");
	
	//! Raise the index
	index_++;
	caseCheck();
}

/*
*******************************************************************
*    Check the type of environment we are into, and publish it    *  
*******************************************************************
*/

void RecognitionLidar::caseCheck(void)
{
	string variance;
	if(average_var_ <= threshold_variance_)
		variance = "|| Walls: Yes   || Features: No  ";
	else
		variance = "|| Walls: Maybe || Features: Yes ";

	string density;
	if(density_ <= threshold_density_)
		density = "|| Density: Sparse ||";
	else
		density = "|| Density: Dense  ||";

	//temp_env_ = variance + density;
	temp_env_ = variance + density;
	if(previous_ != temp_env_)
	{
		previous_ = temp_env_;
		msg_env_.data = temp_env_.c_str();
		env_pub_.publish(msg_env_);
	}
}

}