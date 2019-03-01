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

	if (n_.hasParam(ros::this_node::getName() + "/threshold_in_out"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_in_out", threshold_in_out_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_in_out not found.\
			Using Default");
		threshold_in_out_ = 80.0;
	}
	
	if (n_.hasParam(ros::this_node::getName() + "/threshold_complexity"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_complexity", threshold_complexity_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold_complexity not found.\
			Using Default");
		threshold_complexity_ = 30.0;
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

	for(int i=1; i<ranges_size_; i++)
	{
		//! Search for big changes between 2 ranges, where a wall,     
	    //! a feature or a door could end/start. If one is detected,    
	    //! then raise temp_var (temporary variance) by 1		 
		if((msg_laser->ranges[i] > (1.35)*msg_laser->ranges[i-1]) || (msg_laser->ranges[i] < (0.65)*msg_laser->ranges[i-1]))
		{			
			if((msg_laser->ranges[i-1] > msg_laser->range_max) && (msg_laser->ranges[i] > msg_laser->range_max))
			{
				; //! Do nothing
			}else
			{
				ROS_INFO("*ranges[%d]: %.2f --- ranges[%d]: %.2f", i, msg_laser->ranges[i], i-1, msg_laser->ranges[i-1]);
				temp_var_++;
			}
		}

		//! Count infinite values
		if(msg_laser->ranges[i] > msg_laser->range_max)
			counter_inf_++;
	}

	temp_features_ = ranges_size_- 1 - counter_inf_; 
	averageValues();
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

	//! Temp variable to store the element of the list that is going to be overriden
	int previous;

	//! Calculate average variance so far
	previous = list_vars_[index_];
	list_vars_[index_] = temp_var_;
	sum_vars_ += list_vars_[index_] - previous;
	average_var_ = (float)sum_vars_/samples_;

	//! Calculate average features' sizes so far
	previous = list_features_[index_];
	list_features_[index_] = temp_features_;
	sum_features_ += list_features_[index_] - previous;
	average_features_ = (float)sum_features_/samples_;
	in_out_ = 100*(average_features_/ (ranges_size_-1));

	ROS_INFO("*[threshold_variance_] %.2f", threshold_variance_);
	ROS_INFO("*[threshold_in_out_] %.2f%%", threshold_in_out_);
	ROS_INFO("*[threshold_complexity_] %.2f%%", threshold_complexity_);
	ROS_INFO("*[index_] %d", index_);
	ROS_INFO("*[samples_] %d", samples_);
	ROS_INFO("*[temp_var_] %d", temp_var_);
	ROS_INFO("*[average_var_] %.2f", average_var_);
	ROS_INFO("*[temp_features_] %d", temp_features_);
	ROS_INFO("*[in_out_] %.2f%%", in_out_);
	ROS_INFO("*************************************************");
	
	//! Raise the index
	index_++;
	caseCheck();
}

void RecognitionLidar::caseCheck(void)
{
	string in_out;
	string variance;
	if(in_out_ <= threshold_in_out_)
	{
		in_out = "|| In/Out: Outdoors || Walls: No  ";

		if(average_var_ == 0.0)
			variance = "|| Features: No  ";
		else
			variance = "|| Features: Yes ";
	}
	else
	{
		in_out = "|| In/Out: Indoors  || Walls: Yes ";

		if(average_var_ <= threshold_variance_)
			variance = "|| Features: No  ";
		else
			variance = "|| Features: Yes ";
	}

	string complexity;
	if(average_var_ <= threshold_complexity_)
		complexity = "|| Complexity: Simple  ||";
	else
		complexity = "|| Complexity: Complex ||";
	

	//temp_env_ = variance + in_out;
	temp_env_ = variance + in_out + complexity;
	if(previous_ != temp_env_)
	{
		previous_ = temp_env_;
		msg_env_.data = temp_env_.c_str();
		env_pub_.publish(msg_env_);
	}
}
}