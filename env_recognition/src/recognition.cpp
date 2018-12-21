#include <env_recognition/recognition.h>

namespace env_recognition
{ 

/*
*********************
*    Constructor    * 
*********************
*/

Recognition::Recognition (void)
{
	initParams();
	initValues();
	scan_sub_ = n_.subscribe("/scan", 10, &Recognition::laserCallback, this);
	env_pub_ = n_.advertise<std_msgs::String>("/environment", 10);
}

/*
********************
*    Destructor    *  
********************
*/

Recognition::~Recognition (void)
{
	ROS_INFO("[Recognition] Node destroyed.");
}

/*
************************************
*    Initialize the parameters.    *  
************************************
*/

void Recognition::initParams (void)
{
	if (n_.hasParam(ros::this_node::getName() + "/threshold"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold", threshold_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold not found.\
			Using Default");
		threshold_ = 5;
	}

	if (n_.hasParam(ros::this_node::getName() + "/threshold_density"))
	{
		n_.getParam(ros::this_node::getName() + "/threshold_density", threshold_density_);
	}  
	else 
	{
		ROS_WARN("[Parameters] Parameter threshold not found.\
			Using Default");
		threshold_density_ = 40;
	}
}

/*
******************************************
*    Initialize the necessary values.    *  
******************************************
*/

void Recognition::initValues (void)
{
	sum_vars_ = 0;
	sum_features_ = 0;
	counter_msgs_ = 0;
	temp_env_ = ""; 
}

/*
*************************************
*    Get data from laser sensor.    *  
*************************************
*/

void Recognition::laserCallback (const sensor_msgs::LaserScan::ConstPtr &msg_laser)
{
	ranges_size_ = msg_laser->ranges.size();
	temp_var_ = 0;
	temp_features_ = 0;
	counter_inf_ = 0;
	ROS_INFO("*************************************************");
	for(int i=2; i<(msg_laser->ranges.size()-1); i++)
	{

		/*
		******************************************************************
		*    Search for big changes between 2 ranges, where a wall,      *
	    *    a feature or a door could end/start. If one is detected,    *
	    *	 then raise temp_var (temporary variance) by 1.			 *
		******************************************************************
		*/
		if((msg_laser->ranges[i] > (1.35)*msg_laser->ranges[i-1]) || (msg_laser->ranges[i] < (0.65)*msg_laser->ranges[i-1]))
		{			
			if((msg_laser->ranges[i-1] > msg_laser->range_max) && (msg_laser->ranges[i] > msg_laser->range_max))
			{
				; //do nothing
			}else
			{
				ROS_INFO("*ranges[%d]: %f --- ranges[%d]: %f", i, msg_laser->ranges[i], i-1, msg_laser->ranges[i-1]);
				temp_var_++;

				//Calculate size of each feature, and add it to sum of all features' sizes
				if((msg_laser->ranges[i] < (0.75)*msg_laser->ranges[i-1]))
				{
					start_ = i;
				}else if((msg_laser->ranges[i] > (1.35))*msg_laser->ranges[i-1])
				{
					if(start_ < msg_laser->ranges[i])
						end_ = i;
						temp_features_ += end_ - start_;
				}
			}
		}else
		{

		}

		//! Count infinite values
		if(msg_laser->ranges[i] > msg_laser->range_max)
		{
			counter_inf_++;
		}
	}

	//! If the robot sees only inf values, then it sees nothing
	if(counter_inf_ == msg_laser->ranges.size()-3)
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
		varianceAverageValue();
	}
}

/*
***************************************************
*    Calculate the average variance until now.    *  
***************************************************
*/

void Recognition::varianceAverageValue (void)
{
	counter_msgs_++;
	//! Every 500 messages, reset average variance
	if(counter_msgs_%500 == 0)
	{	
		counter_msgs_ = 0;
		sum_vars_ = 0;
		sum_features_ = 0;
	}

	//! Calculate average variance so far
	sum_vars_ += temp_var_;
	average_var_ = (float)sum_vars_ / ((float)counter_msgs_);

	//! Calculate average features' sizes so far
	sum_features_ += temp_features_;
	average_features_ += (float)sum_features_ / ((float)counter_msgs_);

	ROS_INFO("*[threshold_] %d", threshold_);
	ROS_INFO("*[counter_msgs_] %d", counter_msgs_);
	ROS_INFO("*[temp_var_] %d", temp_var_);
	ROS_INFO("*[average_var_] %f", average_var_);
	ROS_INFO("*[temp_features_] %d", temp_features_);
	ROS_INFO("*[average_features_] %f", average_features_);
	ROS_INFO("*************************************************");

	if((average_var_ <= threshold_ && average_var_ >= 0))
	{
		if(case_ != 1)
		{
			case_ = 1;
			
			if(100*(average_features_/ranges_size_) > threshold_density_)
				temp_env_ = "|| Walls: Yes   || Features: No  || Density: Dense  ||";
			else
				temp_env_ = "|| Walls: Yes   || Features: No  || Density: Sparse ||";
			
			ROS_INFO("[average_var_] %f", average_var_);
			msg_env_.data = temp_env_.c_str();
			env_pub_.publish(msg_env_);
		}
	}else if((average_var_ > threshold_))
	{
		if(case_ != 2)
		{	
			case_ = 2;

			if(100*(average_features_/ranges_size_) > threshold_density_)
				temp_env_ = "|| Walls: Maybe || Features: Yes || Density: Dense  ||";
			else
				temp_env_ = "|| Walls: Maybe || Features: No  || Density: Sparse ||";
			
			ROS_INFO("[average_var_] %f", average_var_);
			msg_env_.data = temp_env_.c_str();
			env_pub_.publish(msg_env_);
		}
	}	
}

}