#ifndef RECOGNITION_H
#define RECOGNITION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <iostream>

using namespace std;

namespace env_recognition
{
class Recognition
{
	private:
		ros::NodeHandle n_;
		std_msgs::String msg_env_;	//! Publish the recognised environment through this msg

		int ranges_size_;			//! Size of ranges[] list
		int temp_var_;				//! Variance of each measurement				
		int sum_vars_;				//! Sum of variances so far
		float average_var_;			//! Mean value of variance of each measurement
		int start_;					//! Start point of most recent feature
		int end_;					//! End point of most recent feature
		int temp_features_;			//! Sum of all features' of each measurement
		int sum_features_;			//! Sum of all features' sizes so far
		float average_features_;	//! Mean value of all features' sizes of each measurement
		int counter_msgs_;			//! Counts the laser measuremets so far
		int counter_inf_;			//! Counts the infinite values at each measurement
		string temp_env_;			//! Environment type to be published
		int case_;					//! Environment case we are at
		
		//! ROS Parameters
		int threshold_;				//! Lower -> Walls  || Higher -> Features (maybe with walls)
		int threshold_density_;		//! Lower -> Sparse || Higher -> Dense

		//! ROS Publisher
		ros::Publisher env_pub_;

		//! ROS Subscriber
		ros::Subscriber scan_sub_;

		//! Random Functions
		void laserCallback			(const sensor_msgs::LaserScan::ConstPtr &msg_laser);
		void varianceAverageValue	(void);
		void initParams 		    (void);
		void initValues				(void);

	public:
		Recognition 	(void);
		~Recognition 	(void);
};
}

#endif