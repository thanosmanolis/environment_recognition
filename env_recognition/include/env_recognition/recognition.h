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

		//! Random variables
		int ranges_size_;			//! Size of ranges[] list

		int temp_var_;				//! Variance of each measurement	
		int list_vars_[100];		//! A list of the last 100 variances			
		int sum_vars_;				//! Sum of the last 100 variances
		float average_var_;			//! Mean value of the last 100 variances
		
		int temp_features_;			//! Non-infinite ranges of each measurement
		int list_features_[100];	//! A list of the last 100 non-ininite ranges		
		int sum_features_;			//! Sum of the last 100 non-ininite ranges
		float average_features_;	//! Mean value of the last 100 non-ininite ranges

		int counter_msgs_;			//! Counts the laser measuremets so far
		int counter_inf_;			//! Counts the infinite values at each measurement
		int index_;					//! Index of the list we are working on 
		int samples_; 				//! Samples, from which average value will occur
		int case_;					//! Environment case we are at
		string temp_env_;			//! Environment type to be published
		
		//! ROS Parameters
		int threshold_;				//! Lower -> Walls  || Higher -> Features (maybe with walls)
		int threshold_density_;		//! Lower -> Sparse || Higher -> Dense

		//! ROS Publisher
		ros::Publisher env_pub_;

		//! ROS Subscriber
		ros::Subscriber scan_sub_;

		//! Random Functions
		void laserCallback	(const sensor_msgs::LaserScan::ConstPtr &msg_laser);
		void averageValues	(void);
		void initParams 	(void);
		void initValues		(void);

	public:
		Recognition 	(void);
		~Recognition 	(void);
};
}

#endif