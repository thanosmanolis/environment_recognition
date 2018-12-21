#include <env_recognition/recognition.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition");

	env_recognition::Recognition obj;

	ros::spin();
	return 0;
}