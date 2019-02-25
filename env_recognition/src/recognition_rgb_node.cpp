#include <env_recognition_test/recognition_rgb.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "recognition_rgb");

	env_recognition_test::RecognitionRgb obj;

	ros::spin();
	return 0;
}