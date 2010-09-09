#include "pose_estimation/pose_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poseEstimation");

  PoseEstimationNode poseEstimationNode;

  ros::spin();

	return 0;
}
