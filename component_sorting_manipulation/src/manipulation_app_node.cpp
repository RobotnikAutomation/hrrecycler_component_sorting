#include <component_sorting_manipulation/manipulation_app.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_app");
  ros::NodeHandle n;

  ManipulationApp ma(n);
  ma.asyncStart();

  ros::AsyncSpinner spinner(8);
//  ros::Duration(10).sleep();
//  cs.gripper_on();
  //ros::spin();
  spinner.start();
  ros::waitForShutdown();
}
