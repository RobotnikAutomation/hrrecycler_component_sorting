#include <component_sorting/component_sorting.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "component_sorting");
  ros::NodeHandle n;

  ComponentSorting cs(n);
  cs.asyncStart();

//  ros::Duration(10).sleep();
//  cs.gripper_on();
  ros::spin();
}
