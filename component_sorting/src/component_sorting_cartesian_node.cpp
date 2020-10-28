#include <component_sorting/component_sorting_cartesian.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "component_sorting_cartesian");
  ros::NodeHandle n;

  ComponentSorting cs(n);
  cs.asyncStart();
  ros::spin();
}
