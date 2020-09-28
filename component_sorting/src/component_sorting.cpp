#include <component_sorting/component_sorting.h>

ComponentSorting::ComponentSorting(ros::NodeHandle h) : RComponent(h)
{
  init(h);
}
ComponentSorting::~ComponentSorting()
{
}

void ComponentSorting::rosReadParams()
{
  bool required = true;

  required = true;
  group_name_ = "arm";
  readParam(pnh_, "group_name", group_name_, group_name_, required);

  required = false;
  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, required);
  move_group_timeout_ = ros::WallDuration(timeout);
}

int ComponentSorting::rosSetup()
{
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  tf2_buffer_.reset(new tf2_ros::Buffer);
  try
  {
    move_group_.reset(
        new moveit::planning_interface::MoveGroupInterface(group_name_, tf2_buffer_, move_group_timeout_));
  }
  catch (const std::runtime_error& e)
  {
    RCOMPONENT_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?",
                     group_name_.c_str());
    RCOMPONENT_ERROR_STREAM("Exception: " << e.what());
    return rcomponent::ERROR;
  }

  bool autostart = false;
  pickup_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(pnh_, "pickup", autostart));
  pickup_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  pickup_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  place_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(pnh_, "place", autostart));
  place_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  place_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));

  bool spin_action_thread = true;
  pickup_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh_, "pickup", spin_action_thread));
  place_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh_, "place", spin_action_thread));

  return RComponent::rosSetup();
}

int ComponentSorting::rosShutdown()
{
  return RComponent::rosShutdown();
}

int ComponentSorting::setup()
{
  // Checks if has been initialized
  int setup_result;

  setup_result = rcomponent::RComponent::setup();
  if (setup_result != rcomponent::OK)
  {
    return setup_result;
  }

  ros::Duration timeout = ros::Duration(5);
  pickup_ac_->waitForServer(timeout);
  if (pickup_ac_->isServerConnected() == false)
  {
    RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: " << "pickup");
    return rcomponent::ERROR;
  }
  place_ac_->waitForServer(timeout);
  if (place_ac_->isServerConnected() == false)
  {
    RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: " << "place");
    return rcomponent::ERROR;
  }
  
  pickup_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: " << "pickup");
  place_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: " << "place");
}

void ComponentSorting::standbyState()
{
  switchToState(robotnik_msgs::State::READY_STATE);
}

void ComponentSorting::readyState()
{
}

void ComponentSorting::goalCB()
{

}

void ComponentSorting::preemptCB()
{
}
