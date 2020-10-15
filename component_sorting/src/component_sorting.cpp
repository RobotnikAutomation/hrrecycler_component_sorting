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
  pickup_from_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>(pnh_, "pickup_from", autostart));
  pickup_from_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("pickup_from")));
  pickup_from_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  place_on_as_.reset(
      new actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>(pnh_, "place_on", autostart));
  place_on_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this, std::string("place_on")));
  place_on_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));

  // Connect to database
  host = "localhost";
  port = 33829;
  connection_timeout = 5.0;
  connection_retries = 5;

    try
  {
    conn_ = moveit_warehouse::loadDatabase();
    conn_->setParams(host, port, connection_timeout);

    ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);
    int tries = 0;
    while (!conn_->connect())
    {
      ++tries;
      ROS_WARN("Failed to connect to DB on %s:%d (try %d/%d).", host.c_str(), port, tries, connection_retries);
      if (tries == connection_retries)
      {
        ROS_FATAL("Failed to connect too many times, giving up");
        return 1;
      }
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 1;
  }

  move_group_->setConstraintsDatabase(host,port);
  std::vector< std::string > stored_constraints = move_group_->getKnownConstraints();
  if (stored_constraints.empty())
    ROS_INFO("There are no constraints stored in database");
  else
  {
    ROS_INFO("Constraints currently stored in database:");
    for (const std::string& name : stored_constraints)
      ROS_INFO(" * %s", name.c_str());
  }


  // in case we contact MoveIt through actionlib
  // pickup_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(pnh_, "pickup", autostart));
  // pickup_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  // pickup_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  // place_as_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(pnh_, "place", autostart));
  // place_as_->registerGoalCallback(boost::bind(&ComponentSorting::goalCB, this));
  // place_as_->registerPreemptCallback(boost::bind(&ComponentSorting::preemptCB, this));
  //
  // bool spin_action_thread = true;
  // pickup_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PickupAction>(nh_, "pickup", spin_action_thread));
  // place_ac_.reset(new actionlib::SimpleActionClient<moveit_msgs::PlaceAction>(nh_, "place", spin_action_thread));

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

  // in case we contact MoveIt through actionlib
  // ros::Duration timeout = ros::Duration(5);
  // pickup_ac_->waitForServer(timeout);
  // if (pickup_ac_->isServerConnected() == false)
  // {
  //   RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: pickup");
  //   return rcomponent::ERROR;
  // }
  // place_ac_->waitForServer(timeout);
  // if (place_ac_->isServerConnected() == false)
  // {
  //   RCOMPONENT_ERROR_STREAM("Cannot connect to pick up client: place");
  //   return rcomponent::ERROR;
  // }

  // pickup_as_->start();
  // RCOMPONENT_INFO_STREAM("Started server: pickup");
  // place_as_->start();
  // RCOMPONENT_INFO_STREAM("Started server: place");

  pickup_from_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: pickup from");
  place_on_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: place on");

  return rcomponent::OK;
}

void ComponentSorting::standbyState()
{
  switchToState(robotnik_msgs::State::READY_STATE);
}

void ComponentSorting::readyState()
{
  if (pickup_from_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }


  ROS_INFO_THROTTLE(3, "I have a new goal!");

  //moveit_msgs::Constraints selected_constraint;
  //selected_constraint.name = "downright"; 
  std::string selected_constraint = "downright";

  move_group_->setPathConstraints(selected_constraint);
  moveit_msgs::Constraints prueba = move_group_->getPathConstraints();	
  ROS_INFO("Planning with Constraint: %s", prueba.name.c_str());
  //  geometry_msgs::Pose target_pose;
  //  target_pose.orientation.x = 0.0;
  //  target_pose.orientation.y = 0.0;
  //  target_pose.orientation.z = 1.0;
  //  target_pose.orientation.w = 0.0;
  //  target_pose.position.x = 0.111;
  //  target_pose.position.y = -0.230;
  //  target_pose.position.z = 0.221;
  //
  //  move_group_->setPoseTarget(target_pose);


  // Get desired goal and set as target
  std::string desired_goal = pickup_from_goal_->from;
  move_group_->setNamedTarget(desired_goal);

  // std::vector<double> joints(move_group_->getJointValueTarget().getJointModelGroup("arm")->getVariableNames().size());
  
  // for(int i=0; i < joints.size(); i++)
  //   std::cout << joints.at(i) << ' ';

  // std:: cout << endl;

  //move_group_->setJointValueTarget(joints);

  // Check if goal is active and Plan to target goal
  if (!pickup_from_as_->isActive())
        return;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  ROS_INFO_THROTTLE(3, "About to plan");
  //ROS_INFO_STREAM("Plan solved: " << move_group_->plan(plan));
  //== moveit::planning_interface::MoveItErrorCode::SUCCESS);
  bool success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_plan){
    feedback_.state = "Plan to desired position computed";
    pickup_from_as_->publishFeedback(feedback_);
  }

  //Check if goal is active and move to target goal
  ROS_INFO_THROTTLE(3, "About to move");
  if (!pickup_from_as_->isActive())
        return;

  //move_group_->move();
  bool success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_move){
    ROS_INFO_THROTTLE(3, "Moved!");
    feedback_.state.clear();
    feedback_.state = "Moved to desired position";
    pickup_from_as_->publishFeedback(feedback_);

    result_.success = true;
    result_.message = "Moved to desired position SUCCESSFUL";
    pickup_from_as_->setSucceeded(result_);
  }else{
    feedback_.state.clear();
    feedback_.state = "Could not move to desired position";
    pickup_from_as_->publishFeedback(feedback_);

    result_.success = false;
    result_.message = "Moved to desired position ABORTED";
    pickup_from_as_->setAborted(result_);

  }

}

void ComponentSorting::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if (action_ == "pickup_from")
    pickup_from_goal_ = pickup_from_as_->acceptNewGoal();
    
}

void ComponentSorting::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  // set the action state to preempted
  pickup_from_as_->setPreempted();
}
