#include <component_sorting/component_sorting_cartesian.h>

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

  required = true;
  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  required = true;
  port = 33829;
  readParam(pnh_, "port", port, port, required);

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
  //host = "localhost";
  //port = 33829;
  //connection_timeout = 5.0;
  //connection_retries = 3;

  conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_WARN("Failed to connect to DB on %s:%d ", host.c_str(), port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  

  //move_group_->setPoseReferenceFrame("robot_map");


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

  allowed_movement.key = "allowed_movement";
  allowed_movement.db = "type: 'allowed_movement'";
  create_planning_scene();

  // kairos cartesian poses
 // kairos_frame.frame_id = "robot_base_footprint";
  pre_kairos_center_pose.header.frame_id = "robot_center_approach";
  pre_kairos_center_pose.pose.orientation.w = 1;
  pre_kairos_right_pose.header.frame_id = "robot_right_approach";
  pre_kairos_right_pose.pose.orientation.w = 1;
  pre_kairos_left_pose.header.frame_id = "robot_left_approach";
  pre_kairos_left_pose.pose.orientation.w = 1;
  kairos_center_pose.header.frame_id = "robot_center_grab";
  kairos_center_pose.pose.orientation.w = 1;
  kairos_right_pose.header.frame_id = "robot_right_grab";
  kairos_right_pose.pose.orientation.w = 1;
  kairos_left_pose.header.frame_id = "robot_left_grab";
  kairos_left_pose.pose.orientation.w = 1;

  //kairos table poses
  //table_qr_frame.frame_id = "table_qr_frame";
  pre_table_center_pose.header.frame_id = "table_center_approach";
  pre_table_center_pose.pose.orientation.w = 1;
  pre_table_right_pose.header.frame_id = "table_right_approach";
  pre_table_right_pose.pose.orientation.w = 1;
  pre_table_left_pose.header.frame_id = "table_left_approach";
  pre_table_left_pose.pose.orientation.w = 1;
  table_center_pose.header.frame_id = "table_center_grab";
  table_center_pose.pose.orientation.w = 1;
  table_right_pose.header.frame_id = "table_right_grab";
  table_right_pose.pose.orientation.w = 1;
  table_left_pose.header.frame_id = "table_left_grab";
  table_left_pose.pose.orientation.w = 1;

  client = nh_.serviceClient<ur_msgs::SetIO>("arm/ur_hardware_interface/set_io");

/*   visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update"));

  robot_state_ = visual_tools_->getSharedRobotState(); 
  
  joint_model_group= robot_state_->getJointModelGroup("arm"); */
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("robot_base_footprint","/move_group/display_grasp_markers"));
 /*  moveit_visual_tools::MoveItVisualTools visual_tools_("robot_base_footprint"); */
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  
/*   visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
 */
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
  bool prueba = move_group_->startStateMonitor (10);
  std:: cout << prueba << endl;
  robot_state_ = move_group_->getCurrentState();
  joint_model_group= robot_state_->getJointModelGroup("arm");
  
  

    // Move to home position without selected constraints
  move_group_->detachObject();  
  move_group_->clearPathConstraints();
  move_group_->setNamedTarget("home_position");
  success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success_move){
    ROS_INFO("Moved to home position, ready to take commands");
    switchToState(robotnik_msgs::State::READY_STATE);
  }else{
    ROS_INFO("Could not move to home position, warning");
  }
  
}

void ComponentSorting::readyState()
{ 

  if (pickup_from_as_->isActive() == false && place_on_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  ROS_INFO_THROTTLE(3, "I have a new goal!");

  //moveit_msgs::Constraints selected_constraint;
  //selected_constraint.name = "downright"; 

  // Select constraint
  std::string selected_constraint = "elbow_up";

  move_group_->setPathConstraints(selected_constraint);
  //move_group_->clearPathConstraints (); //Remove
  moveit_msgs::Constraints prueba = move_group_->getPathConstraints();	
  ROS_INFO("Planning with Constraint: %s", prueba.name.c_str());

  //std::string planning_frame = move_group_->getPlanningFrame();
  //std:: cout << planning_frame << endl;

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

  // Value-Defintions of the different switch cases (desired chain actions)
   enum StringValue { ev_NotDefined,
                      ev_right, 
                      ev_left, 
                      ev_center, 
                      ev_table_right,
                      ev_table_left,
                      ev_table_center };
  // Map switch case values (desired chain actions)
   static std::map<std::string, StringValue> s_mapStringValues;
   s_mapStringValues["kairos_right"] = ev_right;
   s_mapStringValues["kairos_left"] = ev_left;
   s_mapStringValues["kairos_center"] = ev_center;
   s_mapStringValues["table_right"] = ev_table_right;
   s_mapStringValues["table_left"] = ev_table_left;
   s_mapStringValues["table_center"] = ev_table_center;

  if(pickup_from_as_->isActive() == true){ 

    // Get desired goal and set as target
    std::string desired_goal = pickup_from_goal_->from;
    
    switch(s_mapStringValues[desired_goal]){
      case ev_right :
      {  
        pick_chain_movement(pre_kairos_right_pose,kairos_right_pose,"box_right","box_handle_right");
        break;
      }
      case ev_left :
      {
        pick_chain_movement(pre_kairos_left_pose,kairos_left_pose,"box_left","box_handle_left");
        break;
      }
      case ev_center :
      {
        pick_chain_movement(pre_kairos_center_pose,kairos_center_pose,"box_center","box_handle_center");
        break;
      }
      case ev_table_right :
      {
        pick_chain_movement(pre_table_right_pose,table_right_pose,"box_right","box_handle_right");
        break;
      }
      case ev_table_left :
      {
        pick_chain_movement(pre_table_left_pose,table_left_pose,"box_left","box_handle_left");
        break;
      }
      case ev_table_center :
      {
        pick_chain_movement(pre_table_center_pose,table_center_pose,"box_center","box_handle_center");
        break;
      }
      default :
      {   
        ROS_INFO("Introduced goal position is not defined");
        pick_result_.success = false;
        pick_result_.message = "Introduced goal position is not defined";
        pickup_from_as_->setAborted(pick_result_);
        break;
      }
    }  
  }   
  
  if(place_on_as_->isActive() == true){ 
    // Get desired goal and set as target
    std::string desired_goal = place_on_goal_->in;
    
    switch(s_mapStringValues[desired_goal]){
      case ev_right :
      { 
        place_chain_movement(pre_kairos_right_pose,kairos_right_pose);
        break;
      }
      case ev_left :
      {
        place_chain_movement(pre_kairos_left_pose,kairos_left_pose);
        break;
      }
      case ev_center :
      {
        place_chain_movement(pre_kairos_center_pose,kairos_center_pose);
        break;
      }
      case ev_table_right :
      {
        place_chain_movement(pre_table_right_pose,table_right_pose);
        break;
      }
      case ev_table_left :
      {
        place_chain_movement(pre_table_left_pose,table_left_pose);
        break;
      }
      case ev_table_center :
      {
        place_chain_movement(pre_table_center_pose,table_center_pose);
        break;
      }
      default :
      {   
        ROS_INFO("Introduced goal position is not defined");
        place_result_.success = false;
        place_result_.message = "Introduced goal position is not defined";
        place_on_as_->setAborted(place_result_);
        break;
      }
    }  
  }   
}

void ComponentSorting::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if (action_ == "pickup_from"){
    pickup_from_goal_ = pickup_from_as_->acceptNewGoal();}
  if (action_ == "place_on"){
    place_on_goal_ = place_on_as_->acceptNewGoal();}
    
}

void ComponentSorting::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  //result_.success = false;
  //result_.message = "Goal has been cancelled, stopping execution.";
  // set the action state to preempted
  pickup_from_as_->setPreempted();
  place_on_as_->setPreempted();
}

void ComponentSorting::pick_chain_movement(geometry_msgs::PoseStamped pre_position, geometry_msgs::PoseStamped position, std::string box_id, std::string handle_id)
{ 
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  // Set pre-position goal
  move_group_->setPoseTarget(pre_position);
  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 
  visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools_->trigger();
  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved to desired pre-position";
      pickup_from_as_->publishFeedback(pick_feedback_);
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move to desired pre-position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }
          
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan to desired pre-position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  // Check if goal is active 
  //if (!pickup_from_as_->isActive()) return;
  // Get current end effector position and check whether there is a box to grab
  current_cartesian_pose = move_group_->getCurrentPose().pose;

  objects = planning_scene_interface.getKnownObjectNamesInROI	(current_cartesian_pose.position.x - box_length/2, current_cartesian_pose.position.y - box_width/2 ,0, current_cartesian_pose.position.x 
  + box_length/2, current_cartesian_pose.position.y + box_width/2 , 3, false );

  if(objects.size() < 2){
    pick_result_.success = false;
    pick_result_.message = "There is no box to grab";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  //Cartesian plan to position goal
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

/*   visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger(); */


  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved to desired position";
      pickup_from_as_->publishFeedback(pick_feedback_);
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move to desired position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }       
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan to desired position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

  //Pick object identified as handle
  for(int i=0; i < objects.size(); i++){
   if(objects[i].compare(0,6,"handle")==0){
      if(move_group_->attachObject(objects[i])){
        pick_feedback_.state.clear();
        pick_feedback_.state = "Handle attached to end effector";
        pickup_from_as_->publishFeedback(pick_feedback_);
      }else{
        pick_result_.success = false;
        pick_result_.message = "Could not attach handle";
        pickup_from_as_->setAborted(pick_result_);
        return;
      }
      break;
   } 
  }

  //Activate gripper
  gripper_on();
  ros::Duration(1).sleep();

  //Move upwards 2 cm and attach box
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoint_cartesian_pose.position.z -= 0.02; 
  waypoints.push_back(waypoint_cartesian_pose);  

  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  for(int i=0; i < objects.size(); i++){
   if(objects[i].compare(0,3,"box")==0){
      if(move_group_->attachObject(objects[i])){
        pick_feedback_.state.clear();
        pick_feedback_.state = "Box attached to end effector";
        pickup_from_as_->publishFeedback(pick_feedback_);
      }else{
        pick_result_.success = false;
        pick_result_.message = "Could not attach box";
        pickup_from_as_->setAborted(pick_result_);
        return;
      }
      break;
   } 
  }

  // Plan back to pre-position goal
  waypoints.clear();
  waypoint_cartesian_pose = pre_position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(pre_position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

    
  visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger();

  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan back to pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(3).sleep();

    //Check if execute is successful
    if(success_execute){
      pick_feedback_.state.clear();
      pick_feedback_.state = "Moved back to pre-position";
      pickup_from_as_->publishFeedback(pick_feedback_);

      pick_result_.success = true;
      pick_result_.message = "Pick-up from desired position SUCCESSFUL";
      pickup_from_as_->setSucceeded(pick_result_);
      return;
    }else{
      pick_result_.success = false;
      pick_result_.message = "Could not move back to pre-position";
      pickup_from_as_->setAborted(pick_result_);
      return;
    }          
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not plan back to pre-position";
    pickup_from_as_->setAborted(pick_result_);
    return;
  }

}

void ComponentSorting::place_chain_movement(geometry_msgs::PoseStamped pre_position, geometry_msgs::PoseStamped position)
{ 
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  // Set pre-position goal
  move_group_->setPoseTarget(pre_position);
  //Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools_->trigger();  
  //If plan is successful execute trajectory
  if(success_plan){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(1).sleep();

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
      place_feedback_.state.clear();
      place_feedback_.state = "Moved to desired pre-position";
      place_on_as_->publishFeedback(place_feedback_);
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move to desired pre-position";
      place_on_as_->setAborted(place_result_);
      return;
    }
          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan to desired pre-position";
    place_on_as_->setAborted(place_result_);
    return;
  }

  //Plan to position goal + 0.02 m 
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoint_cartesian_pose.position.z -= 0.02;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  visual_tools_->publishTrajectoryLine(cartesian_plan.trajectory_, joint_model_group);
  visual_tools_->trigger();
  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ros::Duration(2).sleep();

    //Check if execute is successful
    if(success_execute){
      place_feedback_.state.clear();
      place_feedback_.state = "Moved to desired position";
      place_on_as_->publishFeedback(place_feedback_);
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move to desired position";
      place_on_as_->setAborted(place_result_);
      return;
    }
          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan to desired position";
    place_on_as_->setAborted(place_result_);
    return;
  }

  //Detach box from end effector
  for(int i=0; i < objects.size(); i++){
   if(objects[i].compare(0,3,"box")==0){
     // Detach box from effector
      move_group_->detachObject(objects[i]);
      break;
   } 
  }

  //Move to position goal and detach handle from end effector
  waypoints.clear();
  waypoint_cartesian_pose = position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);

  for(int i=0; i < objects.size(); i++){
   if(objects[i].compare(0,6,"handle")==0){
     // Detach handle from end effector
      move_group_->detachObject(objects[i]);
      break;
   } 
  }

  //Deactivate gripper
  gripper_off();
  ros::Duration(2).sleep();

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");

  //Plan to pre-position goal
  waypoints.clear();
  waypoint_cartesian_pose = pre_position.pose;
  waypoints.push_back(waypoint_cartesian_pose);  
  move_group_->setPoseReferenceFrame(pre_position.header.frame_id);
  success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  cartesian_plan.trajectory_ = trajectory;

  //If plan is successful execute trajectory
  if(success_cartesian_plan >= allowed_fraction_success){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan back to pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(cartesian_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ros::Duration(3).sleep();

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
      place_feedback_.state.clear();
      place_feedback_.state = "Moved back to pre-position";
      place_on_as_->publishFeedback(place_feedback_);

      place_result_.success = true;
      place_result_.message = "Pick-up from desired position SUCCESSFUL";
      place_on_as_->setSucceeded(place_result_);
      return;
    }else{
      place_result_.success = false;
      place_result_.message = "Could not move back to pre-position";
      place_on_as_->setAborted(place_result_);
      return;
    }          
  }else{
    place_result_.success = false;
    place_result_.message = "Could not plan back to pre-position";
    place_on_as_->setAborted(place_result_);
    return;
  }

}

void ComponentSorting::gripper_on(){
  srv.request.fun = 1;
  srv.request.pin =16;
  srv.request.state =1;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 16: %s", client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin =17;
  srv.request.state =1;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 17: %s", client.getService().c_str());
     return;
   }
}


void ComponentSorting::gripper_off(){
  srv.request.fun = 1;
  srv.request.pin =16;
  srv.request.state =0;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 16: %s", client.getService().c_str());
     return;
   }
  srv.request.fun = 1;
  srv.request.pin =17;
  srv.request.state =0;
  if (client.call(srv))
   {
     ROS_INFO("Result: %d", srv.response.success);
   }
  else
   {
     ROS_ERROR("Failed to call service set/IO on pin 17: %s", client.getService().c_str());
     return;
   }
}

void ComponentSorting::create_planning_scene(){
    // The id of the object is used to identify it.
  co_1.id = "holder_right";
  co_2.id = "holder_center";
  co_3.id = "holder_left";
  co_4.id = "box_right";
  co_4.type = allowed_movement;
  co_5.id = "box_center";
  co_5.type = allowed_movement;
  co_6.id = "box_left";
  co_6.type = allowed_movement;
  co_7.id = "handle_right";
  co_7.type = allowed_movement;
  co_8.id = "handle_center";
  co_8.type = allowed_movement;
  co_9.id = "handle_left";
  co_9.type = allowed_movement;
  co_10.id = "table";
  co_1.header.frame_id = "table_qr";
  co_2.header.frame_id = "table_qr";
  co_3.header.frame_id = "table_qr";
  co_4.header.frame_id = "robot_right_holder_link";
  co_5.header.frame_id = "robot_center_holder_link";
  co_6.header.frame_id = "robot_left_holder_link";
  co_7.header.frame_id = "robot_right_holder_link";
  co_8.header.frame_id = "robot_center_holder_link";
  co_9.header.frame_id = "robot_left_holder_link";
  co_10.header.frame_id = "table_qr";

  //Path where the .dae or .stl object is located
  std::string holder_mesh_path = "package://component_sorting_description/meshes/box/box_holder.stl";

  //shapes::Mesh* holder_m = shapes::createMeshFromResource("package://component_sorting_description/meshes/box/box_handle.stl", vectorScale); 
  shapes::Mesh* holder_m = shapes::createMeshFromResource(holder_mesh_path); 
  ROS_INFO("Your holder_mesh was loaded");
  
  shape_msgs::Mesh holder_mesh;
  shapes::ShapeMsg holder_mesh_msg;  
  shapes::constructMsgFromShape(holder_m, holder_mesh_msg);
  holder_mesh = boost::get<shape_msgs::Mesh>(holder_mesh_msg);

  //Path where the .dae or .stl object is located
  std::string box_mesh_path = "package://component_sorting_description/meshes/box/box_base.stl";

  //shapes::Mesh* holder_m = shapes::createMeshFromResource("package://component_sorting_description/meshes/box/box_handle.stl", vectorScale); 
  shapes::Mesh* box_m = shapes::createMeshFromResource(box_mesh_path); 
  ROS_INFO("Your box_mesh was loaded");
  
  shape_msgs::Mesh box_mesh;
  shapes::ShapeMsg box_mesh_msg;  
  shapes::constructMsgFromShape(box_m, box_mesh_msg);
  box_mesh = boost::get<shape_msgs::Mesh>(box_mesh_msg);

  //Path where the .dae or .stl object is located
  std::string handle_mesh_path = "package://component_sorting_description/meshes/box/handle.stl";

  //shapes::Mesh* holder_m = shapes::createMeshFromResource("package://component_sorting_description/meshes/box/box_handle.stl", vectorScale); 
  shapes::Mesh* handle_m = shapes::createMeshFromResource(handle_mesh_path); 
  ROS_INFO("Your handle_mesh was loaded");
  
  shape_msgs::Mesh handle_mesh;
  shapes::ShapeMsg handle_mesh_msg;  
  shapes::constructMsgFromShape(handle_m, handle_mesh_msg);
  handle_mesh = boost::get<shape_msgs::Mesh>(handle_mesh_msg);

  //Define table mesh as a box

  shape_msgs::SolidPrimitive table_mesh;
  table_mesh.type = table_mesh.BOX;
  table_mesh.dimensions.resize(3);
  table_mesh.dimensions[0] = table_length;
  table_mesh.dimensions[1] = table_width;
  table_mesh.dimensions[2] = table_height;
  
  //Define a pose for your holder_mesh (specified relative to frame_id)

  geometry_msgs::Pose holder_center_pose;
  holder_center_pose.position.x = holder_length/2;
  holder_center_pose.position.y = 0.0;
  holder_center_pose.position.z = table_mesh.dimensions[2] - qr_height; //0.775
  holder_center_pose.orientation.w = 1.0; 

  geometry_msgs::Pose holder_right_pose;
  holder_right_pose = holder_center_pose;
  holder_right_pose.position.y -= holder_width;

  geometry_msgs::Pose holder_left_pose;
  holder_left_pose = holder_center_pose;
  holder_left_pose.position.y += holder_width;

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 0.01;
  box_pose.orientation.w = 1.0;

  geometry_msgs::Pose handle_pose;
  handle_pose.position.x = 0;
  handle_pose.position.y = 0;
  handle_pose.position.z = 0.08; //0.01
  handle_pose.orientation.w = 1.0;

  geometry_msgs::Pose table_pose;
  table_pose.position.x =  table_mesh.dimensions[0]/2; //0.84
  table_pose.position.y = 0;
  table_pose.position.z = table_mesh.dimensions[2]/2 - qr_height; //0.375
  table_pose.orientation.w = 1.0;

  // Add the holder_mesh to the Collision object message 
  co_1.meshes.push_back(holder_mesh);
  co_1.mesh_poses.push_back(holder_right_pose);
  co_1.operation = co_1.ADD;

  co_2.meshes.push_back(holder_mesh);
  co_2.mesh_poses.push_back(holder_center_pose);
  co_2.operation = co_2.ADD;

  co_3.meshes.push_back(holder_mesh);
  co_3.mesh_poses.push_back(holder_left_pose);
  co_3.operation = co_3.ADD;

  co_4.meshes.push_back(box_mesh);
  co_4.mesh_poses.push_back(box_pose);
  co_4.operation = co_4.ADD;  

  co_5.meshes.push_back(box_mesh);
  co_5.mesh_poses.push_back(box_pose);
  co_5.operation = co_5.ADD;

  co_6.meshes.push_back(box_mesh);
  co_6.mesh_poses.push_back(box_pose);
  co_6.operation = co_6.ADD;  
  
  co_7.meshes.push_back(handle_mesh);
  co_7.mesh_poses.push_back(handle_pose);
  co_7.operation = co_7.ADD;  

  co_8.meshes.push_back(handle_mesh);
  co_8.mesh_poses.push_back(handle_pose);
  co_8.operation = co_8.ADD;

  co_9.meshes.push_back(handle_mesh);
  co_9.mesh_poses.push_back(handle_pose);
  co_9.operation = co_9.ADD;  
  
  co_10.primitives.push_back(table_mesh);
  co_10.primitive_poses.push_back(table_pose);
  co_10.operation = co_10.ADD; 

  //Publish object in monitored planning scene
  
  // Create vector of collision objects to add 
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(co_1);
  objects.push_back(co_2);
  objects.push_back(co_3);
  objects.push_back(co_4);
  objects.push_back(co_5);
  objects.push_back(co_6); 
  objects.push_back(co_7);
  objects.push_back(co_8);
  objects.push_back(co_9);
  objects.push_back(co_10);
  // Add the collision object into the world
  planning_scene_interface.addCollisionObjects(objects);
}
