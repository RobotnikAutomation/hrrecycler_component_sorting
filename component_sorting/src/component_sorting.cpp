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

 /*     try
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
  } */

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

  create_planning_scene();

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
    // Move to home position without selected constraints
  move_group_->setPathConstraints("");
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
  std::string selected_constraint = "prueba";

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
        pick_chain_movement("kairos_pre_right","kairos_right","box_right");
        break;
      }
      case ev_left :
      {
        pick_chain_movement("kairos_pre_left","kairos_left","box_left");
        break;
      }
      case ev_center :
      {
        pick_chain_movement("kairos_pre_center","kairos_center","box_center");
        break;
      }
      case ev_table_right :
      {
        pick_chain_movement("table_pre_right","table_right","box_right");
        break;
      }
      case ev_table_left :
      {
        pick_chain_movement("table_pre_left","table_left","box_left");
        break;
      }
      case ev_table_center :
      {
        pick_chain_movement("table_pre_center","table_center","box_center");
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
        place_chain_movement("kairos_pre_right","kairos_right");
        break;
      }
      case ev_left :
      {
        place_chain_movement("kairos_pre_left","kairos_left");
        break;
      }
      case ev_center :
      {
        place_chain_movement("kairos_pre_center","kairos_center");
        break;
      }
      case ev_table_right :
      {
        place_chain_movement("table_pre_right","table_right");
        break;
      }
      case ev_table_left :
      {
        place_chain_movement("table_pre_left","table_left");
        break;
      }
      case ev_table_center :
      {
        place_chain_movement("table_pre_center","table_center");
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

void ComponentSorting::pick_chain_movement(std::string pre_position, std::string position, std::string box_id)
{ 
  // Set pre-position goal
  move_group_->setNamedTarget(pre_position);

  // Check if goal is active and Plan to target goal
  if (!pickup_from_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
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

  // Set pre-position goal
  move_group_->setNamedTarget(position);

  // Check if goal is active and Plan to target goal
  if (!pickup_from_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan to desired position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
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

  // Attach box to end effector
  if(move_group_->attachObject(box_id)){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Box attached to end effector";
    pickup_from_as_->publishFeedback(pick_feedback_);
  }else{
    pick_result_.success = false;
    pick_result_.message = "Could not attach box";
    pickup_from_as_->setAborted(pick_result_);
    return;
  };

  // Set position goal
  move_group_->setNamedTarget(pre_position);

  // Check if goal is active and Plan to target goal
  if (!pickup_from_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    pick_feedback_.state.clear();
    pick_feedback_.state = "Plan back to pre-position computed";
    pickup_from_as_->publishFeedback(pick_feedback_);

    //Check if goal is active and move to target goal
    if (!pickup_from_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
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

void ComponentSorting::place_chain_movement(std::string pre_position, std::string position)
{ 
  // Set pre-position goal
  move_group_->setNamedTarget(pre_position);

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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

  // Set pre-position goal
  move_group_->setNamedTarget(position);

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan to desired position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      ROS_INFO_THROTTLE(3, "Moved!");
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

  // Detach box to end effector
  if(move_group_->detachObject()){
    place_feedback_.state.clear();
    place_feedback_.state = "Box detached from end effector";
    place_on_as_->publishFeedback(place_feedback_);
  }else{
    place_result_.success = false;
    place_result_.message = "Could not detach box";
    place_on_as_->setAborted(place_result_);
    return;
  };

  // Set position goal
  move_group_->setNamedTarget(pre_position);

  // Check if goal is active and Plan to target goal
  if (!place_on_as_->isActive()) return;
  ROS_INFO_THROTTLE(3, "About to plan");
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //If plan is successful execute trajectory
  if(success_plan){
    place_feedback_.state.clear();
    place_feedback_.state = "Plan back to pre-position computed";
    place_on_as_->publishFeedback(place_feedback_);

    //Check if goal is active and move to target goal
    if (!place_on_as_->isActive()) return;
    ROS_INFO_THROTTLE(3, "About to move");
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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
  /* move_group_->attachObject(co_4.id);
  chain_movement("kairos_pre_right","kairos_pre_right"); */
}


void ComponentSorting::create_planning_scene(){
    // The id of the object is used to identify it.
  co_1.id = "holder_right";
  co_2.id = "holder_center";
  co_3.id = "holder_left";
  co_4.id = "box_right";
  co_5.id = "box_center";
  co_6.id = "box_left";
  co_1.header.frame_id = "robot_base_footprint";
  co_2.header.frame_id = "robot_base_footprint";
  co_3.header.frame_id = "robot_base_footprint";
  co_4.header.frame_id = "robot_right_holder_link";
  co_5.header.frame_id = "robot_center_holder_link";
  co_6.header.frame_id = "robot_left_holder_link";

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
  std::string box_mesh_path = "package://component_sorting_description/meshes/box/ensamblaje_caja_asa.stl";

  //shapes::Mesh* holder_m = shapes::createMeshFromResource("package://component_sorting_description/meshes/box/box_handle.stl", vectorScale); 
  shapes::Mesh* box_m = shapes::createMeshFromResource(box_mesh_path); 
  ROS_INFO("Your box_mesh was loaded");
  
  shape_msgs::Mesh box_mesh;
  shapes::ShapeMsg box_mesh_msg;  
  shapes::constructMsgFromShape(box_m, box_mesh_msg);
  box_mesh = boost::get<shape_msgs::Mesh>(box_mesh_msg);
  
  //Define a pose for your holder_mesh (specified relative to frame_id)
  geometry_msgs::Pose holder_right_pose;
  holder_right_pose.position.x = 0.68;
  holder_right_pose.position.y = -0.228;
  holder_right_pose.position.z = 0.88; //0.9
  
  geometry_msgs::Pose holder_center_pose;
  holder_center_pose.position.x = 0.68;
  holder_center_pose.position.y = 0.0;
  holder_center_pose.position.z = 0.88; //0.9

  geometry_msgs::Pose holder_left_pose;
  holder_left_pose.position.x = 0.68;
  holder_left_pose.position.y = 0.228;
  holder_left_pose.position.z = 0.88; //0.9

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = 0.01;


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
  
  //Publish object in monitored planning scene
  
  // Create vector of collision objects to add 
  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(co_1);
  objects.push_back(co_2);
  objects.push_back(co_3);
  objects.push_back(co_4);
  objects.push_back(co_5);
  objects.push_back(co_6); 
  // Add the collision object into the world
  planning_scene_interface.addCollisionObjects(objects);
}
