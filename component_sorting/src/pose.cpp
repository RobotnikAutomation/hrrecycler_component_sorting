#include <component_sorting/pose.h>

Pose::Pose(ros::NodeHandle pnh)
{
    // Get parameters from parameter server

    pnh_ = pnh;
    pnh_.getParam("frame_id", frame_id_); 
    pnh_.getParam("pose", pose_); 

    pose_stamped_msg.header.frame_id = frame_id_;

    // Process parameter pose

    if (pose_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_.size() == 2)
    {    
        XmlRpc::XmlRpcValue pose_position_;
        XmlRpc::XmlRpcValue pose_orientation_;
        tf2::Quaternion quaternion_orientation;

        pose_position_ = pose_[0];
        pose_orientation_ = pose_[1];

        if (pose_position_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_position_.size() == 3){
           
            pose_stamped_msg.pose.position.x = (pose_position_[0]);
            pose_stamped_msg.pose.position.y = (pose_position_[1]);
            pose_stamped_msg.pose.position.z = (pose_position_[2]); 

        }else{
            ROS_WARN("Cannot process pose position parameter, it should contain [x,y,z] array, check object configuration yaml");
        }

        if (pose_orientation_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_orientation_.size() == 3){

            quaternion_orientation.setRPY( pose_orientation_[0], pose_orientation_[1], pose_orientation_[2]);

            pose_stamped_msg.pose.orientation.x = quaternion_orientation[0];
            pose_stamped_msg.pose.orientation.y = quaternion_orientation[1];
            pose_stamped_msg.pose.orientation.z = quaternion_orientation[2];     
            pose_stamped_msg.pose.orientation.w = quaternion_orientation[3];             
        }else{
            ROS_WARN("Cannot process pose orientation parameter, it should contain [r,p,y] array, check object configuration yaml");
        }

    }else{
        ROS_WARN("Cannot process pose parameter, it should contain [[x,y,z],[r,p,y]] list, check object configuration yaml");
    }
}

Pose::~Pose(){
    
};


geometry_msgs::PoseStamped Pose::get_pose(){
    return pose_stamped_msg;
}

