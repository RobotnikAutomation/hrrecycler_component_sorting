#include <component_sorting/object.h>

Object::Object(ros::NodeHandle pnh, std::string id)
{
    // Get parameters from parameter server

    pnh_ = pnh;
    id_ = id;
    pnh_.getParam("frame_id", frame_id_); 
    pnh_.getParam("pose", pose_); 
    pnh_.getParam("geometry", geometry_); 


    // Process parameter geometry and check if is of type mesh or type primitive

    if ( geometry_.hasMember("mesh")){

        check_mesh = true;

        std::string mesh_path_; 
        pnh_.getParam("geometry/mesh", mesh_path_ );

        shapes::Mesh* m = shapes::createMeshFromResource(mesh_path_); 
        shapes::ShapeMsg mesh_msg;  
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    }else if (geometry_.hasMember("box")){
        
        check_primitive = true;

        XmlRpc::XmlRpcValue box_primitive_;
        pnh_.getParam("geometry/box", box_primitive_ );

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = box_primitive_["length"];
        primitive.dimensions[1] = box_primitive_["width"];
        primitive.dimensions[2] = box_primitive_["height"];
    }else{
        ROS_WARN("Cannot process geometry parameter, check object configuration yaml");
    }

    // Process parameter pose

    if (pose_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_.size() == 2)
    {    
        XmlRpc::XmlRpcValue pose_position_;
        XmlRpc::XmlRpcValue pose_orientation_;
        tf2::Quaternion quaternion_orientation;

        pose_position_ = pose_[0];
        pose_orientation_ = pose_[1];

        if (pose_position_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_position_.size() == 3){
           
            pose_msg.position.x = (pose_position_[0]);
            pose_msg.position.y = (pose_position_[1]);
            pose_msg.position.z = (pose_position_[2]); 

        }else{
            ROS_WARN("Cannot process pose position parameter, it should contain [x,y,z] array, check object configuration yaml");
        }

        if (pose_orientation_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_orientation_.size() == 3){

            quaternion_orientation.setRPY( pose_orientation_[0], pose_orientation_[1], pose_orientation_[2]);
        
            pose_msg.orientation.x = pose_orientation_[0];
            pose_msg.orientation.y = pose_orientation_[1];
            pose_msg.orientation.z = pose_orientation_[2];     
            pose_msg.orientation.w = pose_orientation_[3];             
        }else{
            ROS_WARN("Cannot process pose orientation parameter, it should contain [r,p,y] array, check object configuration yaml");
        }

    }else{
        ROS_WARN("Cannot process pose parameter, it should contain [[x,y,z],[r,p,y]] list, check object configuration yaml");
    }
}

Object::~Object(){
    
};

std::string Object::get_id(){
    return id_;
}

std::string Object::get_frame_id(){
    return frame_id_;
}

bool Object::has_mesh(){
    return check_mesh;
}

bool Object::has_primitive(){
    return check_primitive;
}

geometry_msgs::Pose Object::get_pose(){
    return pose_msg;
}

shape_msgs::Mesh Object::get_mesh(){
    return mesh;
}

shape_msgs::SolidPrimitive Object::get_primitive(){
    return primitive;
}