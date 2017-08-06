#include "gazebo_ros_joint_force/gazebo_ros_joint_force.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointForce);

GazeboRosJointForce::GazeboRosJointForce() {
}

GazeboRosJointForce::~GazeboRosJointForce() {
}

void GazeboRosJointForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	this->model_ = _model;
	this->sdf_ = _sdf;

	//TODO begin or end
	update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosJointForce::onWorldUpdate, this));

	if(!ros::isInitialized()) {
		ROS_FATAL_STREAM("A ROS node for Gazegbo has not been initialized, unable to load plugin.");
		return;
	}
	rosNode_.reset(new ros::NodeHandle()); // "gazebo_ros_joint_force_plugin"
//	rosSubscriber_ = rosNode_->subscribe()

}

void GazeboRosJointForce::onWorldUpdate() {
}

}  // namespace gazebo

