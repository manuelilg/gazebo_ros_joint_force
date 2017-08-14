#include "gazebo_ros_joint_force/gazebo_ros_joint_force.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointForce);

GazeboRosJointForce::GazeboRosJointForce() {
}

GazeboRosJointForce::~GazeboRosJointForce() {
	event::Events::DisconnectWorldUpdateBegin(update_connection_);

	rosQueue_.clear();
	rosQueue_.disable();
	rosNode_->shutdown();
}

void GazeboRosJointForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	this->model_ = _model;
	this->sdf_ = _sdf;

	robotNamespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();

	topicName_ = sdf_->GetElement("topicName")->Get<std::string>();

	if (!sdf_->HasElement("jointName"))
		return;
	auto jointName = sdf_->GetElement("jointName")->Get<std::string>();
	joints_.push_back(model_->GetJoint(jointName)); //TODO test successful

	//TODO begin or end
	update_connection_ = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&GazeboRosJointForce::onWorldUpdate, this));

	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM(
				"A ROS node for Gazegbo has not been initialized, unable to load plugin.");
		return;
	}
	rosNode_.reset(new ros::NodeHandle(robotNamespace_)); // "gazebo_ros_joint_force_plugin"

	auto subOps = ros::SubscribeOptions::create<sensor_msgs::JointState>(
			topicName_, 1,
			boost::bind(&GazeboRosJointForce::onRosMsg, this, _1),
			ros::VoidPtr(), &this->rosQueue_);
	rosSubscriber_ = rosNode_->subscribe(subOps);

	ROS_INFO("gazebo_ros_joint_force plugin successfully loaded");

}

void GazeboRosJointForce::onWorldUpdate() {
	while (rosQueue_.isEmpty()) {
		usleep(1);
	}

	if (rosQueue_.callOne() != ros::CallbackQueue::CallOneResult::Called) {
		ROS_ERROR("Error in gazebo_ros_motor plugin callback not successful called");
	}

	if (joints_.size() == forces_.size()) {
		int i = 0;
		for (auto j : joints_) {
			j->SetForce(0, forces_[i++]);
		}
		forces_.clear(); //TODO costs

	} else {
		//TODO ROSERROR
	}
}

void GazeboRosJointForce::onRosMsg(const sensor_msgs::JointStateConstPtr& msg) {
//	ROS_INFO("onRosMsg called");

	for (auto f : msg->effort) {
		forces_.push_back(f);
	}
//	forces_.push_back(msg->effort[0]); // or checked access at(0)
}

}  // namespace gazebo

