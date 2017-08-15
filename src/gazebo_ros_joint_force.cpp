#include "gazebo_ros_joint_force/gazebo_ros_joint_force.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointForce);

GazeboRosJointForce::GazeboRosJointForce() {
}

GazeboRosJointForce::~GazeboRosJointForce() {
	event::Events::DisconnectWorldUpdateBegin(update_connection_);

	ros_queue_.clear();
	ros_queue_.disable();
	ros_node_->shutdown();
}

void GazeboRosJointForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	this->model_ = _model;

	this->robot_namespace_ = this->model_->GetName() + "/";
	if (!_sdf->HasElement("robotNamespace")) {
		ROS_INFO_NAMED("joint_force", "GazeboRosJointForce Plugin missing <robotNamespace>, default to \"%s\"", this->robot_namespace_.c_str());
	} else {
		auto robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
		if (!robot_namespace.empty()) {
			this->robot_namespace_ = robot_namespace + "/";
		}
	}

	if (!_sdf->HasElement("jointName")) {
		ROS_ASSERT("GazeboRosJointForce Plugin missing <jointName>");
	} else {
		auto joint_names = _sdf->GetElement("jointName")->Get<std::string>();
		boost::erase_all(joint_names, " ");
		boost::split(this->joint_names_, joint_names, boost::is_any_of(","));
	}


	for (auto jn : joint_names_) {
		this->joints_.push_back(model_->GetJoint(jn));
		this->forces_.push_back(0.0);
		ROS_INFO_NAMED("joint_force", "GazeboRosJointForce Plugin is going to set Force on joint: \"%s\"", jn.c_str());
	}


	this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosJointForce::onWorldUpdate, this));

	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("joint_force" ,"A ROS node for Gazegbo has not been initialized, unable to load plugin joint_force. "
				<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
		return;
	}

	this->ros_node_.reset(new ros::NodeHandle(this->robot_namespace_));

	ROS_INFO_NAMED("joint_force", "Starting GazeboRosJointForce Plugin (ns = %s)!, model name: %s", this->robot_namespace_.c_str(), this->model_->GetName().c_str() );

	auto subOps = ros::SubscribeOptions::create<sensor_msgs::JointState>(
			this->topic_name_, 1,
			boost::bind(&GazeboRosJointForce::onRosMsg, this, _1),
			ros::VoidPtr(), &this->ros_queue_);
	this->ros_subscriber_ = this->ros_node_->subscribe(subOps);

	ROS_INFO_NAMED("joint_force", "GazeboRosJointForce plugin successfully loaded");
}


void GazeboRosJointForce::onWorldUpdate() {
	waitForMsg();

	if (ros_queue_.callOne() != ros::CallbackQueue::CallOneResult::Called) {
		ROS_ERROR_NAMED("joint_force", "Error in GazeboRosJointForce plugin callback not successful called");
	}

	unsigned int i = 0;
	for (auto joint : this->joints_) {
		joint->SetForce(0, this->forces_[i++]);
	}
}


void GazeboRosJointForce::onRosMsg(const sensor_msgs::JointStateConstPtr& _msg) {
	if(!_msg->name.empty()) {
		for (auto jointName : _msg->name) {
			auto it = std::find(this->joint_names_.begin(), this->joint_names_.end(), jointName);

			if(it != this->joint_names_.end()) {
				auto pos = std::distance(this->joint_names_.begin(), it);
				this->forces_.at(pos);
			} else {
				ROS_ERROR_NAMED("joint_force", "joint: \"%s\" not found, add joint to plugin settings <jointName>", jointName.c_str());
			}
		}

	} else {
		unsigned int i = 0;
		for(auto f : _msg->effort) {
			this->forces_.at(i) = f;
			i++;
		}
	}
}


void GazeboRosJointForce::waitForMsg() {
	while (this->ros_queue_.isEmpty()) {
		usleep(1);
	}
}

}  // namespace gazebo

