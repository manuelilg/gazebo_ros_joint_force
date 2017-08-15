#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include <unistd.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>


// Usage in URDF:
// <gazebo>
// 	 <plugin name="joint_force" filename="libgazebo_ros_joint_force.so">
// 	 	 <robotNamespace>/eeduro_delta_sim</robotNamespace>
// 	 	 <jointName>arm1_motor_joint, arm2_motor_joint, arm3_motor_joint</jointName>
// 	 </plugin>
// </gazebo>


namespace gazebo {

class GazeboRosJointForce: public ModelPlugin {
public:
	GazeboRosJointForce();
	virtual ~GazeboRosJointForce();

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
	void onWorldUpdate();
	void onRosMsg(const sensor_msgs::JointStateConstPtr& msg);
	void waitForMsg();

private:
	physics::ModelPtr model_;
//	sdf::ElementPtr sdf_;
	event::ConnectionPtr update_connection_;
	std::unique_ptr<ros::NodeHandle> ros_node_; //TODO unique_ptr required
	ros::Subscriber ros_subscriber_;
	ros::CallbackQueue ros_queue_;

	std::string topic_name_;
	std::string robot_namespace_;
	std::vector<std::string> joint_names_;

	std::vector<physics::JointPtr> joints_;
	std::vector<double> forces_;
};

}  // namespace gazebo
