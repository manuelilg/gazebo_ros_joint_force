# pragma once

# include <unistd.h>

# include <ros/ros.h>
# include <sensor_msgs/JointState.h>

# include <gazebo/gazebo.hh>
# include <gazebo/physics/physics.hh>

# include <boost/bind.hpp>

namespace gazebo {

class GazeboRosJointForce: public ModelPlugin {
public:
	GazeboRosJointForce();
	virtual ~GazeboRosJointForce();

protected:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
	void onWorldUpdate();

private:
	physics::ModelPtr model_;
	sdf::ElementPtr sdf_;
	event::ConnectionPtr update_connection_;
	std::unique_ptr<ros::NodeHandle> rosNode_; //TODO unique_ptr required
	ros::Subscriber rosSubscriber_;

};

}  // namespace gazebo
