#ifndef _GAZEBO_DIFFDRIVE_PLUGIN_HH_
#define _GAZEBO_DIFFDRIVE_PLUGIN_HH_

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace gazebo
{
  class DiffDrivePlugin : public ModelPlugin
  {
    public: DiffDrivePlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnVelMsg(const geometry_msgs::Twist::ConstPtr& _msg);

    private: ros::NodeHandlePtr node;
    private: ros::Subscriber velSub;

    private: physics::ModelPtr model;
    private: physics::JointPtr leftJoint, rightJoint;
    private: event::ConnectionPtr updateConnection;
    private: double wheelSpeed[2];
    private: double torque;
    private: double wheelSeparation;
    private: double wheelRadius;
    private: common::Time prevUpdateTime;

    private: physics::LinkPtr link, leftWheelLink, rightWheelLink;

    private: double sum;
  };
}
#endif
