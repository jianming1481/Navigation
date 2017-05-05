#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
//#include <gazebo/math/gzmath.hh>

namespace gazebo
{
class WuMotion : public ModelPlugin
{
    ros::NodeHandlePtr nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher pose_pub;
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;
        this->nh.reset(new ros::NodeHandle(this->model->GetName()));

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WuMotion::OnUpdate,this,_1));

        cmd_vel_sub = nh->subscribe("cmd_vel",1,&WuMotion::velCallback, this);
    }

public: void OnUpdate(const common::UpdateInfo &)
    {
    }

    void velCallback(const geometry_msgs::Twist::ConstPtr& twist)
    {
        math::Pose pose(this->model->GetWorldPose());
        this->model->SetLinearVel(pose.rot.RotateVector(math::Vector3(twist->linear.x,twist->linear.y,0)));
        this->model->SetAngularVel(math::Vector3(0,0,twist->angular.z));
    }

private:
    physics::ModelPtr model;
    double x_,rot_;

    math::Pose pose;

private: event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(WuMotion)
}
