#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/JointWrench.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_plugins/gazebo_ros_utils.h>
//#include <gazebo/math/gzmath.hh>

#define ROBOT_RADIUS 0.19
#define WHEEL_RADIUS 0.06
#define debug 0

const double mAngle1Sin(sin(M_PI/6));
const double mAngle2Sin(sin(5*M_PI/6));
const double mAngle3Sin(sin(3*M_PI/2));
const double mAngle1Cos(cos(M_PI/6));
const double mAngle2Cos(cos(5*M_PI/6));
const double mAngle3Cos(cos(3*M_PI/2));

namespace gazebo
{
class Motion : public ModelPlugin
{
    ros::NodeHandlePtr nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher pose_pub;
public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        this->model = _parent;
        this->nh.reset(new ros::NodeHandle(this->model->GetName()));
        std::cout << "Robot's name is " << this->model->GetName() << std::endl;

        this->joint_controller = new physics::JointController(model);
        this->w1_joint = this->model->GetJoint("base_l_wheel_joint");
        this->w2_joint = this->model->GetJoint("base_r_wheel_joint");
        this->w3_joint = this->model->GetJoint("base_b_wheel_joint");

        this->w1_joint->SetMaxForce(0,10);
        this->w2_joint->SetMaxForce(0,10);
        this->w3_joint->SetMaxForce(0,10);


        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Motion::OnUpdate,this,_1));
        cmd_vel_sub = nh->subscribe("cmd_vel",1,&Motion::velCallback, this);
    }

public: void OnUpdate(const common::UpdateInfo &)
    {
        w1_joint->SetVelocity(0,w1_vel);
        w2_joint->SetVelocity(0,w2_vel);
        w3_joint->SetVelocity(0,w3_vel);
    }

    void velCallback(const geometry_msgs::Twist::ConstPtr& twist)
    {
        //math::Pose pose(this->model->GetWorldPose());

        w1_vel= mAngle1Cos*twist->linear.x + mAngle1Sin*twist->linear.y + ROBOT_RADIUS*twist->angular.z;
        w2_vel= mAngle2Cos*twist->linear.x + mAngle2Sin*twist->linear.y + ROBOT_RADIUS*twist->angular.z;
        w3_vel= mAngle3Cos*twist->linear.x + mAngle3Sin*twist->linear.y + ROBOT_RADIUS*twist->angular.z;


        int times = -5;
        w1_vel = w1_vel*times;
        w2_vel = w2_vel*times;
        w3_vel = w3_vel*times;

        if(debug)
        {
            std::cout << "w1:" << w1_vel << std::endl;
            std::cout << "w2:" << w2_vel << std::endl;
            std::cout << "w3:" << w3_vel << std::endl;
        }
    }

private:
    physics::ModelPtr model;
    physics::JointPtr w1_joint,w2_joint,w3_joint;
    physics::JointController *joint_controller;
    double w1_vel,w2_vel,w3_vel;
    double x_,rot_;

    math::Pose pose;

private: event::ConnectionPtr updateConnection;
};
GZ_REGISTER_MODEL_PLUGIN(Motion)
}
