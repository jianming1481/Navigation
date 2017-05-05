#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"

ros::Time cur_time;
ros::Time pre_time;
void model_states_CB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  int model_num = msg->name.size();
  geometry_msgs::Pose Ball_pose;
  geometry_msgs::Pose R_pose;

  for(int i=0;i<model_num;i++)
  {
    if(msg->name[i]=="My Ball")
    {
      Ball_pose = msg->pose[i];
    }

    if(msg->name[i]=="omni_robot")
    {
      R_pose = msg->pose[i];
    }
  }
  cur_time = ros::Time::now();
  double sample_time = (cur_time.toSec()-pre_time.toSec());
  if(sample_time>0.5)
  {
    printf("%f,%f,%f,%f,%f", Ball_pose.position.x, Ball_pose.position.y, R_pose.position.x, R_pose.position.y,sample_time);
    pre_time = cur_time;
  }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;
  ros::Subscriber velocity_sub = n.subscribe("/gazebo/model_states",100,model_states_CB);

  ros::spin();

  return 0;
}
