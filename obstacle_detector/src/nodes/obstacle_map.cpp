#include "ros/ros.h"
#include "std_msgs/String.h"
#include "obstacle_detector/Obstacles.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include <math.h>
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"


geometry_msgs::Twist tmp;
geometry_msgs::Twist R_vel;
geometry_msgs::Twist B_vel;
geometry_msgs::Pose R_pose;
geometry_msgs::Pose B_pose;
std_srvs::Empty empty;
ros::Time curr_time;
ros::Time pre_time;
ros::Publisher pre_obstacle_pose_pub;
ros::ServiceClient reset_map_client;
struct BadValue : public std::exception {};


void velocity_CB(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  double roll, pitch, yaw;
  int model_num = msg->name.size();
  for(int i=0;i<model_num;i++)
  {
    if(msg->name[i]=="My Ball2")
    {
      B_vel.linear.x = msg->twist[i].linear.x;
      B_vel.linear.y = msg->twist[i].linear.y;
      B_pose.position = msg->pose[i].position;
      B_pose.orientation = msg->pose[i].orientation;
    }
    if(msg->name[i]=="omni_robot")
    {

      // Get robot's linear and angular speed
      R_vel.linear.x = msg->twist[i].linear.x;
      R_vel.linear.y = msg->twist[i].linear.y;
      R_vel.angular.z = msg->twist[i].angular.z;

      // Get the orientation of robot
      R_pose.position = msg->pose[i].position;
      R_pose.orientation = msg->pose[i].orientation;
    }
  }
  // Transfer from quaternion
  tf::Quaternion q(R_pose.orientation.x,R_pose.orientation.y,R_pose.orientation.z,R_pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  R_vel.linear.z = yaw;
  // printf("R_vel.linear.x:%f\n", R_vel.linear.x);
  // printf("R_vel.linear.y:%f\n", R_vel.linear.y);
  //printf("YAW:%f\n", yaw);
  //printf("R_vel.angular.z:%f\n", R_vel.angular.z);
}

void obstacle_detector_CB(const obstacle_detector::Obstacles::ConstPtr& msg)
{
  int obstacle_num = msg->circles.size();
  int tmp_index = 0;
  double tmp_dist_0 = 20;
  double tmp_dist_1 = 20;
  double tmp_x;
  double tmp_y;

  if(obstacle_num!=0)
  {
    curr_time = ros::Time::now();
    //printf("Obstacles number: %i\n",obstacle_num);
    for(int i=0;i < msg->circles.size();i++)
    {
      // ROS_INFO("I heard: [%f]", msg->circles[i].center.x);
      //ROS_INFO("MSG_sec: [%f]\n", msg->header.stamp.toSec());
      //ROS_INFO("ROS_TIME: [%f]\n", ros_time.toSec());
      tmp_x = msg->circles[i].center.x-R_pose.position.x;
      tmp_y = msg->circles[i].center.y-R_pose.position.y;
      tmp_dist_1 = hypot(tmp_x,tmp_y);
      //printf("tmp_dist_0: %f\n",tmp_dist_0);
      //printf("tmp_dist_1: %f\n",tmp_dist_1);
      if(tmp_dist_0 > tmp_dist_1)
      {
        tmp_index=i;
        tmp_dist_0=tmp_dist_1;
      }
      //printf("Most close is ID_%d\n",tmp_index);
      // curr_time = ros::Time::now();
      // if(curr_time.toSec()!=msg->header.stamp.toSec())
      // {
      //   tmp.linear.x = 20;
      //   tmp.linear.y = 20;
      //   //ROS_INFO("Obstacle_map_TIME: [%f]\n", ros_time.toSec());
      //   //ROS_INFO("MSG_TIME: [%f]\n", msg->header.stamp.toSec());
      //   //ROS_INFO("Topic Obstacle is not update\n");
      // }
    }
    tmp.angular.x = msg->circles[tmp_index].velocity.x;
    tmp.angular.y = msg->circles[tmp_index].velocity.y;
    //tmp.linear.x = msg->circles[tmp_index].center.x;
    //tmp.linear.y = msg->circles[tmp_index].center.y;
    tmp.linear.x = msg->circles[tmp_index].center.x;
    tmp.linear.y = msg->circles[tmp_index].center.y;

    //printf("Obstacle x:%f\tObstacle y:%f\n",tmp.linear.x,tmp.linear.y);
    //printf("Obstacle a_y:%f\n",tmp_a_y);
    double sample_time = curr_time.toSec()-pre_time.toSec();
    //printf("%f,%f,%f,%f,%f,",msg->circles[tmp_index].velocity.x,msg->circles[tmp_index].velocity.y,B_vel.linear.x,B_vel.linear.y,sample_time);
    //printf("%f,%f,%f,%f,%f,%f,%f",msg->circles[tmp_index].center.x,msg->circles[tmp_index].center.y,B_pose.position.x,B_pose.position.y,R_pose.position.x,R_pose.position.y,sample_time);
    pre_time = curr_time;
    pre_obstacle_pose_pub.publish(tmp);
  }else{
    printf("No detect Obstacles!\n");
  }

  //ROS_INFO("Tmp_x: [%f]\tTmp_y: [%f]\n", tmp.linear.x, tmp.linear.y);
  //ROS_INFO("Already_pub\n");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_map");

  ros::NodeHandle n;
  //reset_map_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  pre_obstacle_pose_pub = n.advertise<geometry_msgs::Twist>("pre_obstacles_pose",1);
  ros::Subscriber obstacle_sub = n.subscribe("/obstacles",1,obstacle_detector_CB);
  ros::Subscriber velocity_sub = n.subscribe("/gazebo/model_states",100,velocity_CB);
  //ros::Rate loop_rate(5);

  // while (ros::ok())
  // {
  //   //reset_map_client.call(empty);
  //   ros::spinOnce();
  //
  //   loop_rate.sleep();
  // }
  ros::spin();

  return 0;
}
