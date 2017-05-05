#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <stdio.h>
#include <std_msgs/Int32.h>
#include <obstacle_detector/Obstacles.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

namespace simple_layer_namespace
{

class SimpleLayer : public costmap_2d::Layer
{
public:
  SimpleLayer();
  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  //void obstacle_detector_CB(const obstacle_detector::Obstacles::ConstPtr& msg);
  void obstacle_detector_CB(const geometry_msgs::Twist::ConstPtr& msg);

  double mark_x_, mark_y_;
  double tmp_x,tmp_y;
  std_srvs::Empty reset_map_empty;
  obstacle_detector::Obstacles obstacles_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  ros::Subscriber obstacle_sub;
  ros::ServiceClient reset_map_client;
};
}
#endif
