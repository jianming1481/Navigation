#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

namespace grid_layer_namespace
{

class GridLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GridLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void obstacle_detector_CB(const obstacle_detector::Obstacles::ConstPtr& msg);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

  ros::Subscriber obstacle_sub;
  obstacle_detector::Obstacles obstacles_;

  double tmp_x;
  double tmp_y;
  double mark_x;
  double mark_y;
  int pre_mx[90];
  int pre_my[90];
};
}
#endif
