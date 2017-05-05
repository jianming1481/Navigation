#include<grid_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(grid_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace grid_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();
  pre_mx[90] = {0};
  pre_my[90] = {0};

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  //obstacle_sub = nh.subscribe("/pre_obstacles_pose",1,&GridLayer::obstacle_detector_CB, this);
  obstacle_sub = nh.subscribe("/obstacles",1,&GridLayer::obstacle_detector_CB, this);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::obstacle_detector_CB(const obstacle_detector::Obstacles::ConstPtr& msg) {
  int obstacle_num = msg->circles.size();
  if(obstacle_num!=0)
  {
    obstacles_.circles = msg->circles;
  }
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  int obstacle_num = obstacles_.circles.size();
  int t=5;
  printf("Obstacles number: %i\n",obstacle_num);
  /*for(int i=0;i<5;i++)
  {
    mark_x = i;
    mark_y = i;
    unsigned int mx;
    unsigned int my;
    if(worldToMap(mark_x, mark_y, mx, my)){
      setCost(mx, my, LETHAL_OBSTACLE);
    }else{
      printf("FAIL!\n");
    }
  }*/


  for(int i=0;i<obstacle_num;i++)
  {

    mark_x = obstacles_.circles[i].center.x+5-robot_x + obstacles_.circles[i].velocity.x*t;
    mark_y = obstacles_.circles[i].center.y+5-robot_y + obstacles_.circles[i].velocity.y*t;
    //printf("i=%i\t mark_x: %f, mark_y:%f\n",i,mark_x,mark_y);
    unsigned int mx;
    unsigned int my;
    setCost(pre_mx[i], pre_my[i], FREE_SPACE);
    printf("Outside pre_mx: %i\t",pre_mx[i]);
    printf("Outside pre_my: %i\n",pre_my[i]);
    if(worldToMap(mark_x, mark_y, mx, my)){
      //printf("Obstacle_%i: velocity_x: %f\n",i,obstacles_.circles[i].velocity.x);
      //printf("Obstacle_%i: velocity_y: %f\n",i,obstacles_.circles[i].velocity.y);
      setCost(mx, my, LETHAL_OBSTACLE);
      pre_mx[i] = mx;
      pre_my[i] = my;
      printf("In pre_mx: %i\t",pre_mx[i]);
      printf("In pre_my: %i\n",pre_my[i]);
    }else{
      printf("Obstacle_%i: FAIL!\n",i);
    }

  }
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace
