
#include <costmap_2d/forbidden_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ForbiddenLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;
using Point = std::pair<double, double>;
using Points = std::vector<Point>;
using MapPoint = std::pair<int, int>;
using MapPoints = std::vector<MapPoint>;

namespace costmap_2d
{


ForbiddenLayer::ForbiddenLayer() {}

void ForbiddenLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &ForbiddenLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void ForbiddenLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void ForbiddenLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void ForbiddenLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{   

}

void ForbiddenLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned int mx, my;
  unsigned int  umin_i = 1e5;
  unsigned int  umin_j = 1e5;
  unsigned int  umax_i = 0;
  unsigned int  umax_j = 0;

  Points forbidden_area { {-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0} };
  for (const auto& point : forbidden_area) {
    if (worldToMap(point.first, point.second, mx, my)) {
        // std::cout << " ( " << point.first << " , "<< point.second << " ) --> (mx, my): " << mx << " , " << my << std::endl;
        // std::cout << "umin_i: " << umin_i << " umin_j: " << umin_j << " umax_i: " << umax_i << " umax_j: " << umax_j << std::endl;
        umin_i = std::min(umin_i, mx);
        umin_j = std::min(umin_j, my);
        umax_i = std::max(umax_i, mx);
        umax_j = std::max(umax_j, my);
        // std::cout << "umin_i: " << umin_i << " umin_j: " << umin_j << " umax_i: " << umax_i << " umax_j: " << umax_j << std::endl;
    }
  }

  for (int j = umin_j; j < umax_j; j++)
  {
    for (int i = umin_i; i < umax_i; i++)
    {
      master_grid.setCost(i, j, LETHAL_OBSTACLE); 
    }
  }
}

} // end namespace