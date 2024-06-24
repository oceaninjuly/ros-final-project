// my_global_planner.h

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/transform_broadcaster.h>
#include <queue>
#include <vector>
#include <cmath>

namespace my_global_planner {

struct GridCell {
  int x, y;
  double g_cost, h_cost;
  GridCell* parent;

  GridCell(int x, int y, double g, double h, GridCell* p)
    : x(x), y(y), g_cost(g), h_cost(h), parent(p) {}

  double getFCost() const {
    return g_cost + h_cost;
  }

  bool operator>(const GridCell& other) const {
    return this->getFCost() > other.getFCost();
  }
};

class MyGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
  MyGlobalPlanner();
  MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

protected:
  costmap_2d::Costmap2D* costmap_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::string frame_id_;
  ros::Publisher plan_pub_;
  double step_size_, min_dist_from_robot_;
  bool initialized_ = false;

private:
  double getHeuristic(int x1, int y1, int x2, int y2);
  bool isCellFree(int x, int y);
  void convertToWorldCoordinates(int x, int y, double& world_x, double& world_y);
};

} // namespace my_global_planner

#endif // GLOBAL_PLANNER_CPP

