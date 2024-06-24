// my_global_planner.cpp

#include "my_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(my_global_planner::MyGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace my_global_planner;

MyGlobalPlanner::MyGlobalPlanner() : costmap_(nullptr), initialized_(false)
{
    ROS_ERROR("hello MyGlobalPlanner");
}

MyGlobalPlanner::MyGlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    initialize(name, costmap_ros);
    ROS_ERROR("hello MyGlobalPlanner initialize");
}

void MyGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
    if (!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        frame_id_ = costmap_ros_->getGlobalFrameID();
        initialized_ = true;
        ROS_ERROR("MyGlobalPlanner initialized");
    }
}

double MyGlobalPlanner::getHeuristic(int x1, int y1, int x2, int y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool MyGlobalPlanner::isCellFree(int x, int y)
{
    unsigned char cost = costmap_->getCost(x, y);
    return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

void MyGlobalPlanner::convertToWorldCoordinates(int x, int y, double &world_x, double &world_y)
{
    costmap_->mapToWorld(x, y, world_x, world_y);
}

bool MyGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
    if (!initialized_)
    {
        ROS_ERROR("MyGlobalPlanner has not been initialized");
        return false;
    }

    plan.clear();
    ROS_INFO("Start making plan");

    // Get start and goal in map coordinates
    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
    {
        ROS_WARN("The start or goal is out of the costmap bounds");
        return false;
    }

    priority_queue<GridCell, vector<GridCell>, greater<GridCell>> open_list;
    vector<vector<bool>> closed_list(costmap_->getSizeInCellsX(), vector<bool>(costmap_->getSizeInCellsY(), false));

    GridCell start_cell(start_x, start_y, 0, getHeuristic(start_x, start_y, goal_x, goal_y), nullptr);
    open_list.push(start_cell);

    while (!open_list.empty())
    {
        GridCell current = open_list.top();
        open_list.pop();

        if (closed_list[current.x][current.y])
        {
            continue;
        }

        closed_list[current.x][current.y] = true;

        if (current.x == goal_x && current.y == goal_y)
        {
            GridCell *cell = &current;
            while (cell != nullptr)
            {
                double world_x, world_y;
                convertToWorldCoordinates(cell->x, cell->y, world_x, world_y);

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = world_x;
                pose.pose.position.y = world_y;
                pose.pose.orientation.w = 1.0;

                plan.push_back(pose);
                cell = cell->parent;
            }

            reverse(plan.begin(), plan.end());
            return true;
        }

        static const int dx[] = {-1, 1, 0, 0, -1, 1, -1, 1};
        static const int dy[] = {0, 0, -1, 1, -1, -1, 1, 1};

        for (int i = 0; i < 8; ++i)
        {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (nx >= 0 && ny >= 0 && nx < costmap_->getSizeInCellsX() && ny < costmap_->getSizeInCellsY())
            {
                if (isCellFree(nx, ny) && !closed_list[nx][ny])
                {
                    double g_cost = current.g_cost + (i < 4 ? 1.0 : sqrt(2.0));
                    double h_cost = getHeuristic(nx, ny, goal_x, goal_y);
                    GridCell neighbor(nx, ny, g_cost, h_cost, new GridCell(current));
                    open_list.push(neighbor);
                }
            }
        }
    }

    ROS_WARN("Failed to find a path");
    return false;
}
