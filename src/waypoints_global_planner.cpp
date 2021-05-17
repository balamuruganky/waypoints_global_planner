/**
Copyright (c) 2021 Balamurugan Kandan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**/

#include "waypoints_global_planner/waypoints_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(waypoints_global_planner::WaypointsGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace waypoints_global_planner
{

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

WaypointsGlobalPlanner::WaypointsGlobalPlanner() : costmap_ros_(NULL), initialized_(false)
{
}

WaypointsGlobalPlanner::WaypointsGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

WaypointsGlobalPlanner::~WaypointsGlobalPlanner()
{
}

void WaypointsGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // Parameter list
    pnh.param<std::string>("map_frame", map_frame_, "map"); 

    // Subscribe desired path and file path to populate the plan
    desired_path_sub_ = pnh.subscribe("/desired_path", 1, &WaypointsGlobalPlanner::desiredPathCallback, this);
    yamlfile_path_sub_ = pnh.subscribe("/yaml_filepath", 1, &WaypointsGlobalPlanner::yamlFilePathCallback, this);

    //
    // Publish plan and goal
    //
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    initialized_ = true;
    ROS_INFO("Planner has been initialized");
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
}

//
// Base class override start
//
bool WaypointsGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan_pub_.publish(path_);
  plan = path_.poses;
  ROS_INFO("Published global plan");
  return true;
}
//
// Base class override end
//

void WaypointsGlobalPlanner::desiredPathCallback(const nav_msgs::PathConstPtr& plan)
{
  path_.poses.clear();

  path_.header.frame_id = map_frame_;
  path_.header.stamp = ros::Time::now();
  path_.poses = plan->poses;
  goal_pub_.publish(path_.poses.back());
}

void WaypointsGlobalPlanner::yamlFilePathCallback(const std_msgs::String& yaml_filepath)
{
  if (false == buildWaypointsFromFile((yaml_filepath.data).c_str())) {
    ROS_ERROR("Unable to process the YAML file.");
  }
}

/*
 * Build waypoints from file
 */
bool WaypointsGlobalPlanner::buildWaypointsFromFile(const char *waypoints_filename)
{
  // clear way points vector
  path_.poses.clear();

  //std::string waypoints_path_filename;
  //std::string pkg_path = ros::package::getPath( "waypoints_global_planner" );
  //waypoints_path_filename = pkg_path + "/" + waypoints_filename;

  try
  {
    // check file open
    std::ifstream ifs( waypoints_filename, std::ifstream::in );
    if( !ifs.good() )
    {
      return false;
    }
    // yaml node
    YAML::Node yaml_node;
    #ifdef HAVE_NEW_YAMLCPP
      yaml_node = YAML::Load(ifs);
      const YAML::Node &wp_node_tmp= yaml_node[ "waypoints" ];
      const YAML::Node *wp_node= wp_node_tmp ? &wp_node_tmp : NULL;
    #else
      YAML::Parser parser(ifs);
      parser.GetNextDocument(yaml_node);
      const YAML::Node* wp_node = yaml_node.FindValue("waypoints");
    #endif

    if(wp_node != NULL)
    {
      std::vector<geometry_msgs::PoseStamped> waypoints;
      double yaw = 0.0;
      // loop over all the waypoints
      for(int i=0; i < wp_node->size(); i++)
      {
        // get each waypoint
        geometry_msgs::PoseStamped pose;
        (*wp_node)[i]["point"]["x"] >> pose.pose.position.x;
        (*wp_node)[i]["point"]["y"] >> pose.pose.position.y;
        (*wp_node)[i]["point"]["yaw"] >> yaw;
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY( 0., 0., yaw );
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pose.header.frame_id = map_frame_;
        pose.header.stamp = ros::Time::now();
        waypoints.push_back(pose);
      }
      path_.header.frame_id = map_frame_;
      path_.header.stamp = ros::Time::now();
      path_.poses = waypoints;
      goal_pub_.publish(path_.poses.back());
    }
    else
    {
      ROS_ERROR("Invalid YAML Node");
      return false;
    }
  }
  catch(YAML::ParserException &e)
  {
      ROS_ERROR("Parser Exception : %s", e.what());
      return false;
  }
  catch(YAML::RepresentationException &e)
  {
      ROS_ERROR("Parser Exception : %s", e.what());
      return false;
  }

  return true;
}

}
