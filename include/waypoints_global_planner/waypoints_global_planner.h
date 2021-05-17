#ifndef WAYPOINTS_GLOBAL_PLANNER_H
#define WAYPOINTS_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
// yaml file handling
#include <yaml-cpp/yaml.h>
#include <fstream>

using std::string;

namespace waypoints_global_planner
{

/**
 * @class CarrotPlanner
 * @brief Provides a simple global planner for producing boustrophedon paths without taking into account obstacles
 */
class WaypointsGlobalPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    /**
     * @brief Default Constructor
     */
    WaypointsGlobalPlanner();

    /**
     * @brief Constructor for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    WaypointsGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Default destructor
     */
    ~WaypointsGlobalPlanner();

    /**
     * @brief Initialization function for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start_pose The starting pose of the robot
     * @param goal The goal pose
     * @param plan The plan filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan) override;

    /**
     * @brief Desired path callback
     * @param plan The received plan
     */
    void desiredPathCallback(const nav_msgs::PathConstPtr& plan);
    void yamlFilePathCallback(const std_msgs::String& path);

  private:
    bool initialized_;  //!< flag indicating the planner has been initialized
    costmap_2d::Costmap2DROS* costmap_ros_;  //!< costmap ros wrapper
    costmap_2d::Costmap2D* costmap_;  //!< costmap container
    base_local_planner::WorldModel* world_model_;  //!< world model
    //std_msgs::String waypoints_filepath_; //!< yaml waypint file path
    std::string map_frame_;

    // subscribers
    ros::Subscriber desired_path_sub_;  //!< subscriber of desired path input
    ros::Subscriber yamlfile_path_sub_; //!< subscriber of yaml file path input
    // publishers
    ros::Publisher goal_pub_;  //!< publisher of goal corresponding to the final waypoint
    ros::Publisher plan_pub_;  //!< publisher of the global plan

    // containers
    std::vector<geometry_msgs::PoseStamped> waypoints_;  //!< container for the manually inserted waypoints
    nav_msgs::Path path_;  //!< container for the generated interpolated path

    bool buildWaypointsFromFile(const char *yaml_filepath); //!< function to build path from yaml file content
};

}  // namespace waypoints_global_planner

#endif  // WAYPOINTS_GLOBAL_PLANNER_H
