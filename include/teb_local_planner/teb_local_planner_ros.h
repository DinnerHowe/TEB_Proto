#pragma once

#include <ros/ros.h>

// base local planner base class and utilities
#include <nav_core/base_local_planner.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>


// timed-elastic-band related classes
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/recovery_behaviors.h>

// message types
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <teb_local_planner/ObstacleMsg.h>

// transforms
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// costmap
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>


// dynamic reconfigure
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>
#include <dynamic_reconfigure/server.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


namespace teb_local_planner {
class TebLocalPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  TebLocalPlannerROS();
  ~TebLocalPlannerROS();

  void initialize(std::string name,
                  tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();

  static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);

  static RobotFootprintModelPtr getRobotFootprintFromParamServer(const ros::NodeHandle& nh);

  static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                                 const std::string& full_param_name);

  static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value,
                                   const std::string& full_param_name);
 protected:
  void updateObstacleContainerWithCostmap();

  void updateObstacleContainerWithCostmapConverter();

  void updateObstacleContainerWithCustomObstacles();

  void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                                double min_separation);

  void reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level);

  void customObstacleCB(const teb_local_planner::ObstacleMsg::ConstPtr& obst_msg);

  bool pruneGlobalPlan(const tf::TransformListener& tf,
                       const tf::Stamped<tf::Pose>& global_pose,
                       std::vector<geometry_msgs::PoseStamped>& global_plan,
                       double dist_behind_robot=1);

  bool transformGlobalPlan(const tf::TransformListener& tf,
                           const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const tf::Stamped<tf::Pose>& global_pose,
                           const costmap_2d::Costmap2D& costmap,
                           const std::string& global_frame,
                           double max_plan_length,
                           std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                           int* current_goal_idx = NULL,
                           tf::StampedTransform* tf_plan_to_global = NULL) const;

  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                      const tf::Stamped<tf::Pose>& local_goal,
                                      int current_goal_idx,
                                      const tf::StampedTransform& tf_plan_to_global,
                                      int moving_average_length=3) const;

  void saturateVelocity(double& vx,
                        double& vy,
                        double& omega,
                        double max_vel_x,
                        double max_vel_y,
                        double max_vel_theta,
                        double max_vel_x_backwards) const;

  double convertTransRotVelToSteeringAngle(double v,
                                           double omega,
                                           double wheelbase,
                                           double min_turning_radius = 0) const;

  void validateFootprints(double opt_inscribed_radius,
                          double costmap_inscribed_radius,
                          double min_obst_dist);

  void configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                            int& goal_idx);

private:
  // external objects (store weak pointers)
  costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  tf::TransformListener* tf_; //!< pointer to Transform Listener

  // internal objects (memory management owned)
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;
  TebConfig cfg_; //!< Config class that stores and manages all related parameters
  FailureDetector failure_detector_; //!< Detect if the robot got stucked

  std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

  base_local_planner::OdometryHelperRos odom_helper_; //!< Provides an interface to receive the current velocity from the robot

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  

  boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
  ros::Subscriber custom_obst_sub_; //!< Subscriber for custom obstacles received via a ObstacleMsg.
  boost::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
  ObstacleMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal
  geometry_msgs::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  bool goal_reached_; //!< store whether the goal is reached or not
  ros::Time time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
  int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
  ros::Time time_last_oscillation_; //!< Store at which time stamp the last oscillation was detected
  RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
  geometry_msgs::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()

  std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot 
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot

  std::string global_frame_; //!< The frame in which the controller will run
  std::string robot_base_frame_; //!< Used as the base frame id of the robot

  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}; // namespace teb_local_planner
