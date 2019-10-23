#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>


using namespace teb_local_planner;

PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber clicked_points_sub;
unsigned int no_fixed_obstacles;

void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const teb_local_planner::ObstacleMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);


int main( int argc, char** argv ) {
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");

  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);

  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);

  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);

  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  obst_vector.push_back(boost::make_shared<PointObstacle>(-5,1));
  obst_vector.push_back(boost::make_shared<PointObstacle>(-5,2.2));
  obst_vector.push_back(boost::make_shared<PointObstacle>(0,0.1));

  for (unsigned int i=0; i<obst_vector.size(); ++i) {
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst) {
      CreateInteractiveMarker(pobst->x(),
                              pobst->y(),
                              i,
                              config.map_frame,
                              &marker_server,
                              &CB_obstacle_marker);
    }
  }
  marker_server.applyChanges();

  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config.map_frame));

  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning) {
    // use parallels planning
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config,
                                                           &obst_vector,
                                                           robot_model,
                                                           visual,
                                                           &via_points));
  } else {
    // use single planning
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config,
                                                        &obst_vector,
                                                        robot_model,
                                                        visual,
                                                        &via_points));
  }

  no_fixed_obstacles = obst_vector.size();
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e) {
  planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0));
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e) {
  planner->visualize();
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig,
                    uint32_t level) {
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x,
                             const double& init_y,
                             unsigned int id,
                             std::string frame,
                             interactive_markers::InteractiveMarkerServer* marker_server,
                             interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb) {
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 1;
  move_control.orientation.x = 0;
  move_control.orientation.y = 1;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  if (index>=no_fixed_obstacles) {
    return;
  }
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const teb_local_planner::ObstacleMsg::ConstPtr& obst_msg) {
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
  for (std::vector<geometry_msgs::PolygonStamped>::const_iterator obst_it = obst_msg->obstacles.begin();
        obst_it != obst_msg->obstacles.end(); ++obst_it) {
    if (obst_it->polygon.points.size() == 1 ) {
      obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_it->polygon.points.front().x,
                                                           obst_it->polygon.points.front().y )));
    } else {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (int i=0; i<(int)obst_it->polygon.points.size(); ++i) {
        polyobst->pushBackVertex( obst_it->polygon.points[i].x, obst_it->polygon.points[i].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }
  }
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg) {
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0) {
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
  }
}
