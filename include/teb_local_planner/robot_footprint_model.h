#ifndef ROBOT_FOOTPRINT_MODEL_H
#define ROBOT_FOOTPRINT_MODEL_H

#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/obstacles.h>
#include <visualization_msgs/Marker.h>

namespace teb_local_planner
{

/**
 * @class BaseRobotFootprintModel
 * @brief Abstract class that defines the interface for robot footprint/contour models
 * 
 * The robot model class is currently used in optimization only, since
 * taking the navigation stack footprint into account might be
 * inefficient. The footprint is only used for checking feasibility.
 */
class BaseRobotFootprintModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    */
  BaseRobotFootprintModel()
  {
  }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~BaseRobotFootprintModel()
  {
  }


  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const = 0;

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const {}
  
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() = 0;

	

public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const BaseRobotFootprintModel> RobotFootprintModelConstPtr;



/**
 * @class PointRobotShape
 * @brief Class that defines a point-robot
 * 
 * Instead of using a CircularRobotFootprint this class might
 * be utitilzed and the robot radius can be added to the mininum distance 
 * parameter. This avoids a subtraction of zero each time a distance is calculated.
 */
class PointRobotFootprint : public BaseRobotFootprintModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    */
  PointRobotFootprint() {}

  /**
    * @brief Default constructor of the abstract obstacle class
    * @param min_obstacle_dist Minimum obstacle distance
    */
  PointRobotFootprint(const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist) {}
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~PointRobotFootprint() {}

  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    return obstacle->getMinimumDistance(current_pose.position());
  }
  
  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t);
  }

  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() {return 0.0;}

  /**
   * @brief Visualize the robot using a markers
   * 
   * Fill a marker message with all necessary information (type, pose, scale and color).
   * The header, namespace, id and marker lifetime will be overwritten.
   * @param current_pose Current robot pose
   * @param[out] markers container of marker messages describing the robot shape
   * @param color Color of the footprint
   */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {
    // point footprint
    markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = markers.back();
    marker.type = visualization_msgs::Marker::POINTS;
    current_pose.toPoseMsg(marker.pose); // all points are transformed into the robot frame!
    marker.points.push_back(geometry_msgs::Point());
    marker.scale.x = 0.025; 
    marker.color = color;

    if (min_obstacle_dist_ <= 0)
    {
      return;
    }

    // footprint with min_obstacle_dist
    markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker2 = markers.back();
    marker2.type = visualization_msgs::Marker::LINE_STRIP;
    marker2.scale.x = 0.025; 
    marker2.color = color;
    current_pose.toPoseMsg(marker2.pose); // all points are transformed into the robot frame!

    const double n = 9;
    const double r = min_obstacle_dist_;
    for (double theta = 0; theta <= 2 * M_PI; theta += M_PI / n)
    {
      geometry_msgs::Point pt;
      pt.x = r * cos(theta);
      pt.y = r * sin(theta);
      marker2.points.push_back(pt);
    }
  }

private:
  const double min_obstacle_dist_ = 0.0;
};


/**
 * @class CircularRobotFootprint
 * @brief Class that defines the a robot of circular shape
 */
class CircularRobotFootprint : public BaseRobotFootprintModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param radius radius of the robot
    */
  CircularRobotFootprint(double radius) : radius_(radius) { }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~CircularRobotFootprint() { }

  /**
    * @brief Set radius of the circular robot
    * @param radius radius of the robot
    */
  void setRadius(double radius) {radius_ = radius;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    return obstacle->getMinimumDistance(current_pose.position()) - radius_;
  }

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t) - radius_;
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {
    markers.resize(1);
    visualization_msgs::Marker& marker = markers.back();
    marker.type = visualization_msgs::Marker::CYLINDER;
    current_pose.toPoseMsg(marker.pose);
    marker.scale.x = marker.scale.y = 2*radius_; // scale = diameter
    marker.scale.z = 0.05;
    marker.color = color;
  }
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() {return radius_;}

private:
    
  double radius_;
};


/**
 * @class TwoCirclesRobotFootprint
 * @brief Class that approximates the robot with two shifted circles
 */
class TwoCirclesRobotFootprint : public BaseRobotFootprintModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
    * @param front_radius radius of the front circle
    * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
    * @param rear_radius radius of the front circle
    */
  TwoCirclesRobotFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius) 
    : front_offset_(front_offset), front_radius_(front_radius), rear_offset_(rear_offset), rear_radius_(rear_radius) { }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~TwoCirclesRobotFootprint() { }

  /**
   * @brief Set parameters of the contour/footprint
   * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) 
  {front_offset_=front_offset; front_radius_=front_radius; rear_offset_=rear_offset; rear_radius_=rear_radius;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front = obstacle->getMinimumDistance(current_pose.position() + front_offset_*dir) - front_radius_;
    double dist_rear = obstacle->getMinimumDistance(current_pose.position() - rear_offset_*dir) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() + front_offset_*dir, t) - front_radius_;
    double dist_rear = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() - rear_offset_*dir, t) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {    
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    if (front_radius_>0)
    {
      markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker1 = markers.back();
      marker1.type = visualization_msgs::Marker::CYLINDER;
      current_pose.toPoseMsg(marker1.pose);
      marker1.pose.position.x += front_offset_*dir.x();
      marker1.pose.position.y += front_offset_*dir.y();
      marker1.scale.x = marker1.scale.y = 2*front_radius_; // scale = diameter
      marker1.pose.orientation.w = 1;
//       marker1.scale.z = 0.05;
      marker1.color = color;

    }
    if (rear_radius_>0)
    {
      markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker2 = markers.back();
      marker2.type = visualization_msgs::Marker::CYLINDER;
      current_pose.toPoseMsg(marker2.pose);
      marker2.pose.position.x -= rear_offset_*dir.x();
      marker2.pose.position.y -= rear_offset_*dir.y();
      marker2.pose.orientation.w = 1;
      marker2.scale.x = marker2.scale.y = 2*rear_radius_; // scale = diameter
//       marker2.scale.z = 0.05;
      marker2.color = color;
    }
  }
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() 
  {
      double min_longitudinal = std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
      double min_lateral = std::min(rear_radius_, front_radius_);
      return std::min(min_longitudinal, min_lateral);
  }

private:
    
  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;
  
};



/**
 * @class LineRobotFootprint
 * @brief Class that approximates the robot with line segment (zero-width)
 */
class LineRobotFootprint : public BaseRobotFootprintModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param line_start start coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
    * @param line_end end coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
    */
  LineRobotFootprint(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end)
  {
    setLine(line_start, line_end);
  }
  
  /**
  * @brief Default constructor of the abstract obstacle class (Eigen Version)
  * @param line_start start coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
  * @param line_end end coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
  */
  LineRobotFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, const double min_obstacle_dist) : min_obstacle_dist_(min_obstacle_dist)
  {
    setLine(line_start, line_end);
  }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~LineRobotFootprint() { }

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setLine(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end)
  {
    line_start_.x() = line_start.x; 
    line_start_.y() = line_start.y; 
    line_end_.x() = line_end.x;
    line_end_.y() = line_end.y;
  }
  
  /**
   * @brief Set vertices of the contour/footprint (Eigen version)
   * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end)
  {
    line_start_ = line_start; 
    line_end_ = line_end;
  }
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumDistance(line_start_world, line_end_world);
  }

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    Eigen::Vector2d line_start_world;
    Eigen::Vector2d line_end_world;
    transformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->getMinimumSpatioTemporalDistance(line_start_world, line_end_world, t);
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {   
    markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = markers.back();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    current_pose.toPoseMsg(marker.pose); // all points are transformed into the robot frame!
    
    // line
    geometry_msgs::Point line_start_world;
    line_start_world.x = line_start_.x();
    line_start_world.y = line_start_.y();
    line_start_world.z = 0;
    marker.points.push_back(line_start_world);
    
    geometry_msgs::Point line_end_world;
    line_end_world.x = line_end_.x();
    line_end_world.y = line_end_.y();
    line_end_world.z = 0;
    marker.points.push_back(line_end_world);

    marker.scale.x = marker_line_width; 
    marker.color = color;    

    if (min_obstacle_dist_ <= 0)
    {
      return;
    }

    // footprint with min_obstacle_dist
    markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker2 = markers.back();
    marker2.type = visualization_msgs::Marker::LINE_STRIP;
    marker2.scale.x = marker_line_width; 
    marker2.color = color;
    current_pose.toPoseMsg(marker2.pose); // all points are transformed into the robot frame!

    const double n = 9;
    const double r = min_obstacle_dist_;
    const double ori = atan2(line_end_.y() - line_start_.y(), line_end_.x() - line_start_.x());

    // first half-circle
    for (double theta = M_PI_2 + ori; theta <= 3 * M_PI_2 + ori; theta += M_PI / n)
    {
      geometry_msgs::Point pt;
      pt.x = line_start_.x() + r * cos(theta);
      pt.y = line_start_.y() + r * sin(theta);
      marker2.points.push_back(pt);
    }

    // second half-circle
    for (double theta = -M_PI_2 + ori; theta <= M_PI_2 + ori; theta += M_PI / n)
    {
      geometry_msgs::Point pt;
      pt.x = line_end_.x() + r * cos(theta);
      pt.y = line_end_.y() + r * sin(theta);
      marker2.points.push_back(pt);
    }

    // duplicate 1st point to close shape
    geometry_msgs::Point pt;
    pt.x = line_start_.x() + r * cos(M_PI_2 + ori);
    pt.y = line_start_.y() + r * sin(M_PI_2 + ori);
    marker2.points.push_back(pt);
  }
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() 
  {
      return 0.0; // lateral distance = 0.0
  }

private:
    
  /**
    * @brief Transforms a line to the world frame manually
    * @param current_pose Current robot pose
    * @param[out] line_start line_start_ in the world frame
    * @param[out] line_end line_end_ in the world frame
    */
  void transformToWorld(const PoseSE2& current_pose, Eigen::Vector2d& line_start_world, Eigen::Vector2d& line_end_world) const
  {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    line_start_world.x() = current_pose.x() + cos_th * line_start_.x() - sin_th * line_start_.y();
    line_start_world.y() = current_pose.y() + sin_th * line_start_.x() + cos_th * line_start_.y();
    line_end_world.x() = current_pose.x() + cos_th * line_end_.x() - sin_th * line_end_.y();
    line_end_world.y() = current_pose.y() + sin_th * line_end_.x() + cos_th * line_end_.y();
  }

  Eigen::Vector2d line_start_;
  Eigen::Vector2d line_end_;
  const double min_obstacle_dist_ = 0.0;
  const double marker_line_width = 0.2;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
};



/**
 * @class PolygonRobotFootprint
 * @brief Class that approximates the robot with a closed polygon
 */
class PolygonRobotFootprint : public BaseRobotFootprintModel
{
public:
  
  
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
    */
  PolygonRobotFootprint(const Point2dContainer& vertices) : vertices_(vertices) { }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~PolygonRobotFootprint() { }

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
   */
  void setVertices(const Point2dContainer& vertices) {vertices_ = vertices;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    Point2dContainer polygon_world(vertices_.size());
    transformToWorld(current_pose, polygon_world);
    return obstacle->getMinimumDistance(polygon_world);
  }

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    Point2dContainer polygon_world(vertices_.size());
    transformToWorld(current_pose, polygon_world);
    return obstacle->getMinimumSpatioTemporalDistance(polygon_world, t);
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {
    if (vertices_.empty())
      return;

    markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = markers.back();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    current_pose.toPoseMsg(marker.pose); // all points are transformed into the robot frame!
    
    for (std::size_t i = 0; i < vertices_.size(); ++i)
    {
      geometry_msgs::Point point;
      point.x = vertices_[i].x();
      point.y = vertices_[i].y();
      point.z = 0;
      marker.points.push_back(point);
    }
    // add first point again in order to close the polygon
    geometry_msgs::Point point;
    point.x = vertices_.front().x();
    point.y = vertices_.front().y();
    point.z = 0;
    marker.points.push_back(point);

    marker.scale.x = marker_line_width; 
    marker.color = color;

  }
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() 
  {
     double min_dist = std::numeric_limits<double>::max();
     Eigen::Vector2d center(0.0, 0.0);
      
     if (vertices_.size() <= 2)
        return 0.0;

     for (int i = 0; i < (int)vertices_.size() - 1; ++i)
     {
        // compute distance from the robot center point to the first vertex
        double vertex_dist = vertices_[i].norm();
        double edge_dist = distance_point_to_segment_2d(center, vertices_[i], vertices_[i+1]);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
     }
 
     // we also need to check the last vertex and the first vertex
     double vertex_dist = vertices_.back().norm();
     double edge_dist = distance_point_to_segment_2d(center, vertices_.back(), vertices_.front());
     return std::min(min_dist, std::min(vertex_dist, edge_dist));
  }

private:
    
  /**
    * @brief Transforms a polygon to the world frame manually
    * @param current_pose Current robot pose
    * @param[out] polygon_world polygon in the world frame
    */
  void transformToWorld(const PoseSE2& current_pose, Point2dContainer& polygon_world) const
  {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());
    for (std::size_t i=0; i<vertices_.size(); ++i)
    {
      polygon_world[i].x() = current_pose.x() + cos_th * vertices_[i].x() - sin_th * vertices_[i].y();
      polygon_world[i].y() = current_pose.y() + sin_th * vertices_[i].x() + cos_th * vertices_[i].y();
    }
  }

  Point2dContainer vertices_;
  const double marker_line_width = 0.2;
  
};


/**
 * @class CenterArticulatedRobotFootprint
 * @brief Class that approximates a center ariculated robot with two boxes approach
 */
class CenterArticulatedRobotFootprint : public BaseRobotFootprintModel
{
public:

  struct CenterArticulatedModel
  {
    double ff_length; //!< Distance from center articulation to front-front tip
    double fr_length; //!< Distance from center articulation to front-front rear
    double rf_length; //!< Distance from center articulation to rear-front tip
    double rr_length; //!< Distance from center articulation to rear-rear tip
    double f_width; //!< Widht of front reactangle
    double r_width; //!< Widht of rear reactangle
  }; //!< Trajectory related parameters
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
    */
  CenterArticulatedRobotFootprint(const CenterArticulatedModel& model){model_ = model;}
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~CenterArticulatedRobotFootprint() { }

  /**
   * @brief Set vertices of the contour/footprint
   * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
   */
  void updateModel(const CenterArticulatedModel& model) {model_ = model;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    // bool debug_msg = obstacle->getCentroid().x()>13 && obstacle->getCentroid().x()<15 && obstacle->getCentroid().y()>29 && obstacle->getCentroid().y()<32;
    bool debug_msg = false;
    Point2dContainer front_polygon;
    Point2dContainer rear_polygon;
    transformToWorldFront(current_pose, front_polygon);
    transformToWorldRear(current_pose, rear_polygon);
    Eigen::Vector2d current_pose_position = current_pose.position();
    Eigen::Vector2d obstacle_point = obstacle->getCentroid();
    Eigen::Vector2d diff;
    diff.x() = (obstacle_point.x()-current_pose.x());
    diff.y() = (obstacle_point.y()-current_pose.y());
    ROS_INFO_COND(debug_msg, "    current_pose_position = %.3f, %.3f. %.3f, %.3f", current_pose_position.x(), current_pose_position.y(), current_pose.theta(), current_pose.steering_pos());
    ROS_INFO_COND(debug_msg, "    diff = %.3f, %.3f", diff.x(), diff.y());
    ROS_INFO_COND(debug_msg, "    obstacle_point = %.3f, %.3f", obstacle_point.x(), obstacle_point.y());
    // obstacle_point
    {
      Eigen::Vector2d obstacle_point_transformed_front = obstacle->getCentroid();
      double cos_th = std::cos(current_pose.theta());
      double sin_th = std::sin(current_pose.theta());
      obstacle_point_transformed_front.x() = cos_th * (diff.x()) + sin_th * (diff.y());
      obstacle_point_transformed_front.y() = - sin_th * (diff.x()) + cos_th * (diff.y());
      ROS_INFO_COND(debug_msg, "    FRONT: obstacle_point_transformed_front = %.3f, %.3f", obstacle_point_transformed_front.x(), obstacle_point_transformed_front.y());
      if(obstacle_point_transformed_front.x() >= model_.fr_length &&
          obstacle_point_transformed_front.x() <= model_.ff_length &&
          obstacle_point_transformed_front.y() >= -model_.f_width/2 &&
          obstacle_point_transformed_front.y() <= model_.f_width/2
          )
      {
        ROS_INFO_COND(debug_msg, "    FRONT COLLISION: theta, steering_pos = %.3f, %.3f", current_pose.theta(), current_pose.steering_pos());
        ROS_INFO_COND(debug_msg, "    FRONT COLLISION: cos_th, sin_th = %.3f, %.3f", cos_th, sin_th);
        return 0.0;
      }
    }
    // Rear
    {
      Eigen::Vector2d obstacle_point_transformed_rear = obstacle->getCentroid();
      double cos_th = std::cos(current_pose.theta()-current_pose.steering_pos());
      double sin_th = std::sin(current_pose.theta()-current_pose.steering_pos());
      obstacle_point_transformed_rear.x() = cos_th * (diff.x()) + sin_th * (diff.y());
      obstacle_point_transformed_rear.y() = - sin_th * (diff.x()) + cos_th * (diff.y());
      ROS_INFO_COND(debug_msg, "    REAR: obstacle_point_transformed_rear = %.3f, %.3f", obstacle_point_transformed_rear.x(), obstacle_point_transformed_rear.y());
      if(obstacle_point_transformed_rear.x() >= -model_.rr_length &&
        obstacle_point_transformed_rear.x() <= -model_.rf_length &&
        obstacle_point_transformed_rear.y() >= -model_.r_width/2 &&
        obstacle_point_transformed_rear.y() <= model_.r_width/2
        )
      {
        ROS_INFO_COND(debug_msg, "    REAR COLLISION: obstacle_point_transformed_rear = %.3f, %.3f", obstacle_point_transformed_rear.x(), obstacle_point_transformed_rear.y());
        ROS_INFO_COND(debug_msg, "    REAR COLLISION: theta, steering_pos = %.3f, %.3f", current_pose.theta(), current_pose.steering_pos());
        ROS_INFO_COND(debug_msg, "    REAR COLLISION: cos_th, sin_th = %.3f, %.3f", cos_th, sin_th);
        return 0.0;
      }
    }
    double min_dist_front = obstacle->getMinimumDistance(front_polygon);
    double min_dist_rear = obstacle->getMinimumDistance(rear_polygon);
    double min_dist = std::min(min_dist_front, min_dist_rear);
    ROS_INFO_COND(debug_msg, "    MINDIST: %.3f", min_dist);
    return min_dist;
  }

  /**
    * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
    * @param current_pose robot pose, from which the distance to the obstacle is estimated
    * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
    * @param t time, for which the predicted distance to the obstacle is calculated
    * @return Euclidean distance to the robot
    */
  virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, const Obstacle* obstacle, double t) const
  {
    Point2dContainer front_polygon;
    Point2dContainer rear_polygon;
    transformToWorldFront(current_pose, front_polygon);
    transformToWorldRear(current_pose, rear_polygon);
    double min_dist_front = obstacle->getMinimumSpatioTemporalDistance(front_polygon, t);
    double min_dist_rear = obstacle->getMinimumSpatioTemporalDistance(rear_polygon, t);
    return std::min(min_dist_front, min_dist_rear);
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information (type, pose, scale and color).
    * The header, namespace, id and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] markers container of marker messages describing the robot shape
    * @param color Color of the footprint
    */
  virtual void visualizeRobot(const PoseSE2& current_pose, std::vector<visualization_msgs::Marker>& markers, const std_msgs::ColorRGBA& color) const
  {
    if (model_.ff_length>0  && model_.fr_length>0 && model_.f_width>0)
    {
      markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker1 = markers.back();
      marker1.type = visualization_msgs::Marker::LINE_STRIP;
      current_pose.toPoseMsg(marker1.pose);

      geometry_msgs::Point point;
      // Front-RL
      point.x = model_.fr_length;
      point.y = -model_.f_width/2;
      point.z = 0;
      marker1.points.push_back(point);
      // Front-RR
      point.x = model_.fr_length;
      point.y = model_.f_width/2;
      point.z = 0;
      marker1.points.push_back(point);
      // Front-FR
      point.x = model_.ff_length;
      point.y = model_.f_width/2;
      point.z = 0;
      marker1.points.push_back(point);
      // Front-FL
      point.x = model_.ff_length;
      point.y = -model_.f_width/2;
      point.z = 0;
      marker1.points.push_back(point);
      // add first point again in order to close the polygon
      point.x = model_.fr_length;
      point.y = -model_.f_width/2;
      point.z = 0;
      marker1.points.push_back(point);

      marker1.scale.x = marker_line_width; 
      marker1.color = color;

    }

    if (model_.rr_length>0  && model_.rf_length>0 && model_.r_width>0)
    {
      markers.push_back(visualization_msgs::Marker());
      visualization_msgs::Marker& marker2 = markers.back();
      marker2.type = visualization_msgs::Marker::LINE_STRIP;
      current_pose.toSteeringPoseMsg(marker2.pose);

      geometry_msgs::Point point;
      // Front-RL
      point.x = -model_.rr_length;
      point.y = -model_.r_width/2;
      point.z = 0;
      marker2.points.push_back(point);
      // Front-RR
      point.x = -model_.rr_length;
      point.y = model_.r_width/2;
      point.z = 0;
      marker2.points.push_back(point);
      // Front-FR
      point.x = -model_.rf_length;
      point.y = model_.r_width/2;
      point.z = 0;
      marker2.points.push_back(point);
      // Front-FL
      point.x = -model_.rf_length;
      point.y = -model_.r_width/2;
      point.z = 0;
      marker2.points.push_back(point);
      // add first point again in order to close the polygon
      point.x = -model_.rr_length;
      point.y = -model_.r_width/2;
      point.z = 0;
      marker2.points.push_back(point);

      marker2.scale.x = marker_line_width; 
      marker2.color = color;
    }
  }
  
  /**
   * @brief Compute the inscribed radius of the footprint model
   * @return inscribed radius
   */
  virtual double getInscribedRadius() 
  {
    return std::min(model_.f_width, model_.r_width);
  }

private:
    
  /**
    * @brief Transforms a polygon to the world frame manually
    * @param current_pose Current robot pose
    * @param[out] polygon_world polygon in the world frame
    */
  void transformToWorldFront(const PoseSE2& current_pose, Point2dContainer& polygon_world) const
  {
    double cos_th = std::cos(current_pose.theta());
    double sin_th = std::sin(current_pose.theta());

    polygon_world.resize(4);
    // Front - FR
    polygon_world[0].x() = current_pose.x() + cos_th * model_.ff_length - sin_th * model_.f_width/2;
    polygon_world[0].y() = current_pose.y() + sin_th * model_.ff_length + cos_th * model_.f_width/2;
    // Front - FL
    polygon_world[1].x() = current_pose.x() + cos_th * model_.ff_length - sin_th * (-model_.f_width/2);
    polygon_world[1].y() = current_pose.y() + sin_th * model_.ff_length + cos_th * (-model_.f_width/2);
    // Front - RL
    polygon_world[2].x() = current_pose.x() + cos_th * model_.fr_length - sin_th * (-model_.f_width/2);
    polygon_world[2].y() = current_pose.y() + sin_th * model_.fr_length + cos_th * (-model_.f_width/2);
    // Front - RR
    polygon_world[3].x() = current_pose.x() + cos_th * model_.fr_length - sin_th * model_.f_width/2;
    polygon_world[3].y() = current_pose.y() + sin_th * model_.fr_length + cos_th * model_.f_width/2;
  } 
  
  /**
    * @brief Transforms a polygon to the world frame manually
    * @param current_pose Current robot pose
    * @param[out] polygon_world polygon in the world frame
    */
  void transformToWorldRear(const PoseSE2& current_pose, Point2dContainer& polygon_world) const
  {
    double cos_th = std::cos(current_pose.theta()-current_pose.steering_pos());
    double sin_th = std::sin(current_pose.theta()-current_pose.steering_pos());

    polygon_world.resize(4);
    // Rear - FR
    polygon_world[0].x() = current_pose.x() + cos_th * (-model_.rf_length) - sin_th * model_.r_width/2;
    polygon_world[0].y() = current_pose.y() + sin_th * (-model_.rf_length) + cos_th * model_.r_width/2;
    // Rear - FL
    polygon_world[1].x() = current_pose.x() + cos_th * (-model_.rf_length) - sin_th * (-model_.r_width/2);
    polygon_world[1].y() = current_pose.y() + sin_th * (-model_.rf_length) + cos_th * (-model_.r_width/2);
    // Rear - RL
    polygon_world[2].x() = current_pose.x() + cos_th * (-model_.rr_length) - sin_th * (-model_.r_width/2);
    polygon_world[2].y() = current_pose.y() + sin_th * (-model_.rr_length) + cos_th * (-model_.r_width/2);
    // Rear - RR
    polygon_world[3].x() = current_pose.x() + cos_th * (-model_.rr_length) - sin_th * model_.r_width/2;
    polygon_world[3].y() = current_pose.y() + sin_th * (-model_.rr_length) + cos_th * model_.r_width/2;
  }

  Point2dContainer vertices_;
  CenterArticulatedModel model_;
  const double marker_line_width = 0.2;
  
};




} // namespace teb_local_planner

#endif /* ROBOT_FOOTPRINT_MODEL_H */
