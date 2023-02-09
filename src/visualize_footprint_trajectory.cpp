/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daniel Cardenas
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <tf2/utils.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
TebVisualizationPtr visual;
TebConfig config;
std::vector<teb_local_planner::TrajectoryPointMsg> last_trajectory;
ros::Subscriber feedback_sub;
ros::Publisher footprint_marker_pub;
RobotFootprintModelPtr robot_model;
unsigned int counter = 0;

void CB_feddback(const teb_local_planner::FeedbackMsgConstPtr& feedback);
void CB_mainCycle(const ros::TimerEvent& e);


// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualize_footprint_trajectory");
  ros::NodeHandle n("/move_base_node/TebLocalPlannerROS");
  
  config.loadRosParamFromNodeHandle(n);
  counter = 0;
 
  // Subscription
  feedback_sub = n.subscribe("/move_base_node/TebLocalPlannerROS/teb_feedback", 1, CB_feddback);
  footprint_marker_pub = n.advertise<visualization_msgs::Marker>("fotprint_animation", 1000);

  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Timers
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.3), CB_mainCycle);

  ros::spin();

  return 0;
}

// Feedback CB
void CB_feddback(const teb_local_planner::FeedbackMsgConstPtr& feedback)
{
  last_trajectory = feedback->trajectories[0].trajectory;
  feedback_sub.shutdown();
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  std::vector<visualization_msgs::Marker> markers;

  if(last_trajectory.empty())
  {
    return;
  }

  ROS_INFO("i = %u. SP = %.3f, H = %.3f, omega=%.3f,v=%.3f", 
    counter, last_trajectory[counter].steering_pos,
    tf2::getYaw(last_trajectory[counter].pose.orientation),
    last_trajectory[counter].velocity.angular.z,
    last_trajectory[counter].velocity.linear.x);

  std_msgs::ColorRGBA color_red = visual->toColorMsg(0.9, 1.0, 0, 0);
  PoseSE2 pose(last_trajectory[counter].pose);
  pose.addSteeringPose(last_trajectory[counter].steering_pos);
  robot_model->visualizeRobot(pose, markers, color_red);

  int idx = 1000000;  // avoid overshadowing by obstacles
  for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  {
    marker_it->header.frame_id = "map";
    marker_it->header.stamp = ros::Time::now();
    marker_it->action = visualization_msgs::Marker::ADD;
    marker_it->ns = "RobotFootprintModel";
    marker_it->id = idx;
    marker_it->lifetime = ros::Duration();
    footprint_marker_pub.publish(*marker_it);
  }
  counter = (counter+1)%last_trajectory.size();
}