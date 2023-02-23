/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
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
 *   * Neither the name of the Jiri Horner nor the names of its
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
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <actionlib/client/simple_action_client.h>
#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <traceback_msgs/GoalAndImage.h>
#include <traceback_msgs/ImageAndImage.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore
{
public:
  Explore();
  ~Explore();

  void start();
  void stop();

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  /**
   * @brief  Publish a frontiers as markers
   */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& frontier_goal);

  void reachedTracebackGoal(const actionlib::SimpleClientGoalState& status,
                            const move_base_msgs::MoveBaseResultConstPtr& result,
                            const move_base_msgs::MoveBaseGoal& traceback_goal);

  bool goalOnBlacklist(const geometry_msgs::Point& goal);

  // e.g. return "/tb3_0"
  std::string getRobotName()
  {
    return ros::this_node::getNamespace();
  }

  ros::Timer traceback_timeout_timer_;
  ros::Timer traceback_oneshot_;
  void doTraceback(move_base_msgs::MoveBaseGoal goal, int abort_timeout);

  void sendResultToTraceback(bool aborted);

  void tracebackGoalAndImageUpdate(
      const traceback_msgs::GoalAndImage::ConstPtr& msg);

  void CameraImageUpdate(const sensor_msgs::ImageConstPtr& msg);
  void CameraDepthImageUpdate(const sensor_msgs::ImageConstPtr& msg);
  // void CameraPointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr& msg);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;
  size_t last_markers_count_;

  // In order to synchronize image and point cloud although they don't really do
  sensor_msgs::Image temp_image_;
  ros::Subscriber robot_camera_image_subscriber_;
  std::string robot_camera_image_topic_ = "camera/rgb/image_raw";
  sensor_msgs::Image current_image_;
  ros::Subscriber robot_camera_depth_image_subscriber_;
  std::string robot_camera_depth_image_topic_ = "camera/depth/image_raw";
  sensor_msgs::Image current_depth_image_;
  ros::Subscriber robot_camera_point_cloud_subscriber_;
  std::string robot_camera_point_cloud_topic_ = "camera/depth/points";
  // sensor_msgs::PointCloud2 current_point_cloud_;

  ros::Subscriber traceback_goal_and_image_subscriber_;
  std::string traceback_goal_and_image_topic_ = "traceback/goal_and_image";
  ros::Publisher traceback_image_and_image_publisher_;
  std::string traceback_image_and_image_topic_ = "traceback/image_and_image";

  bool in_traceback_ = false;
  ros::Timer resume_timer_;
  void resumeNormalExplorationLater(int32_t second);
  void cancelResumeNormalExplorationLater();

  move_base_msgs::MoveBaseGoal current_traceback_goal_;
  sensor_msgs::Image current_traced_robot_image_;
  sensor_msgs::Image current_traced_robot_depth_image_;
  // sensor_msgs::PointCloud2 current_traced_robot_point_cloud_;
  std::string current_tracer_robot_;
  std::string current_traced_robot_;
  double current_src_map_origin_x_;
  double current_src_map_origin_y_;
  double current_dst_map_origin_x_;
  double current_dst_map_origin_y_;
  bool current_second_traceback_;
  int64_t current_traced_robot_stamp_;

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  ros::Duration progress_timeout_;
  bool visualize_;
};
}  // namespace explore

#endif
