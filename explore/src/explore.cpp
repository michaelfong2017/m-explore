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

#include <explore/explore.h>
#include <nav_msgs/GetPlan.h>

#include <thread>

#include "geometry_msgs/Pose.h"
#include "mongodb_store/message_store.h"

using namespace geometry_msgs;
using namespace mongodb_store;
using namespace std;

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });

  // Note that private_nh_ is for example under /tb3_0/explore, but I want
  // /tb3_0/traceback/goal_and_image instead.
  traceback_goal_and_image_subscriber_ =
      private_nh_.subscribe<traceback_msgs::GoalAndImage>(
          ros::names::append(getRobotName(), traceback_goal_and_image_topic_),
          10, [this](const traceback_msgs::GoalAndImage::ConstPtr& msg) {
            tracebackGoalAndImageUpdate(msg);
          });

  // Note that private_nh_ is for example under /tb3_0/explore, but I want
  // /tb3_0/traceback/image_and_image instead.
  traceback_image_and_image_publisher_ =
      private_nh_.advertise<traceback_msgs::ImageAndImage>(
          ros::names::append(getRobotName(), traceback_image_and_image_topic_),
          10);

  robot_camera_image_subscriber_ = private_nh_.subscribe<sensor_msgs::Image>(
      ros::names::append(getRobotName(), robot_camera_image_topic_), 50,
      [this](const sensor_msgs::ImageConstPtr& msg) {
        CameraImageUpdate(msg);
      });

  robot_camera_depth_image_subscriber_ = private_nh_.subscribe<sensor_msgs::Image>(
      ros::names::append(getRobotName(), robot_camera_depth_image_topic_), 50,
      [this](const sensor_msgs::ImageConstPtr& msg) {
        CameraDepthImageUpdate(msg);
      });

  // robot_camera_point_cloud_subscriber_ =
  //     private_nh_.subscribe<sensor_msgs::PointCloud2>(
  //         ros::names::append(getRobotName(), robot_camera_point_cloud_topic_),
  //         50, [this](const sensor_msgs::PointCloud2ConstPtr& msg) {
  //           CameraPointCloudUpdate(msg);
  //         });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  // Called every 3 seconds for each robot
  ROS_DEBUG("\n\nmakePlan\n%d\n", ros::Time::now().sec);
  // return;

  if (in_traceback_) {
    return;
  }

  // find frontiers
  auto pose = costmap_client_.getRobotPose();

  // ros::NodeHandle nh;
  // MessageStoreProxy messageStore(nh);

  // Pose p;
  // p.position.z = 666;
  // string name("my pose");
  // ROS_DEBUG("Name: %s, z: %f", name, p.position.z);
  // string id(messageStore.insertNamed(name, p));
  // messageStore.updateID(id, p);
  // assert(messageStore.queryID<Pose>(id).first->position.z == 666);

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });

  geometry_msgs::Quaternion target_orientation;
  target_orientation.x = 0.0;
  target_orientation.y = 0.0;
  target_orientation.z = 0.0;
  target_orientation.w = 1.0;

  geometry_msgs::Point target_position;
  double min_distance;
  if (frontier != frontiers.end()) {
    target_position = frontier->centroid;
    min_distance = frontier->min_distance;
  } else {
    // stop();
    // return;

    // In order to test in bigger maps, instead of stopping exploration,
    // command the robot to randomly move in a direction.
    geometry_msgs::Point random_point;
    int r = rand() % 4;
    switch (r) {
      case 0:
        random_point.x = pose.position.x + 5.0;
        random_point.y = pose.position.y + 0.0;
        break;
      case 1:
        random_point.x = pose.position.x + -5.0;
        random_point.y = pose.position.y + 0.0;
        break;
      case 2:
        random_point.x = pose.position.x + 0.0;
        random_point.y = pose.position.y + 5.0;
        break;
      case 3:
        random_point.x = pose.position.x + 0.0;
        random_point.y = pose.position.x + -5.0;
        break;
    }
    random_point.z = 0.0;
    target_position = random_point;
    min_distance = 5.0;
  }

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    return;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal) {
    return;
  }

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation = target_orientation;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 2;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::doTraceback(move_base_msgs::MoveBaseGoal goal, int abort_timeout)
{
  ROS_DEBUG("\n\ndoTraceback\n%d\n", ros::Time::now().sec);

  geometry_msgs::Point target_position = goal.target_pose.pose.position;

  // if (goalOnBlacklist(target_position)) {
  //   ROS_DEBUG("traceback goal is on blacklist");
  //   move_base_client_.cancelGoal();
  //   // Then, process here

  //   // May cause duplication
  //   frontier_blacklist_.push_back(target_position);
  //   ROS_DEBUG("Adding current traceback goal to black list");
  //   // Wait 1 second in order to collect a correct image at the goal
  //   traceback_oneshot_ = relative_nh_.createTimer(
  //       ros::Duration(1, 0),
  //       [this](const ros::TimerEvent&) { sendResultToTraceback(true); }, true);

  //   // Wait for some time for Traceback to process (continue traceback),
  //   // If no message is sent after 3 seconds, resume to normal exploration
  //   resumeNormalExplorationLater(3);
  // }

  // auto pose = costmap_client_.getRobotPose();

  // double min_distance =
  //     sqrt(pow((double(pose.position.x) - double(target_position.x)), 2.0) +
  //          pow((double(pose.position.y) - double(target_position.y)), 2.0));

  // Cancel the goal after some seconds, which is the timeout
  // Also blacklist the goal
  int traceback_timeout = abort_timeout;
  traceback_timeout_timer_ = relative_nh_.createTimer(
      ros::Duration(traceback_timeout, 0),
      [this, target_position, traceback_timeout](const ros::TimerEvent&) {
        ROS_DEBUG("Cancelling traceback goal after %d seconds",
                  traceback_timeout);
        move_base_client_.cancelGoal();
        // Then, process here

        // May cause duplication
        // frontier_blacklist_.push_back(target_position);
        // ROS_DEBUG("Adding current traceback goal to black list");
        // Wait 1 second in order to collect a correct image at the goal
        traceback_oneshot_ = relative_nh_.createTimer(
            ros::Duration(1, 0),
            [this](const ros::TimerEvent&) { sendResultToTraceback(true); },
            true);

        // Wait for some time for Traceback to process (continue traceback),
        // If no message is sent after 3 seconds, resume to normal exploration
        resumeNormalExplorationLater(3);
      },
      true);

  // send goal to move_base if we have something new to pursue
  move_base_client_.sendGoal(
      goal, [this, goal](const actionlib::SimpleClientGoalState& status,
                         const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedTracebackGoal(status, result, goal);
      });
}

void Explore::sendResultToTraceback(bool aborted)
{
  traceback_msgs::ImageAndImage images;
  images.aborted = aborted;
  images.traced_image = current_traced_robot_image_;
  images.tracer_image = current_image_;
  images.traced_depth_image = current_traced_robot_depth_image_;
  images.tracer_depth_image = current_depth_image_;
  images.tracer_robot = current_tracer_robot_;
  images.traced_robot = current_traced_robot_;
  images.src_map_origin_x = current_src_map_origin_x_;
  images.src_map_origin_y = current_src_map_origin_y_;
  images.dst_map_origin_x = current_dst_map_origin_x_;
  images.dst_map_origin_y = current_dst_map_origin_y_;
  images.arrived_pose = costmap_client_.getRobotPose();

  images.stamp = current_traced_robot_stamp_;
  traceback_image_and_image_publisher_.publish(images);
}

void Explore::tracebackGoalAndImageUpdate(
    const traceback_msgs::GoalAndImage::ConstPtr& msg)
{
  ROS_INFO("tracebackGoalAndImageUpdate");
  cancelResumeNormalExplorationLater();

  // TODO currently, in_traceback_ is set back to false only 30 seconds after
  // sendGoal callback received
  if (!in_traceback_) {
    in_traceback_ = true;
  }

  current_traceback_goal_ = msg->goal;
  current_traced_robot_image_ = msg->image;
  current_traced_robot_depth_image_ = msg->depth_image;
  current_tracer_robot_ = msg->tracer_robot;
  current_traced_robot_ = msg->traced_robot;
  current_src_map_origin_x_ = msg->src_map_origin_x;
  current_src_map_origin_y_ = msg->src_map_origin_y;
  current_dst_map_origin_x_ = msg->dst_map_origin_x;
  current_dst_map_origin_y_ = msg->dst_map_origin_y;
  current_traced_robot_stamp_ = msg->stamp;

  doTraceback(current_traceback_goal_, msg->abort_timeout);
}

void Explore::CameraImageUpdate(const sensor_msgs::ImageConstPtr& msg)
{
  temp_image_ = *msg;
}

void Explore::CameraDepthImageUpdate(const sensor_msgs::ImageConstPtr& msg)
{
  current_depth_image_ = *msg;
  current_image_ = temp_image_;
}

// void Explore::CameraPointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//   current_point_cloud_ = *msg;
//   current_image_ = temp_image_;
// }

void Explore::resumeNormalExplorationLater(int32_t second)
{
  resume_timer_ = relative_nh_.createTimer(
      ros::Duration(second, 0),
      [this](const ros::TimerEvent&) { in_traceback_ = false; }, true);
}

void Explore::cancelResumeNormalExplorationLater()
{
  resume_timer_.stop();
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

// Terminal States
// Rejected - The goal was rejected by the action server without being processed
// and without a request from the action client to cancel Succeeded - The goal
// was achieved successfully by the action server Aborted - The goal was
// terminated by the action server without an external request from the action
// client to cancel Recalled - The goal was canceled by either another goal, or
// a cancel request, before the action server began processing the goal
// Preempted - Processing of the goal was canceled by either another goal, or a
// cancel request sent to the action server
void Explore::reachedTracebackGoal(
    const actionlib::SimpleClientGoalState& status,
    const move_base_msgs::MoveBaseResultConstPtr& result,
    const move_base_msgs::MoveBaseGoal& traceback_goal)
{
  ROS_DEBUG("Traceback reached goal with status: %s",
            status.toString().c_str());

  if (status == actionlib::SimpleClientGoalState::SUCCEEDED) {
    traceback_timeout_timer_.stop();

    // Wait 1 second in order to collect a correct image at the goal
    traceback_oneshot_ = relative_nh_.createTimer(
        ros::Duration(1, 0),
        [this](const ros::TimerEvent&) { sendResultToTraceback(false); }, true);

    // Wait for some time for Traceback to process (continue traceback),
    // If no message is sent after 3 seconds, resume to normal exploration
    resumeNormalExplorationLater(3);
  }
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
