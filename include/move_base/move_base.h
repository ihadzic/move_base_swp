/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAV_MOVE_BASE_ACTION_H_
#define NAV_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_swp/MoveBaseSWPAction.h>

#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

#include <pluginlib/class_loader.hpp>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include "move_base_swp/MoveBaseConfig.h"

namespace move_base {
  //typedefs to help us out with the action server and client so that we
  // don't have to type so much
  typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
  typedef actionlib::SimpleActionServer<move_base_swp::MoveBaseSWPAction> MoveBaseSWPActionServer;
  typedef actionlib::SimpleActionClient<move_base_swp::MoveBaseSWPAction> MoveBaseSWPActionClient;

  enum MoveBaseState {
    PLANNING,
    CONTROLLING,
    CLEARING
  };

  enum RecoveryTrigger
  {
    PLANNING_R,
    CONTROLLING_R,
    OSCILLATION_R
  };

  /**
   * @class MoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */
  class MoveBase {
    public:
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
      MoveBase(tf2_ros::Buffer& tf);

      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~MoveBase();

    private:
      /**
       * @brief  Performs a control cycle
       * @return True if processing of the goal is done, false otherwise
       */
      bool executeCycle();

      /**
       * @brief  A service call that clears the costmaps of obstacles
       * @param req The service request
       * @param resp The service response
       * @return True if the service call succeeds, false otherwise
       */
      bool clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

      /**
       * @brief  A service call that can be made when the action is inactive that will return a plan
       * @param  req The goal request
       * @param  resp The plan request
       * @return True if planning succeeded, false otherwise
       */
      bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Load the recovery behaviors for the navigation stack from the parameter server
       * @param node The ros::NodeHandle to be used for loading parameters
       * @return True if the recovery behaviors were loaded successfully, false otherwise
       */
      bool loadRecoveryBehaviors(ros::NodeHandle node);

      /**
       * @brief  Loads the default recovery behaviors for the navigation stack
       */
      void loadDefaultRecoveryBehaviors();

      /**
       * @brief  Clears obstacles within a window around the robot
       * @param size_x The x size of the window
       * @param size_y The y size of the window
       */
      void clearCostmapWindows(double size_x, double size_y);

      /**
       * @brief  Brakes the robot
       */
      void applyBrakes();

      /**
       * @brief  Publishes and records the robot velocity on cmd_vel
       * @param cmd_vel Requested velocity vector
       **/
      void setVelocity(const geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Ramps velocity down while keeping the same turn radius
       * @param vx Reference to linear velocity (x component)
       * @param vy Reference to linear velocity (y component)
       * @param vy Reference to angular velocity (y component)
       * @param delta Step by which to reduce the velocity in this step
       * @return True if the service call succeeds, false otherwise
       */
      bool rampDownVelocity(double& vx, double& vy, double& omegaz, double delta);

      /**
       * @brief  Reset the state of the move_base action and send a zero velocity command to the base
       */
      void resetState();

      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

      void planThread();
      void brakeThread();

      void logPose(const char *msg, const geometry_msgs::PoseStamped& p);
      void executeCb(const move_base_swp::MoveBaseSWPGoalConstPtr& swp_goal);
      void executeLegacyCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

      bool isQuaternionValid(const geometry_msgs::Quaternion& q);
      bool loadWaypoints(const move_base_swp::MoveBaseSWPGoalConstPtr& swp_goal, std::vector<geometry_msgs::PoseStamped>& waypoints);
      void publishWaypoints(const std::vector<geometry_msgs::PoseStamped>& waypoints);
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

      geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg);

      /**
       * @brief This is used to wake the planner at periodic intervals.
       */
      void wakePlanner(const ros::TimerEvent& event);

      /**
       * @brief  Give waypoints to the planner and start it up
       * @param event Reference to waypoints to load
       */
      void startPlanner(const std::vector<geometry_msgs::PoseStamped>& waypoints);
      /**
       * @brief  A startPlanner variant when waypoints are already loaded
       */
      void startPlanner();

      /**
       * @brief  Stops the planner
       */
      void stopPlanner();

      /**
       * @brief  Finds the closest waypoint on the plan
       * @param cwpi Reference to the current waypoint index
       * @param plan Reference to the plan
       * @return waypoint pose
       */
      geometry_msgs::PoseStamped updateClosestWaypointIndex(int& cwpi, const std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Checks if the near-term plan needs more points fed and advances it
       * @param cwpi Reference to the current waypoint index
       * @param full_plan Global plan
       * @param pwpi reference to the pursued waypoint
       * @param near_term_plan Reference to the constructed near-term plan
       * @return true if the plan was updated
       */
      bool updateNearTermPlan(int cwpi, const std::vector<geometry_msgs::PoseStamped>& full_plan, int& pwpi, std::vector<geometry_msgs::PoseStamped>& near_term_plan);

      /**
       * @brief  Drops waypoints that have been visited
       * @param cwpi Closest Waypoint index
       * @param waypoints Reference to the vector of waypoints
       * @param waypoint_indices Reference to the vector of waypoint indices
       */
      void pruneWaypoints(int cwpi, std::vector<geometry_msgs::PoseStamped>& waypoints, std::vector<int>& waypoint_indices);

      /**
       * @brief  Drops first waypoint in the plan
       * @param cwpi Closest Waypoint index
       * @param waypoints Reference to the vector of waypoints
       * @param waypoint_indices Reference to the vector of waypoint indices
       */
      void pruneFirstWaypoint(std::vector<geometry_msgs::PoseStamped>& waypoints, std::vector<int>& waypoint_indices);

      /**
       * @brief  Clear all plans
       */
      void clearPlans();

      tf2_ros::Buffer& tf_;

      MoveBaseSWPActionServer* as_;
      MoveBaseSWPActionClient* ac_;
      MoveBaseActionServer* as_legacy_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

      boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
      std::string robot_base_frame_, global_frame_;

      std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      std::vector<std::string> recovery_behavior_names_;
      unsigned int recovery_index_;

      geometry_msgs::PoseStamped global_pose_;
      double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      double planner_patience_, controller_patience_;
      int32_t max_planning_retries_;
      uint32_t planning_retries_;
      uint32_t replan_on_incomplete_counter_;
      double conservative_reset_dist_, clearing_radius_;
      double brake_slope_;
      double brake_sample_rate_;
      ros::Publisher current_goal_pub_, current_waypoints_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_, snapped_pose_pub_, pursued_plan_pub_;
      ros::Subscriber goal_sub_;
      ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      double oscillation_timeout_, oscillation_distance_;

      MoveBaseState state_;
      RecoveryTrigger recovery_trigger_;

      ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      geometry_msgs::PoseStamped oscillation_pose_;
      pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      std::vector<geometry_msgs::PoseStamped>* controller_plan_;
      std::vector<geometry_msgs::PoseStamped> near_term_plan_segment_;
      std::vector<int>* planner_waypoint_indices_;
      std::vector<int>* latest_waypoint_indices_;
      std::vector<int>* controller_waypoint_indices_;
      int closest_plan_waypoint_index_;
      int pursued_plan_waypoint_index_;
      int plan_buffer_size_;
      int plan_reload_threshold_;

      //set up the planner's thread
      bool runPlanner_;
      boost::recursive_mutex planner_mutex_;
      boost::condition_variable_any planner_cond_;
      std::vector<geometry_msgs::PoseStamped> planner_waypoints_;
      boost::thread* planner_thread_;

      // brake control
      boost::recursive_mutex brake_mutex_;
      boost::condition_variable_any brake_cond_;
      bool brake_;
      double current_vx_, current_vy_, current_omegaz_;
      boost::thread* brake_thread_;

      boost::recursive_mutex configuration_mutex_;
      dynamic_reconfigure::Server<move_base::MoveBaseConfig> *dsrv_;

      void reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level);

      move_base::MoveBaseConfig last_config_;
      move_base::MoveBaseConfig default_config_;
      bool setup_, p_freq_change_, c_freq_change_;
      bool new_global_plan_;
  };
};
#endif
