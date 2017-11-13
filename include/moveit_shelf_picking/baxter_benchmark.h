/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
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
 *   * Neither the name of PickNik LLC nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Demo dual arm manipulation
*/

#ifndef MOVEIT_SHELF_PICKING_BAXTER_BENCHMARK_H
#define MOVEIT_SHELF_PICKING_BAXTER_BENCHMARK_H

// ROS
#include <ros/ros.h>

#include <bolt_moveit/moveit_base.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/imarker_robot_state.h>
#include <moveit_boilerplate/planning_interface.h>

namespace moveit_shelf_picking
{
static const std::size_t NUM_VISUALS = 6;

class BaxterBenchmark : public bolt_moveit::MoveItBase
{
public:
  /** \brief Constructor */
  BaxterBenchmark(const std::string& package_path);

  /** \brief Destructor */
  ~BaxterBenchmark();

  /** \brief Clear previous planner stuff */
  void reset(std::size_t indent);

  void eachPlanner(std::size_t indent);

  void run(std::size_t indent);

  bool runProblems(std::size_t indent);

  bool plan(std::size_t run_id, std::size_t indent);

  bool getRandomState(moveit::core::RobotStatePtr& robot_state);

  /**
   * \brief Clear all markers displayed in Rviz
   */
  void deleteAllMarkers(std::size_t indent);

  void loadVisualTools(std::size_t indent);

  void loadScene(std::size_t indent);

  void loadOfficeScene(std::size_t indent);

  void loadAmazonScene(std::size_t indent);

  void loadBin(double y, std::size_t indent);

  bool chooseStartGoal(std::size_t run_id, std::size_t indent);

  bool setSingleStartFromIMarker(std::size_t indent);
  bool setSingleGoalFromIMarker(std::size_t indent);

  // --------------------------------------------------------

  // A shared node handle
  ros::NodeHandle nh_;

  // The short name of this class
  std::string name_ = "moveit_shelf_picking";

  // File location of this package
  std::string package_path_;

  // The visual tools for interfacing with Rviz
  std::vector<moveit_visual_tools::MoveItVisualToolsPtr> visual_tools_;

  // Robot states
  moveit::core::RobotStatePtr moveit_start_;
  moveit::core::RobotStatePtr moveit_goal_;

  // Planning groups
  std::string planning_group_name_;
  moveit::core::JointModelGroup* planning_jmg_;

  // End effector data including joint model groups
  std::vector<mvt::ArmData> arm_datas_;


  bool verbose_ = false;
  // Operation settings
  std::size_t num_problems_;
  int problem_type_;
  bool headless_;
  bool auto_run_;

  double velocity_scaling_factor_ = 0.2;

  // Verbosity levels
  bool debug_print_trajectory_;

  // Display preferences
  bool visualize_display_database_;
  bool visualize_interpolated_traj_;
  bool visualize_grid_generation_;
  bool visualize_start_goal_states_;
  bool visualize_cart_neighbors_;
  bool visualize_cart_path_;
  bool visualize_wait_between_plans_ = false;
  double visualize_time_between_plans_;
  bool visualize_database_every_plan_;

  // Used for non-Bolt planning
  double last_plan_path_length_;

  // Interactive markers
  moveit_visual_tools::IMarkerRobotStatePtr imarker_start_;
  moveit_visual_tools::IMarkerRobotStatePtr imarker_goal_;

  // Load states from file
  std::string imarker_start_list_name_;
  std::string imarker_goal_list_name_;
  std::vector<moveit::core::RobotStatePtr> imarker_start_states_;
  std::vector<moveit::core::RobotStatePtr> imarker_goal_states_;
  std::size_t start_state_id_;
  std::size_t goal_state_id_;  // essentially the task

  // Scene vars
  std::size_t scene_type_;
  const double baxter_torso_height_ = -0.95;
  double distance_to_shelf_ = 0;
  bool use_shelf_noise_ = true;
  double bin_height_ = 0;
  std::size_t num_rows_ = 5;
  double penetration_start_ = -0.05;
  Eigen::Affine3d common_transform_;
  double penetration_dist_;

};  // end class

// Create boost pointers for this class
typedef std::shared_ptr<BaxterBenchmark> BaxterBenchmarkPtr;
typedef std::shared_ptr<const BaxterBenchmark> BaxterBenchmarkConstPtr;

}  // namespace moveit_shelf_picking

#endif  // MOVEIT_SHELF_PICKING_BAXTER_BENCHMARK_H
