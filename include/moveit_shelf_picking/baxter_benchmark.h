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

// OMPL
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/ParallelSetup.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

// bolt_core
#include <bolt_core/Bolt.h>

// bolt_moveit
#include <bolt_moveit/moveit_viz_window.h>
#include <bolt_moveit/model_based_state_space.h>
#include <bolt_moveit/moveit_base.h>
#include <bolt_moveit/state_validity_checker.h>
#include <bolt_moveit/cart_path_planner.h>

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

  /** \brief Load the basic planning context components */
  bool loadOMPL(std::size_t indent);

  /** \brief Generate states for testing */
  void testConnectionToGraphOfRandStates(std::size_t indent);

  /** \brief Must lock the planning scene that is passed in for thread safety */
  void loadCollisionChecker(const planning_scene::PlanningSceneConstPtr& planning_scene, std::size_t indent);

  bool loadData(std::size_t indent);

  void eachPlanner(std::size_t indent);

  void run(std::size_t indent);

  bool runProblems(std::size_t indent);

  bool plan(std::size_t run_id, std::size_t indent);

  /** \brief Create multiple dummy cartesian paths */
  bool generateCartGraph(std::size_t indent);

  bool checkOMPLPathSolution(og::PathGeometric& path);
  bool checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj, std::size_t indent);

  bool getRandomState(moveit::core::RobotStatePtr& robot_state);

  /**
   * \brief Clear all markers displayed in Rviz
   */
  void deleteAllMarkers(std::size_t indent);

  void loadVisualTools(std::size_t indent);
  void loadOMPLVisualTools(std::size_t indent);

  void visualizeRawTrajectory(og::PathGeometric& path, std::size_t indent);

  void displayWaitingState(bool waiting);

  void testMotionValidator(std::size_t indent);

  void mirrorGraph(std::size_t indent);

  ob::State* combineStates(const ob::State* state1, const ob::State* state2);

  void benchmarkMemoryAllocation(std::size_t indent);

  void loadScene(std::size_t indent);

  void loadOfficeScene(std::size_t indent);

  void loadAmazonScene(std::size_t indent);

  void loadBin(double y, std::size_t indent);

  void saveIMarkersToFile(std::size_t indent);

  void viewIMarkersFromFile(std::size_t indent);

  void loadIMarkersFromFile(std::vector<moveit::core::RobotStatePtr>& robot_states, std::string file_name,
                            std::size_t indent);

  void loadIMarkers(std::size_t indent);

  robot_trajectory::RobotTrajectoryPtr processSimpleSolution(std::size_t indent);

  robot_trajectory::RobotTrajectoryPtr processSegments(std::size_t indent);

  bool chooseStartGoal(std::size_t run_id, std::size_t indent);

  bool setSingleStartFromIMarker(std::size_t indent);
  bool setSingleGoalFromIMarker(std::size_t indent);

  void displayDisjointSets(std::size_t indent);

  std::string getPlannerFilePath(const std::string& planning_group_name, std::size_t indent);

  void doPostProcessing(std::size_t indent);

  void loadSPARS2Data(std::size_t indent);

  void log(bool solved, std::size_t indent);

  void processAndExecute(robot_trajectory::RobotTrajectoryPtr execution_traj, std::size_t indent);

  void generateMultiGoalsFromIK(std::size_t run_id, std::size_t indent);

  void visualizeRawSolutionLine(std::size_t indent);
  void visualizeSmoothSolutionLine(std::size_t indent);

  // --------------------------------------------------------

  // A shared node handle
  ros::NodeHandle nh_;

  // The short name of this class
  std::string name_ = "moveit_shelf_picking";

  // File location of this package
  std::string package_path_;

  // Save the experience setup until the program ends so that the planner data is not lost
  ompl::geometric::SimpleSetupPtr simple_setup_;
  ompl::geometric::ParallelSetupPtr parallel_setup_;
  ompl::tools::bolt::BoltPtr bolt_;
  ompl::tools::ThunderPtr thunder_;
  ompl::tools::LightningPtr lightning_;
  ompl::geometric::SPARStwoPtr spars2_;

  // Configuration space
  bolt_moveit::ModelBasedStateSpacePtr space_;
  ompl::base::SpaceInformationPtr si_;
  //ob::SpaceInformationPtr secondary_si_;  // used for second thread

  // Mirroring config
  moveit::core::JointModelGroup* both_arms_jmg_;
  moveit::core::JointModelGroup* left_arm_jmg_;
  bolt_moveit::ModelBasedStateSpacePtr both_arms_state_space_;
  bolt_moveit::ModelBasedStateSpacePtr left_arm_state_space_;
  moveit::core::RobotStatePtr mirror_state_;

  // The visual tools for interfacing with Rviz
  std::vector<moveit_visual_tools::MoveItVisualToolsPtr> visual_tools_;
  ompl::tools::VisualizerPtr visual_;

  // TODO: remove these?
  // moveit_visual_tools::MoveItVisualToolsPtr visual_moveit_start_;  // Clone of ompl1
  // moveit_visual_tools::MoveItVisualToolsPtr visual_moveit_goal_;   // Clone of ompl2

  // Robot states
  moveit::core::RobotStatePtr moveit_start_;
  moveit::core::RobotStatePtr moveit_goal_;
  ob::State* ompl_start_;
  ob::State* ompl_goal_;

  // Planning groups
  std::string planning_group_name_;
  moveit::core::JointModelGroup* planning_jmg_;

  // End effector data including joint model groups
  std::vector<mvt::ArmData> arm_datas_;

  // Execute trajectories
  moveit_boilerplate::ExecutionInterfacePtr execution_interface_;

  // Interpolate and parameterize trajectories
  moveit_boilerplate::PlanningInterfacePtr planning_interface_;

  // Logging
  std::ofstream logging_file_;

  bool verbose_ = false;

  // Modes
  bool run_problems_;
  bool create_spars_;
  bool load_spars_;
  bool continue_spars_;
  bool eliminate_dense_disjoint_sets_;
  bool check_valid_vertices_;
  bool display_database_;
  bool display_disjoint_sets_;
  bool benchmark_performance_;
  bool save_imarkers_to_file_;
  bool view_imarkers_from_file_;
  bool post_processing_;
  int post_processing_interval_;
  std::string load_database_version_;

  // Type of planner
  std::vector<std::string> planners_;
  std::vector<std::string> experiment_names_;
  double planning_time_;  // seconds
  std::string planner_;   // the current planner being run
  std::string experiment_name_; // paper name of current planner experiment
  bool is_bolt_ = false;
  bool is_thunder_ = false;
  bool is_lightning_ = false;
  bool is_simple_setup_ = false;

  // Mirroring
  bool mirror_graph_;
  std::string opposite_arm_name_;
  std::string both_arms_group_name_;

  // Fill in dimension
  // bool fill_in_dim_;
  // std::string full_arm_name_;

  // Operation settings
  std::size_t num_problems_;
  int problem_type_;
  bool use_task_planning_;
  bool headless_;
  bool auto_run_;
  bool track_memory_consumption_ = false;
  bool use_logging_ = false;
  std::string log_file_name_ = "moveit_shelf_picking_logging.csv";
  bool collision_checking_enabled_ = true;

  double velocity_scaling_factor_ = 0.2;
  bool connect_to_hardware_ = false;

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

  // Average planning time
  // double total_duration_ = 0;
  // std::size_t total_runs_ = 0;
  // std::size_t total_failures_ = 0;

  // Create constrained paths
  bolt_moveit::CartPathPlannerPtr cart_path_planner_;

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

  // Validity checker
  bolt_moveit::StateValidityCheckerPtr validity_checker_;

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

  double test_var_;

};  // end class

// Create boost pointers for this class
typedef std::shared_ptr<BaxterBenchmark> BaxterBenchmarkPtr;
typedef std::shared_ptr<const BaxterBenchmark> BaxterBenchmarkConstPtr;

}  // namespace moveit_shelf_picking

#endif  // MOVEIT_SHELF_PICKING_BAXTER_BENCHMARK_H
