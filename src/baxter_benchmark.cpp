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

// ROS parameter helpers
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// this package
#include <moveit_shelf_picking/baxter_benchmark.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

// C++
#include <algorithm>  // for lower case

namespace rvt = rviz_visual_tools;
namespace ps = planning_scene;
namespace psm = planning_scene_monitor;

namespace moveit_shelf_picking
{
BaxterBenchmark::BaxterBenchmark(const std::string &package_path)
  : MoveItBase(), nh_("~"), package_path_(package_path)
{
  std::size_t indent = 0;

  std::vector<std::string> ee_tip_links;
  std::vector<std::string> arm_jmgs;

  bool seed_random;
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  // run mode
  error += !rosparam_shortcuts::get(name_, rpnh, "run_problems", run_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "create_spars", create_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "load_spars", load_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "load_database_version", load_database_version_);
  error += !rosparam_shortcuts::get(name_, rpnh, "continue_spars", continue_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "eliminate_dense_disjoint_sets", eliminate_dense_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "check_valid_vertices", check_valid_vertices_);
  error += !rosparam_shortcuts::get(name_, rpnh, "display_database", display_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "display_disjoint_sets", display_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "benchmark_performance", benchmark_performance_);
  error += !rosparam_shortcuts::get(name_, rpnh, "save_imarkers_to_file", save_imarkers_to_file_);
  error += !rosparam_shortcuts::get(name_, rpnh, "view_imarkers_from_file", view_imarkers_from_file_);

  // mirror
  error += !rosparam_shortcuts::get(name_, rpnh, "mirror_graph", mirror_graph_);
  error += !rosparam_shortcuts::get(name_, rpnh, "opposite_arm_name", opposite_arm_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "both_arms_group_name", both_arms_group_name_);

  // fill in last dimension
  // error += !rosparam_shortcuts::get(name_, rpnh, "fill_in_dim", fill_in_dim_);
  // error += !rosparam_shortcuts::get(name_, rpnh, "full_arm_name", full_arm_name_);

  // run type
  error += !rosparam_shortcuts::get(name_, rpnh, "auto_run", auto_run_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planners", planners_);
  error += !rosparam_shortcuts::get(name_, rpnh, "experiment_names", experiment_names_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_time", planning_time_);
  error += !rosparam_shortcuts::get(name_, rpnh, "num_problems", num_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "headless", headless_);
  error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_goal_list", imarker_goal_list_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_start_list", imarker_start_list_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_task_planning", use_task_planning_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "arm_jmgs", arm_jmgs);
  error += !rosparam_shortcuts::get(name_, rpnh, "ee_tip_links", ee_tip_links);
  error += !rosparam_shortcuts::get(name_, rpnh, "seed_random", seed_random);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing", post_processing_);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing_interval", post_processing_interval_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_logging", use_logging_);
  error += !rosparam_shortcuts::get(name_, rpnh, "log_file_name", log_file_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "collision_checking_enabled", collision_checking_enabled_);
  // collision
  error += !rosparam_shortcuts::get(name_, rpnh, "scene_type", scene_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "distance_to_shelf", distance_to_shelf_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_shelf_noise", use_shelf_noise_);
  error += !rosparam_shortcuts::get(name_, rpnh, "bin_height", bin_height_);
  error += !rosparam_shortcuts::get(name_, rpnh, "num_rows", num_rows_);
  error += !rosparam_shortcuts::get(name_, rpnh, "penetration_start", penetration_start_);
  error += !rosparam_shortcuts::get(name_, rpnh, "test_var", test_var_);
  // execution
  error += !rosparam_shortcuts::get(name_, rpnh, "connect_to_hardware", connect_to_hardware_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_scaling_factor", velocity_scaling_factor_);
  // Visualize
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/display_database", visualize_display_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/interpolated_traj", visualize_interpolated_traj_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/time_between_plans", visualize_time_between_plans_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/database_every_plan", visualize_database_every_plan_);
  // Debug
  error += !rosparam_shortcuts::get(name_, rpnh, "verbose/print_trajectory", debug_print_trajectory_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Check Input
  if (planners_.size() != experiment_names_.size())
  {
    BOLT_ERROR("Mismatching number of items in planners vs experiment_names param");
    exit(-1);
  }

  // Estimate the worse-case runtime
  double maxRunTime = planning_time_ * planners_.size() * num_problems_;
  BOLT_INFO(true, "Worse case run time: " << maxRunTime << " sec (" << maxRunTime / 60.0 / 60.0 << " hours)");
  std::cout << std::endl;

  // Seed random
  if (seed_random)
    srand(time(NULL));

  // Initialize MoveIt base
  MoveItBase::init(nh_);

  // Set custom joint weights
  robot_model_->getJointModel("left_s1")->setDistanceFactor(1.0);
  robot_model_->getJointModel("left_e0")->setDistanceFactor(0.9);
  robot_model_->getJointModel("left_e1")->setDistanceFactor(0.8);
  robot_model_->getJointModel("left_w0")->setDistanceFactor(0.7);
  robot_model_->getJointModel("left_w1")->setDistanceFactor(0.6);
  robot_model_->getJointModel("left_w2")->setDistanceFactor(0.0);

  robot_model_->getJointModel("right_s1")->setDistanceFactor(1.0);
  robot_model_->getJointModel("right_e0")->setDistanceFactor(0.9);
  robot_model_->getJointModel("right_e1")->setDistanceFactor(0.8);
  robot_model_->getJointModel("right_w0")->setDistanceFactor(0.7);
  robot_model_->getJointModel("right_w1")->setDistanceFactor(0.6);
  robot_model_->getJointModel("right_w2")->setDistanceFactor(0.0);

  // Load more robot states
  moveit_start_ = std::make_shared<moveit::core::RobotState>(*current_state_);
  moveit_goal_ = std::make_shared<moveit::core::RobotState>(*current_state_);

  // Get the two arms jmg
  planning_jmg_ = robot_model_->getJointModelGroup(planning_group_name_);

  if (arm_jmgs.size() != ee_tip_links.size())
  {
    BOLT_ERROR("Joint model groups array must match size of EEF tip links array");
    exit(-1);
  }

  for (std::size_t i = 0; i < arm_jmgs.size(); ++i)
  {
    arm_datas_.push_back(
        mvt::ArmData(robot_model_->getJointModelGroup(arm_jmgs[i]), robot_model_->getLinkModel(ee_tip_links[i])));
    if (!arm_datas_.back().jmg_)
    {
      BOLT_ERROR("No joint model group found for jmg name " << arm_jmgs[i]);
      exit(-1);
    }
    if (!arm_datas_.back().ee_link_)
    {
      BOLT_ERROR("No link model found for link name " << ee_tip_links[i]);
      exit(-1);
    }
  }

  // Load more visual tool objects
  loadVisualTools(indent);

  // Add a collision objects
  loadScene(indent);

  // Create start/goal state imarker
  if (!headless_)
    loadIMarkers(indent);

  planning_interface_ = std::make_shared<moveit_boilerplate::PlanningInterface>(psm_, visual_tools_[6], planning_jmg_,
                                                                                execution_interface_);
}

BaxterBenchmark::~BaxterBenchmark()
{
}

void BaxterBenchmark::loadVisualTools(std::size_t indent)
{
  // Load the ROS part, but not the OMPL part until loadOMPL has occured
  using namespace moveit_visual_tools;

  std::string namesp = nh_.getNamespace();

  // NOTE that all visuals start at index 1, not 0
  visual_tools_.resize(NUM_VISUALS + 1);
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    const std::string base_frame = "/world_visual" + std::to_string(i);
    const std::string marker_topic = namesp + "/ompl_visual" + std::to_string(i);
    MoveItVisualToolsPtr moveit_visual = std::make_shared<MoveItVisualTools>(base_frame, marker_topic, psm_);
    moveit_visual->loadMarkerPub(false);
    moveit_visual->setManualSceneUpdating(true);
    moveit_visual->setGlobalScale(0.8);
    moveit_visual->enableBatchPublishing();
    moveit_visual->loadRemoteControl();
    visual_tools_[i] = moveit_visual;
  }

  ros::spinOnce();
  moveit_start_->setToDefaultValues();

  // Secondary loop to give time for all the publishers to load up
  Eigen::Affine3d offset;
  if (!headless_)
  {
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      // Get TF
      getTFTransform("world", "world_visual" + std::to_string(i), offset);
      visual_tools_[i]->enableRobotStateRootOffet(offset);
    }
  }

  visual_tools_[6]->setBaseFrame("world");

  ros::spinOnce();

  // Block until all visualizers are finished loading
  if (!headless_)
  {
    const double wait_time = 0.2;
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      visual_tools_[i]->waitForMarkerPub(wait_time);

      // Load publishers
      bool blocking = false;
      visual_tools_[i]->loadRobotStatePub(namesp + "/robot_state" + std::to_string(i), blocking);

      // Show the initial robot state
      usleep(0.001 * 1000000);
      visual_tools_[i]->publishRobotState(moveit_start_);

      // Load trajectory publisher - ONLY for viz6
      if (i == 6)
        visual_tools_[i]->loadTrajectoryPub("/baxter/display_trajectory", blocking);
    }
  }

  deleteAllMarkers(indent);
}

void BaxterBenchmark::run(std::size_t indent)
{
  // Create list of goal states
  if (save_imarkers_to_file_)
    saveIMarkersToFile(indent);

  // View list of goal states
  if (view_imarkers_from_file_)
    viewIMarkersFromFile(indent);

  runProblems(indent);
}

bool BaxterBenchmark::runProblems(std::size_t indent)
{
  const double high_pd = 0.15 + std::numeric_limits<double>::epsilon();
  const double step_pd = 0.05;
  for (penetration_dist_ = penetration_start_; penetration_dist_ <= high_pd; penetration_dist_ += step_pd)
  {
    if (!ros::ok())  // Check if user wants to shutdown
      break;

    std::cout << std::endl;
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ " << std::endl;
    BOLT_INFO(true, "Penetration distance: " << penetration_dist_);
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ " << std::endl;

    // Run the demo the desired number of times
    for (std::size_t run_id = 0; run_id < num_problems_; ++run_id)
    {
      if (!ros::ok())  // Check if user wants to shutdown
        break;

      std::cout << std::endl;
      std::cout << "***************************************************************" << std::endl;
      BOLT_INFO(true, "Planning " << run_id + 1 << " out of " << num_problems_
                                  << " for penetration distance: " << penetration_dist_);
      std::cout << "***************************************************************" << std::endl;

      // Track memory usage
      double vm1, rss1;
      if (track_memory_consumption_)
        processMemUsage(vm1, rss1);

      // Populate TaskGraph, even for non-task planning
      if (is_bolt_)
      {
        if (use_task_planning_)
        {
          if (!generateCartGraph(indent))
          {
            BOLT_ERROR("Unable to create cart path");
            exit(-1);
          }
        }
        else
        {
          bolt_->getTaskGraph()->generateMonoLevelTaskSpace(indent);
        }
      }

      // Track memory usage
      if (track_memory_consumption_)
      {
        double vm2, rss2;
        processMemUsage(vm2, rss2);
        BOLT_INFO(true, "RAM usage diff (VM, RSS) MB:\n" << vm2 - vm1 << ", " << rss2 - rss1);
      }

      // -----------------------------------------------------
      // -----------------------------------------------------
      bool solved = plan(run_id, indent);
      // -----------------------------------------------------
      // -----------------------------------------------------

      // Post Proccess
      if (post_processing_ && run_id % post_processing_interval_ == 0)  // every x runs
      {
        BOLT_INFO(true, "Performing post processing every " << post_processing_interval_ << " plans");
        doPostProcessing(indent);
      }

      // Logging
      if (use_logging_)
        log(solved, indent);

      // Prompt user
      if (visualize_wait_between_plans_ && run_id < num_problems_ - 1)
        visual_->viz1()->prompt("run next problem");
      else  // Main pause between planning instances - allows user to analyze
        ros::Duration(visualize_time_between_plans_).sleep();
      if (!ros::ok())  // Check if user wants to shutdown
        break;

      // Reset marker if this is not our last run
      if (run_id < num_problems_ - 1)
        deleteAllMarkers(indent);

      // Move the shelf with noise
      if (use_shelf_noise_)
      {
        loadAmazonScene(indent);
        visual_tools_[6]->triggerPlanningSceneUpdate();
      }
    }  // for each run

  }  // for each penetration distance

  // Add last experience to database if any remaining
  if (post_processing_)
    doPostProcessing(indent);

  // Save to file
  if (is_bolt_)
    bolt_->saveIfChanged(indent);
  else if (is_thunder_)
    thunder_->saveIfChanged();
  else if (is_lightning_)
    lightning_->saveIfChanged();

  // Extra debug info
  if (is_bolt_)
  {
    std::cout << "numInterfacesSkippedByLength: " << bolt_->getSparseCriteria()->getNumInterfacesSkippedByLength()
              << std::endl;
  }

  return true;
}

bool BaxterBenchmark::plan(std::size_t run_id, std::size_t indent)
{
  BOLT_FUNC(true, "plan()");

  // Setup -----------------------------------------------------------

  // Reset value
  last_plan_path_length_ = 0.0;

  // Clear all planning data. This only includes data generated by motion plan computation.
  // Planner settings, start & goal states are not affected.
  if (is_bolt_)
    bolt_->clearForNextPlan(indent);
  else
    simple_setup_->clear();

  // Generate start/goal pairs
  while (!chooseStartGoal(run_id, indent))
  {
    BOLT_WARN(true, "Invalid start/goal found, trying again");

    // Move the shelf
    loadAmazonScene(indent);
    visual_tools_[6]->triggerPlanningSceneUpdate();
  }

  // Lock the planning scene for read-only while a plan is solved - no outside node should be allowed to modify
  // This is achieved by updating the version of the planning_scene inside StateValidityChecker
  psm::LockedPlanningSceneRO lscene(psm_);
  const ps::PlanningSceneConstPtr &scene = static_cast<const ps::PlanningSceneConstPtr &>(lscene);
  validity_checker_->setPlanningScene(scene);

  // Solve -----------------------------------------------------------

  // Create the termination condition
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(planning_time_, 0.1);

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus status = simple_setup_->solve(ptc);

  bool solved = false;
  if (status == ob::PlannerStatus::EXACT_SOLUTION)
    solved = true;

  if (is_bolt_)
  {
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);
    bolt_->processResults(indent);
    bolt_->printLogs();
  }

  if (!solved)
  {
    BOLT_ERROR("Failed to find solution: " << status);
    return false;
  }
  // Visualize pre-smoothing
  if (visualize_interpolated_traj_)
    visualizeRawSolutionLine(indent);

  // Simplify
  if (visualize_interpolated_traj_ || connect_to_hardware_ || post_processing_)
    simple_setup_->simplifySolution();

  // Visualize post-smoothing
  if (visualize_interpolated_traj_)
    visualizeSmoothSolutionLine(indent);

  robot_trajectory::RobotTrajectoryPtr execution_traj;

  // Interpolate and parameterize
  // if (is_bolt_)
  // {
  //   execution_traj = processSegments(indent);
  // }
  // else  // RRTConnect, etc
  // {
  execution_traj = processSimpleSolution(indent);
  //}

  // Interpolate, parameterize, and execute/visualize
  if (visualize_interpolated_traj_ || connect_to_hardware_)
    processAndExecute(execution_traj, indent);

  return true;
}

void BaxterBenchmark::deleteAllMarkers(std::size_t indent)
{
  // Reset rviz markers
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    visual_tools_[i]->deleteAllMarkers();
    visual_tools_[i]->trigger();
  }
}

bool BaxterBenchmark::getRandomState(moveit::core::RobotStatePtr &robot_state)
{
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);

  std::size_t indent = 0;
  static const std::size_t MAX_ATTEMPTS = 1000;
  for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
  {
    robot_state->setToRandomPositions(planning_jmg_);
    robot_state->update();

    // Error check
    bool check_verbose = false;
    if (planning_scene->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                        // collision check, "" is everything
    {
      return true;
    }

    if (i == 100)
      BOLT_WARN(true, "Taking long time to find valid random state");
  }

  BOLT_ERROR("Unable to find valid random robot state");
  exit(-1);
  return false;
}

void BaxterBenchmark::loadScene(std::size_t indent)
{
  switch (scene_type_)
  {
    case 1:
      loadAmazonScene(indent);
      loadBin(0.7, indent);
      loadBin(-0.7, indent);
    // no break on purpose
    case 0:
      loadOfficeScene(indent);
      break;
  }

  visual_tools_[6]->triggerPlanningSceneUpdate();  // send to Rviz

  // Append to allowed collision matrix
  {
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm_);  // Read/write lock
    collision_detection::AllowedCollisionMatrix &collision_matrix = planning_scene->getAllowedCollisionMatrixNonConst();
    collision_matrix.setEntry("wall", "pedestal", true);
  }
}

void BaxterBenchmark::loadOfficeScene(std::size_t indent)
{
  BOLT_FUNC(true, "loadOfficeScene()");
  // psm_->updateFrameTransforms();

  // const double table_height = -0.77 * baxter_torso_height_;
  const double table_height = -0.75 * baxter_torso_height_;
  visual_tools_[6]->publishCollisionFloor(baxter_torso_height_ + 0.001, "floor", rvt::TRANSLUCENT_DARK);
  visual_tools_[6]->publishCollisionWall(/*x*/ -1.0, /*y*/ 0.0, /*z*/ baxter_torso_height_, /*angle*/ 0,
                                         /*width*/ 2, /*height*/ 2.0, "wall1", rvt::TRANSLUCENT_LIGHT);
  visual_tools_[6]->publishCollisionWall(/*x*/ 0.0, /*y*/ -1.075, /*z*/ baxter_torso_height_, /*angle*/ M_PI / 2.0,
                                         /*width*/ 2, /*height*/ 2.0, "wall2", rvt::TRANSLUCENT_LIGHT);
  visual_tools_[6]->publishCollisionWall(/*x*/ 0.0, /*y*/ 1.075, /*z*/ baxter_torso_height_, /*angle*/ M_PI / 2.0,
                                         /*width*/ 2, /*height*/ 2.0, "wall3", rvt::TRANSLUCENT_LIGHT);
  visual_tools_[6]->publishCollisionTable(/*x*/ 0.85, /*y*/ 0.0, /*z*/ baxter_torso_height_, /*angle*/ 0,
                                          /*width*/ 2.0, /*height*/ table_height, /*depth*/ 0.8, "table",
                                          rvt::DARK_GREY);
}

void BaxterBenchmark::loadAmazonScene(std::size_t indent)
{
  BOLT_FUNC(false, "loadAmazonScene()");

  // Add noise
  double noise = 0.03;  // 3 cm, slighlty more than an inch
  double rnoise_degrees = 5;
  double rnoise = rnoise_degrees * 0.0174533;  // radians
  double x_noise = 0;
  double y_noise = 0;
  double rotation_noise = 0;
  if (use_shelf_noise_)
  {
    x_noise = visual_tools_[6]->dRand(-noise, noise);
    y_noise = visual_tools_[6]->dRand(-noise, noise);
    rotation_noise = visual_tools_[6]->dRand(-rnoise, rnoise);  // z axis

    BOLT_INFO(true, "loadAmazon() x_noise = " << x_noise << "; y_noise = " << y_noise
                                              << "; rotation_noise = " << rotation_noise << ";");
  }

  visual_tools_[6]->deleteAllMarkers();

  // Calculate offset
  common_transform_ = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(rotation_noise, Eigen::Vector3d::UnitZ());
  common_transform_.translation().x() = distance_to_shelf_ + x_noise;
  common_transform_.translation().y() = y_noise;

  // visual_tools_[6]->publishArrow(common_transform_, rvt::ORANGE, rvt::XLARGE);

  Eigen::Affine3d mesh_centroid;
  mesh_centroid = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  mesh_centroid.translation().z() = baxter_torso_height_;
  mesh_centroid = common_transform_ * mesh_centroid;

  // const std::string object_name = "shelf" + std::to_string(visual_tools_[6]->iRand(0,10000)); // Unique name
  const std::string object_name = "shelf";

  static bool haveAdded = false;
  if (!haveAdded)
  {
    // Load mesh file name
    const std::string collision_mesh_path = "file://" + package_path_ + "/meshes/pod_lowres.stl";
    visual_tools_[6]->publishCollisionMesh(mesh_centroid, object_name, collision_mesh_path, rvt::BROWN);
    haveAdded = true;
  }
  else
  {
    // Just move the mesh
    visual_tools_[6]->moveCollisionObject(mesh_centroid, object_name, rvt::BROWN);
  }
}

void BaxterBenchmark::loadBin(double y, std::size_t indent)
{
  // Load mesh file name
  const std::string collision_mesh_path = "file://" + package_path_ + "/meshes/goal_bin.stl";

  BOLT_FUNC(true, "loadBin() " << collision_mesh_path);

  Eigen::Affine3d mesh_centroid = Eigen::Affine3d::Identity();

  // Calculate offset
  mesh_centroid = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  mesh_centroid.translation().x() = distance_to_shelf_ * 0.65;
  mesh_centroid.translation().y() = y;
  mesh_centroid.translation().z() = bin_height_;

  const std::string object_name = "bin" + std::to_string(y);

  // shapes::Shape *mesh = shapes::createMeshFromResource(collision_mesh_path);  // make sure its prepended by file://
  // shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  // if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  // {
  //   BOLT_ERROR("Unable to create mesh shape message from resource " << collision_mesh_path);
  //   return;
  // }

  // shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Add mesh to scene
  visual_tools_[6]->publishCollisionMesh(mesh_centroid, object_name, collision_mesh_path, rvt::RED);
}

void BaxterBenchmark::saveIMarkersToFile(std::size_t indent)
{
  std::string file_path;
  bolt_moveit::getFilePath(file_path, imarker_goal_list_name_, "ros/ompl_storage");
  BOLT_INFO(true, "Saving goal states to " << file_path);

  std::ofstream output_file;
  output_file.open(file_path, std::ofstream::out | std::ofstream::app);  // append

  while (ros::ok())
  {
    visual_->viz1()->prompt("save state to file...");

    const bool verbose = true;
    if (!imarker_goal_->isStateValid(verbose))
    {
      BOLT_WARN(true, "IMarker goal state is in collision, try again");
      continue;
    }

    moveit::core::robotStateToStream(*imarker_goal_->getRobotState(), output_file, false);
  }

  output_file.close();
}

void BaxterBenchmark::viewIMarkersFromFile(std::size_t indent)
{
  BOLT_FUNC(true, "viewIMarkersFromFile()");
  std::vector<moveit::core::RobotStatePtr> robot_states;

  loadIMarkersFromFile(robot_states, imarker_goal_list_name_, indent);

  std::size_t count = 0;
  for (moveit::core::RobotStatePtr state : robot_states)
  {
    BOLT_INFO(true, "Viewing state " << count++);
    visual_tools_[6]->publishRobotState(state);
    ros::Duration(1.0).sleep();
  }

  BOLT_INFO(true, "Finished, shutting down");
  exit(0);
}

void BaxterBenchmark::loadIMarkersFromFile(std::vector<moveit::core::RobotStatePtr> &robot_states, std::string file_name,
                                      std::size_t indent)
{
  BOLT_FUNC(true, "loadIMarkersFromFile()");

  std::string file_path;
  bolt_moveit::getFilePath(file_path, file_name, "ros/ompl_storage");
  BOLT_INFO(true, "Loading goal states from " << file_path);

  // Error check
  if (!boost::filesystem::exists(file_path))
  {
    BOLT_WARN(true, "File not found: " << file_path);
    return;
  }

  std::ifstream input_file(file_path);

  std::string line;
  std::size_t count = 0;
  while (std::getline(input_file, line))
  {
    moveit::core::RobotStatePtr temp = std::make_shared<moveit::core::RobotState>(*current_state_);
    moveit::core::streamToRobotState(*temp, line);
    robot_states.push_back(temp);
  }

  input_file.close();
}

void BaxterBenchmark::loadIMarkers(std::size_t indent)
{
  // Create cartesian planner
  if (use_task_planning_)
  {
    cart_path_planner_ = std::make_shared<bolt_moveit::CartPathPlanner>(arm_datas_, visual_tools_[6], moveit_start_,
                                                                        psm_, package_path_, space_, planning_jmg_);
  }

  // Create start/goal imarkers
  imarker_start_ = std::make_shared<mvt::IMarkerRobotState>(psm_, "start", arm_datas_, rvt::GREEN, package_path_);
  imarker_goal_ = std::make_shared<mvt::IMarkerRobotState>(psm_, "goal", arm_datas_, rvt::ORANGE, package_path_);

  // Error message until current state is valid
  if (connect_to_hardware_)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    bool check_verbose = true;

    planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);
    while (!planning_scene->isStateValid(*getCurrentState(), "", check_verbose) && ros::ok())
    {
      BOLT_ERROR("START STATE INVALID " << ros::Time::now());
      visual_tools_[5]->publishRobotState(getCurrentState(), rvt::RED);
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

bool BaxterBenchmark::chooseStartGoal(std::size_t run_id, std::size_t indent)
{
  BOLT_FUNC(verbose_, "chooseStartGoal() problem_type: " << problem_type_);

  switch (problem_type_)
  {
    // Random ---------------------------------------------
    case 0:
      static const double CLEARANCE = 0; //0.015; // 1.5 cm
      imarker_start_->setToRandomState(CLEARANCE);
      imarker_goal_->setToRandomState(CLEARANCE);

      // Apply imarker start/goal to OMPL SimpleSetup
      if (!setSingleStartFromIMarker(indent))
        return false;
      if (!setSingleGoalFromIMarker(indent))
        return false;

      break;
    // User Sets from IMarkers ------------------------------------------
    case 1:  // imarkers
      // do nothing - already set

      // Apply imarker start/goal to OMPL SimpleSetup
      if (!setSingleStartFromIMarker(indent))
        return false;
      if (!setSingleGoalFromIMarker(indent))
        return false;

      break;
    // imarker goal list ------------------------------------------------
    case 2:
    {
      // choose start
      if (imarker_start_states_.empty())
        loadIMarkersFromFile(imarker_start_states_, imarker_start_list_name_, indent);

      start_state_id_ = run_id % imarker_start_states_.size();
      BOLT_INFO(true, "chooseStartGoal: total start states: " << imarker_start_states_.size() << " run_id: " << run_id
                                                              << " start_state_id: " << start_state_id_);

      imarker_start_->setRobotState(imarker_start_states_[start_state_id_]);

      // choose goal
      if (imarker_goal_states_.empty())
        loadIMarkersFromFile(imarker_goal_states_, imarker_goal_list_name_, indent);

      goal_state_id_ = run_id % imarker_goal_states_.size();
      BOLT_INFO(true, "chooseStartGoal: total goal states: " << imarker_goal_states_.size() << " run_id: " << run_id
                                                             << " goal_state_id: " << goal_state_id_);

      imarker_goal_->setRobotState(imarker_goal_states_[goal_state_id_]);

      // Apply imarker start/goal to OMPL SimpleSetup
      if (!setSingleStartFromIMarker(indent))
        return false;
      if (!setSingleGoalFromIMarker(indent))
        return false;
    }
    break;
    // from SRDF ------------------------------------------------
    case 3:
      BOLT_WARN(true, "NOT IMPLEMENTED");
      return false;
      // start is always the same
      // imarker_start_->getRobotState()->setToDefaultValues(planning_jmg_, "both_ready");
      // imarker_start_->publishRobotState();

      // Apply imarker start/goal to OMPL SimpleSetup
      if (!setSingleStartFromIMarker(indent))
        return false;
      if (!setSingleGoalFromIMarker(indent))
        return false;

      break;
    // from IK ---------------------------------------------
    case 4:
      // choose start
      if (imarker_start_states_.empty())
        loadIMarkersFromFile(imarker_start_states_, imarker_start_list_name_, indent);

      start_state_id_ = run_id % imarker_start_states_.size();
      BOLT_INFO(true, "chooseStartGoal: total start states: " << imarker_start_states_.size() << " run_id: " << run_id
                                                              << " start_state_id: " << start_state_id_);
      imarker_start_->setRobotState(imarker_start_states_[start_state_id_]);

      // Apply imarker start to OMPL SimpleSetup, **but not goal**
      if (!setSingleStartFromIMarker(indent))
        return false;

      // choose goal and set in OMPL SimpleSetup
      generateMultiGoalsFromIK(run_id, indent);
      break;
    default:
      BOLT_ERROR("Unknown problem type");
      return false;
  }

  return true;
}

bool BaxterBenchmark::setSingleStartFromIMarker(std::size_t indent)
{
  // if running on hardware, override start state with current state
  if (connect_to_hardware_)
  {
    imarker_start_->setToCurrentState();
  }

  // Validate with collision checking
  static const bool VERBOSE = true;
  if (!imarker_start_->isStateValid(VERBOSE))
  {
    BOLT_WARN(true, "Invalid start state");
    return false;
  }

  // Convert MoveIt imarker state to OMPL state
  space_->copyToOMPLState(ompl_start_, *imarker_start_->getRobotState());

  // Set the start state in OMPL
  simple_setup_->clearStartStates();
  simple_setup_->addStartState(ompl_start_);

  // Visualize
  if (visualize_start_goal_states_)
    imarker_start_->publishRobotState();

  return true;
}

bool BaxterBenchmark::setSingleGoalFromIMarker(std::size_t indent)
{
  // Validate with collision checking
  static const bool VERBOSE = true;
  if (!imarker_goal_->isStateValid(VERBOSE))
  {
    BOLT_WARN(true, "Invalid goal state");
    return false;
  }

  // Convert MoveIt imarker state to OMPL state
  space_->copyToOMPLState(ompl_goal_, *imarker_goal_->getRobotState());

  // // TODO: delete (temporary)
  // imarker_goal_->publishRobotState();

  // planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);  // Read only lock
  // if (!planning_scene->isStateValid(*imarker_goal_->getRobotState(), "", true))
  // {
  //   BOLT_WARN(true, "Invalid goal state MoveIt! 1");
  //   exit(-1);
  // }

  // if (!planning_scene->isStateValid(*imarker_goal_->getRobotState(), planning_group_name_, true))
  // {
  //   BOLT_WARN(true, "Invalid goal state MoveIt! 2");
  //   exit(-1);
  // }

  // collision_detection::CollisionRequest collision_request_simple_verbose;
  // collision_request_simple_verbose.group_name = planning_group_name_;
  // collision_request_simple_verbose.verbose = true;
  // collision_detection::CollisionResult res;
  // psm_->getPlanningScene()->checkCollision(collision_request_simple_verbose, res, *imarker_goal_->getRobotState());
  // if (res.collision)
  // {
  //   BOLT_WARN(true, "Invalid goal state OMPL 1");
  //   exit(-1);
  // }

  // if (!validity_checker_->isValid(ompl_goal_))
  // {
  //   BOLT_WARN(true, "Invalid goal state OMPL 2");
  //   exit(-1);
  // }

  // Set the goal state in OMPL
  simple_setup_->setGoalState(ompl_goal_);

  // Visualize
  if (visualize_start_goal_states_)
    imarker_goal_->publishRobotState();

  return true;
}

std::string BaxterBenchmark::getPlannerFilePath(const std::string &planning_group_name, std::size_t indent)
{
  // Set the database file location

  std::string planner_lower_ = planner_;
  std::transform(planner_lower_.begin(), planner_lower_.end(), planner_lower_.begin(), ::tolower);

  std::string file_name;
  if (is_bolt_)
  {
    file_name = planner_lower_ + "_" + planning_group_name + "_" +
                std::to_string(bolt_->getSparseCriteria()->sparseDeltaFraction_) + "_" + load_database_version_;
  }
  else if (is_thunder_ || is_lightning_)
  {
    file_name = planner_lower_ + "_" + planning_group_name;
  }

  std::string file_path;
  bolt_moveit::getFilePath(file_path, file_name, "ros/ompl_storage");
  return file_path;
}

void BaxterBenchmark::log(bool solved, std::size_t indent)
{
  BOLT_FUNC(true, "log()");

  std::size_t numVerticesAdded = 0;
  std::size_t numEdgesAdded = 0;
  std::size_t numVertices = 0;
  std::size_t numEdges = 0;
  bool inserted = false;
  double insertionTime = 0;
  if (is_bolt_)
  {
    // Get a copy of the stats and clear them from the Bolt setup
    otb::ExperiencePathStats stats = bolt_->getPostProcessingResultsAndReset();
    numVerticesAdded = stats.numVerticesAdded_;
    numEdgesAdded = stats.numEdgesAdded_;
    inserted = stats.inserted_;
    insertionTime = stats.insertionTime_;

    // General graph stats
    numVertices = bolt_->getSparseGraph()->getNumVertices();
    numEdges = bolt_->getSparseGraph()->getNumEdges();
  }
  if (is_thunder_)
  {
    // Get a copy of the stats and clear them from the Thunder setup
    numVerticesAdded = thunder_->diffNumVertices_;
    numEdgesAdded = thunder_->diffNumEdges_;
    thunder_->diffNumVertices_ = 0;
    thunder_->diffNumEdges_ = 0;

    // General graph stats
    numVertices = thunder_->getExperienceDB()->getSPARSdb()->getNumVertices();
    numEdges = thunder_->getExperienceDB()->getSPARSdb()->getNumEdges();
  }
  if (is_lightning_)
  {
    numVertices = lightning_->getStatesCount();
    // count each path as one edge, but there are no real edges
    numEdges = lightning_->getExperiencesCount();
  }

  double pathLength = 99999;  // some really large number to prevent forgetting to adjust for this in averages
  if (simple_setup_->getProblemDefinition()->hasSolution())
  {
    og::PathGeometric &path =
        static_cast<og::PathGeometric &>(*simple_setup_->getProblemDefinition()->getSolutionPath());
    pathLength = path.length();
  }
  else
    BOLT_WARN(true, "logging: does not have solution");

  logging_file_ << experiment_name_ << ", "                                     // bolt, etc
                << simple_setup_->getSolutionPlannerName() << ", "      // which thread finished first
                << penetration_dist_ << ", "                            // task
                << simple_setup_->getLastPlanComputationTime() << ", "  // planning time
                << pathLength << ", "                                   // basic planning stats
                << simple_setup_->getLastSimplificationTime() << ", "   // smoothing time
                << numVerticesAdded << ", "                             // numVerticesAdded
                << numEdgesAdded << ", "                                // numEdgesAdded
                << numVertices << ", "                                  // total vertex count
                << numEdges << ", "                                     // total edge count
                << inserted << ", "       // whether experience failed to insert (0) or succeeded (1)
                << insertionTime << ", "  // how long it took to insert
                << solved                 // solved
                << std::endl;
  logging_file_.flush();
}

}  // namespace moveit_shelf_picking
