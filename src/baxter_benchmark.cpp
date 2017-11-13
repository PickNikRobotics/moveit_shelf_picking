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
#include <bolt_ros/ompl_rosparam.h>

// bolt_core
#include <bolt_core/SparseMirror.h>
#include <bolt_core/SparseCriteria.h>
#include <bolt_core/SparseFormula.h>
#include <bolt_core/BoltPlanner.h>

// bolt_moveit
#include <bolt_moveit/process_mem_usage.h>
#include <bolt_moveit/projection_viz_window.h>
#include <bolt_moveit/model_size_state_space.h>

// OMPL
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/base/goals/GoalStates.h>

// this package
#include <moveit_shelf_picking/moveit_shelf_picking.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

// Profiling
#include <valgrind/callgrind.h>

// C++
#include <algorithm>  // for lower case

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;
namespace ps = planning_scene;
namespace psm = planning_scene_monitor;

namespace moveit_shelf_picking
{
BoltBaxter::BoltBaxter(const std::string &hostname, const std::string &package_path)
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

  // Auto-set headless if not on developer PC, assume we are on server
  // if (hostname != "ros-monster")
  // {
  //   OMPL_WARN("Auto-setting to headless mode because hostname is %s", hostname.c_str());
  //   headless_ = true;
  // }
  if (headless_)
    OMPL_WARN("Running in headless mode");

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

  // State for copying one arm to another (mirroring)
  mirror_state_ = std::make_shared<moveit::core::RobotState>(*current_state_);
  // set default wrist position (and all other joints)
  mirror_state_->setToDefaultValues();

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

  // Connect to physical hardware
  if (connect_to_hardware_)
  {
    execution_interface_ = std::make_shared<moveit_boilerplate::ExecutionInterface>(psm_, visual_tools_[6]);
  }
  planning_interface_ = std::make_shared<moveit_boilerplate::PlanningInterface>(psm_, visual_tools_[6], planning_jmg_,
                                                                                execution_interface_);

  // Wait until user does something
  if (!auto_run_)
    visual_tools_[1]->prompt("run first problem");

  if (!post_processing_)
    BOLT_WARN(true, "Post processing disabled!");

  // Run application
  eachPlanner(indent);
}

BoltBaxter::~BoltBaxter()
{
}

void BoltBaxter::reset(std::size_t indent)
{
  is_bolt_ = false;
  is_thunder_ = false;
  is_lightning_ = false;
  is_simple_setup_ = false;

  // Free start and goal states
  if (space_)
  {
    space_->freeState(ompl_start_);
    space_->freeState(ompl_goal_);
  }

  // Clear classes
  bolt_.reset();
  thunder_.reset();
  lightning_.reset();
  spars2_.reset();
  simple_setup_.reset();
  parallel_setup_.reset();
  si_.reset();
  //secondary_si_.reset();
  space_.reset();
  visual_.reset();
}

void BoltBaxter::loadVisualTools(std::size_t indent)
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

bool BoltBaxter::loadOMPL(std::size_t indent)
{
  std::size_t visual_id = 6;  // use 6th for sampler, which is loaded inside ModelBasedStateSpace
  bolt_moveit::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, planning_jmg_, visual_tools_[visual_id]);

  // Construct the state space we are planning in
  space_ = bolt_moveit::chooseModelSizeStateSpace(mbss_spec);
  si_ = std::make_shared<ob::SpaceInformation>(space_);

  // Create SimpleSetup
  if (planner_.substr(0, 4) == "Bolt")
  {
    bolt_ = std::make_shared<otb::Bolt>(si_);
    simple_setup_ = bolt_;
    is_bolt_ = true;

    // The visual pointer was already created and populated throughout the Bolt framework
    visual_ = bolt_->getVisual();

    // Customize per type of Bolt
    if (planner_ == "BoltRandom")  // use sampling thread
    {
      bolt_->getBoltPlanner()->useSamplingThread_ = true;
    }
    else if (planner_ == "BoltRRTConnect")
    {
      bolt_->usePFSPlanner_ = true;
      bolt_->useERRTConnect_ = false;
    }
    else if (planner_ == "BoltERRTConnect")
    {
      bolt_->usePFSPlanner_ = true;
      bolt_->useERRTConnect_ = true;
    }
    else
      BOLT_ERROR("Unknown bolt type " << planner_);
  }
  else if (planner_ == "Thunder")
  {
    thunder_ = std::make_shared<ot::Thunder>(si_);
    simple_setup_ = thunder_;
    is_thunder_ = true;

    // This must be called before thunder_->setup() because that is when load() is called
    thunder_->setFilePath(getPlannerFilePath(planning_group_name_, indent) + ".ompl");

    // The visual pointer was already created and populated throughout the Thunder framework
    visual_ = thunder_->getVisual();
  }
  else if (planner_ == "Lightning")
  {
    lightning_ = std::make_shared<ot::Lightning>(si_);
    simple_setup_ = lightning_;
    is_lightning_ = true;

    // Set default planner so that a goal definition isn't necessary
    lightning_->setPlanner(std::make_shared<og::RRTConnect>(si_));

    // This must be called before thunder_->setup() because that is when load() is called
    lightning_->setFilePath(getPlannerFilePath(planning_group_name_, indent) + ".ompl");

    // The visual pointer was already created and populated throughout the Bolt framework
    visual_ = std::make_shared<ot::Visualizer>();
  }
  else  // Assume simple setup
  {
    // Note we run all algorithms in two threads
    simple_setup_ = std::make_shared<og::ParallelSetup>(si_);
    is_simple_setup_ = true;

    visual_ = std::make_shared<ot::Visualizer>();

    if (planner_ == "RRTConnect")
    {
      simple_setup_->setPlanner(std::make_shared<og::RRTConnect>(si_));
      simple_setup_->setPlanner(std::make_shared<og::RRTConnect>(si_));
    }
    else if (planner_ == "LazyRRT")
    {
      simple_setup_->setPlanner(std::make_shared<og::LazyRRT>(si_));
      simple_setup_->setPlanner(std::make_shared<og::LazyRRT>(si_));
    }
    else if (planner_ == "LazyPRM")
    {
      simple_setup_->setPlanner(std::make_shared<og::LazyPRM>(si_));
      simple_setup_->setPlanner(std::make_shared<og::LazyPRM>(si_));
    }
    else if (planner_ == "RRT")
    {
      simple_setup_->setPlanner(std::make_shared<og::RRT>(si_));
      simple_setup_->setPlanner(std::make_shared<og::RRT>(si_));
    }
    else if (planner_ == "PRM")
    {
      simple_setup_->setPlanner(std::make_shared<og::PRM>(si_));
      simple_setup_->setPlanner(std::make_shared<og::PRM>(si_));
    }
    else if (planner_ == "BiEST")
    {
      simple_setup_->setPlanner(std::make_shared<og::BiEST>(si_));
      simple_setup_->setPlanner(std::make_shared<og::BiEST>(si_));
    }
    else if (planner_ == "PDST")
    {
      simple_setup_->setPlanner(std::make_shared<og::PDST>(si_));
      simple_setup_->setPlanner(std::make_shared<og::PDST>(si_));
    }
    else if (planner_ == "SPARStwo")
    {
      spars2_ = std::make_shared<og::SPARStwo>(si_);
      simple_setup_->setPlanner(spars2_);
    }
    else
    {
      BOLT_ERROR("Unknown planner: " << planner_);
      exit(-1);
    }
  }
  // Run interface for loading rosparam settings into OMPL
  if (is_bolt_)
  {
    bolt_moveit::loadOMPLParameters(nh_, name_, bolt_);
  }

  // TODO(davetcoleman): Warning: this uses the raw planning_scene, which is generally bad
  loadCollisionChecker(psm_->getPlanningScene(), indent);

  // Add moveit_visual_tools to visual_ class now that OMPL is finished loading
  loadOMPLVisualTools(indent);

  // Setup base OMPL stuff. Do this before choosing filename so sparseDeltaFraction is ready
  BOLT_INFO(true, "Setting up SimpleSetup");
  simple_setup_->setup();
  assert(si_->isSetup());

  // Stuff that must be run after setup()
  if (is_bolt_)
  {
    bolt_->setFilePath(getPlannerFilePath(planning_group_name_, indent));
    //bolt_->getBoltPlanner()->setSecondarySI(secondary_si_);  // must be called after loadCollisionChecker()
  }
  else if (is_thunder_)
  {
    thunder_->getExperienceDB()->getSPARSdb()->setSparseDeltaFraction(0.1);  // TODO do not hardcode
    thunder_->getExperienceDB()->getSPARSdb()->setDenseDeltaFraction(0.01);  // TODO do not hardcode
    thunder_->getExperienceDB()->getSPARSdb()->setup();
  }
  else if (is_lightning_)
  {
    // nothing
  }

  // Create start and goal states
  ompl_start_ = space_->allocState();
  ompl_goal_ = space_->allocState();

  return true;
}

// Set the OMPL planner / SimpleSetup with proper visualizer
void BoltBaxter::loadOMPLVisualTools(std::size_t indent)
{
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    bolt_moveit::MoveItVizWindowPtr viz = std::make_shared<bolt_moveit::MoveItVizWindow>(visual_tools_[i], si_);
    viz->setJointModelGroup(planning_jmg_);
    for (std::size_t i = 0; i < arm_datas_.size(); ++i)
    {
      viz->setEEFLink(arm_datas_[i].ee_link_);
    }

    // Index the visualizers
    visual_->setVizWindow(i, viz);
  }
}

bool BoltBaxter::loadData(std::size_t indent)
{
  BOLT_FUNC(true, "loadData()");

  double vm1, rss1;
  if (track_memory_consumption_)  // Track memory usage
    processMemUsage(vm1, rss1);

  // Load database or generate new roadmap
  BOLT_INFO(true, "Loading or generating roadmap");
  if (is_bolt_)
  {
    if (!bolt_->load(indent))
    {
      BOLT_INFO(true, "Unable to load sparse graph from file");
      planner_ = planner_ + "NP";  // append "no-preprocessing to planner name
      return false;
    }
  }
  else if (is_thunder_)
  {
    // Already loaded in thunder_->setup();
  }
  else if (is_lightning_)
  {
    // Already loaded in lightning_->setup();
  }

  if (track_memory_consumption_)  // Track memory usage
  {
    double vm2, rss2;
    processMemUsage(vm2, rss2);
    BOLT_INFO(true, "RAM usage diff - VM: " << vm2 - vm1 << " MB | RSS: " << rss2 - rss1 << " MB");
  }

  return true;
}

// Loop through each planner to benchmark
void BoltBaxter::eachPlanner(std::size_t indent)
{
  // Logging
  if (use_logging_)
  {
    std::string file_path;
    bolt_moveit::getFilePath(file_path, log_file_name_, "ros/ompl_storage");
    logging_file_.open(file_path.c_str(), std::ios::out | std::ios::app);
    // Header of CSV file
    logging_file_ << "planner, thread, penetrate, planTime, pathLen, smoothTime, addV, addE, numV, numE, inserted, "
                     "insertTime, solved" << std::endl;
  }

  for (std::size_t i = 0; i < planners_.size(); ++i)
  {
    planner_ = planners_[i];
    experiment_name_ = experiment_names_[i];

    // Feedback
    std::cout << std::endl;
    std::cout << "##############################################################################################\n";
    BOLT_INFO(true, "Testing with planner '" << planner_ << "' (" << experiment_name_ << ")");
    std::cout << "##############################################################################################\n\n";

    // Load planning
    if (!loadOMPL(indent))
    {
      BOLT_ERROR("Unable to load planning context");
      exit(-1);
    }

    // // TEMP
    // std::cout << "before " << std::endl;
    // space_->copyToOMPLState(ompl_start_, *imarker_start_->getRobotState());
    // double dist;
    // if (!si_->getStateValidityChecker()->isValid(ompl_start_, dist))
    //   std::cout << "not VALID " << std::endl;
    // std::cout << "dist: " << dist << std::endl;
    // exit(0);

    // --------------------------------------
    run(indent);
    // --------------------------------------

    // Check for shutdown
    if (!ros::ok())
      break;

    // Wait for user
    if (i + 1 < planners_.size() - 1)  // if there are more planners to test
    {
      BOLT_INFO(true, "Next planner " << planners_[i + 1]);
    }

    // Clear previous planner
    reset(indent);
  }

  logging_file_.close();
}

void BoltBaxter::run(std::size_t indent)
{
  // Benchmark performance
  if (benchmark_performance_)
  {
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);

    benchmarkMemoryAllocation(indent);
    // testMotionValidator();
    // bolt_->getSparseGenerator()->benchmarkSparseGraphGeneration();
    // bolt_->getSparseGenerator()->benchmarkValidClearanceSampler();
    // bolt_->getSparseGenerator()->benchmarkRandValidSampling();
    // bolt_->getSparseGenerator()->benchmarkVisualizeSampling();
    // bolt_->getSparseGenerator()->benchmarkMemoryAllocation();
    BOLT_INFO(true, "Finished benchmarking, shutting down");
    exit(0);
  }

  // Create list of goal states
  if (save_imarkers_to_file_)
    saveIMarkersToFile(indent);

  // View list of goal states
  if (view_imarkers_from_file_)
    viewIMarkersFromFile(indent);

  // Load from file
  bool loaded = false;
  if (is_bolt_)
  {
    if (load_spars_)
    {
      loaded = loadData(indent);
    }

    // Create SPARS
    if (create_spars_ && (!loaded || continue_spars_))
    {
      // TODO: using unsafe non-locking version of planning scene, need to use something like
      // TODO: validity_checker_->setPlanningScene(scene);

      //bolt_->getSparseGenerator()->createSPARS();
      bolt_->getSparseGenerator()->createSPARS2(indent);
      loaded = true;
      BOLT_INFO(true, "Finished creating SPARS, shutting down");
      exit(0);  // after creating graph just end
    }
    if (!loaded)
      BOLT_WARN(true, "Creating AND loading sparse graph disabled, no contents in graph");
  }

  // Display disconnected components
  if (display_disjoint_sets_ && is_bolt_)
  {
    displayDisjointSets(indent);
    BOLT_INFO(true, "Finished, shutting down");
    exit(0);
  }

  // Repair missing coverage in the dense graph
  // if (eliminate_dense_disjoint_sets_)
  // {
  // TODO: using unsafe non-locking version of planning scene, need to use something like
  // TODO: validity_checker_->setPlanningScene(scene);
  //   bolt_->getSparseGraph()->getDiscretizer()->eliminateDisjointSets();
  // }

  // Check for verticies that are somehow in collision
  if (check_valid_vertices_)
  {
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);

    bolt_->getSparseGraph()->verifyGraph(indent);
    BOLT_INFO(true, "Finished, shutting down");
    exit(0);
  }

  // Mirror graph
  if (mirror_graph_)
  {
    mirrorGraph(indent);
    BOLT_INFO(true, "Finished, shutting down");
    exit(0);
  }

  // Display Database
  if (display_database_)
  {
    BOLT_INFO(true, "Display database");
    const bool showVertices = false;
    const bool showEdges = true;
    const std::size_t windowID = 6;
    bolt_->getSparseGraph()->displayDatabase(showVertices, showEdges, windowID, indent);
  }

  // Run the demo
  if (!run_problems_)
  {
    BOLT_INFO(true, "Solving requested to be skipped by config file");
    return;
  }

  // -----------------------------------------------
  // -----------------------------------------------
  runProblems(indent);
  // -----------------------------------------------
  // -----------------------------------------------

  if (is_bolt_)
  {
    bolt_->saveIfChanged(indent);
  }
  else if (is_thunder_)
    thunder_->saveIfChanged();
  else if (is_lightning_)
    lightning_->saveIfChanged();
}

bool BoltBaxter::runProblems(std::size_t indent)
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

bool BoltBaxter::plan(std::size_t run_id, std::size_t indent)
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

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus status = simple_setup_->solve(ptc);

  bool solved = false;
  if (status == ob::PlannerStatus::EXACT_SOLUTION)
    solved = true;

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;

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

void BoltBaxter::loadCollisionChecker(const planning_scene::PlanningSceneConstPtr &planning_scene, std::size_t indent)
{
  // Create state validity checking for this space
  validity_checker_ = std::make_shared<bolt_moveit::StateValidityChecker>(planning_group_name_, si_, *current_state_,
                                                                          planning_scene, space_);
  validity_checker_->setCheckingEnabled(collision_checking_enabled_);

  // Set checker
  si_->setStateValidityChecker(validity_checker_);

  // The interval in which obstacles are checked for between states
  // seems that it default to 0.01 but doesn't do a good job at that level
  // si_->setStateValidityCheckingResolution(0.005);
  si_->setStateValidityCheckingResolution(0.001);

  // Allow collision checker to visualize
  validity_checker_->setVisual(visual_);

  // Load more collision checkers
  // if (is_bolt_)
  // {
  //   secondary_si_ = std::make_shared<ob::SpaceInformation>(space_);
  //   secondary_si_->setup();
  //   validity_checker_ = std::make_shared<bolt_moveit::StateValidityChecker>(planning_group_name_, secondary_si_,
  //                                                                           *current_state_, planning_scene, space_);
  //   validity_checker_->setCheckingEnabled(collision_checking_enabled_);

  //   // Set checker
  //   secondary_si_->setStateValidityChecker(validity_checker_);

  //   // The interval in which obstacles are checked for between states
  //   // seems that it default to 0.01 but doesn't do a good job at that level
  //   // si_->setStateValidityCheckingResolution(0.005);
  //   secondary_si_->setStateValidityCheckingResolution(0.001);
  // }
}

void BoltBaxter::deleteAllMarkers(std::size_t indent)
{
  // Reset rviz markers
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    visual_tools_[i]->deleteAllMarkers();
    visual_tools_[i]->trigger();
  }
}

void BoltBaxter::displayWaitingState(bool waiting)
{
  // std::cout << " TODO display waiting state " << std::endl;
  // if (waiting)
  //   publishViewFinderFrame(rvt::MEDIUM);
  // else
  //   publishViewFinderFrame(rvt::XSMALL);

  // viz_bg_->trigger();
}

void BoltBaxter::testConnectionToGraphOfRandStates(std::size_t indent)
{
  ob::State *random_state = space_->allocState();

  std::size_t successful_connections = 0;
  for (std::size_t run_id = 0; run_id < num_problems_; ++run_id)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    BOLT_INFO(true, "Testing random state " << run_id);

    // Generate random state
    getRandomState(moveit_start_);

    // Visualize
    // visual_tools_[6]->publishRobotState(moveit_start_, rvt::GREEN);

    // Convert to ompl
    space_->copyToOMPLState(random_state, *moveit_start_);

    // Test
    const ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(60.0);
    std::size_t indent = 0;

    BOLT_ERROR("moveit_shelf_picking: not implemented");
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);
    // bool result = bolt_->getBoltPlanner()->canConnect(random_state, ptc, indent);
    // if (result)
    //   successful_connections++;

    BOLT_ERROR("Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
  }

  space_->freeState(random_state);
}

void BoltBaxter::visualizeRawTrajectory(og::PathGeometric &path, std::size_t indent)
{
  BOLT_INFO(true, "Visualizing non-interpolated trajectory");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.05;
  space_->convertPathToRobotState(path, planning_jmg_, traj, speed);

  // Show trajectory line
  visual_tools_[3]->publishTrajectoryLine(traj, arm_datas_[0].ee_link_, rvt::GREY);  // TODO multiple EEs
  visual_->viz3()->trigger();
}

bool BoltBaxter::generateCartGraph(std::size_t indent)
{
  // Generate the Descartes graph - if it fails let user adjust interactive marker
  while (true)
  {
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);
    if (!cart_path_planner_->populateBoltGraph(bolt_->getTaskGraph(), indent))
    {
      BOLT_INFO(true, "Unable to populate Bolt graph - try moving the start location");
      visual_->viz1()->prompt("attempt Bolt graph generation again");
      if (!ros::ok())
      {
        BOLT_INFO(true, "Finished, shutting down");
        exit(0);
      }
    }
    else
      break;
  }

  return true;
}

bool BoltBaxter::checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj, std::size_t indent)
{
  std::size_t state_count = traj->getWayPointCount();
  if (state_count < 3)
  {
    BOLT_WARN(true, "checkMoveItPathSolution: Solution path has only " << state_count << " states");
  }
  else
  {
    BOLT_INFO(true, "checkMoveItPathSolution: Solution path has " << state_count << " states");
  }

  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);

  std::vector<std::size_t> index;
  const bool verbose = true;
  std::cout << "before isPathValid " << std::endl;
  if (planning_scene->isPathValid(*traj, "", verbose, &index))
  {
    return true;
  }

  std::cout << "after isPathValid " << std::endl;
  // if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
  //   ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
  // else
  {
    // display error messages
    std::stringstream ss;
    for (std::size_t i = 0; i < index.size(); ++i)
      ss << index[i] << " ";
    BOLT_ERROR("checkMoveItPathSolution: Computed path is not valid. Invalid states at index locations: [ "
               << ss.str() << "] out of " << state_count << ". Explanations follow in command line.");

    // Call validity checks in verbose mode for the problematic states
    visualization_msgs::MarkerArray combined_array;
    for (std::size_t i = 0; i < index.size(); ++i)
    {
      // check validity with verbose on
      const robot_state::RobotState &robot_state = traj->getWayPoint(index[i]);
      bool check_verbose = true;
      planning_scene->isStateValid(robot_state, planning_group_name_, check_verbose);

      // compute the contacts if any
      collision_detection::CollisionRequest c_req;
      collision_detection::CollisionResult c_res;
      c_req.contacts = true;
      c_req.max_contacts = 10;
      c_req.max_contacts_per_pair = 3;
      c_req.verbose = false;
      planning_scene->checkCollision(c_req, c_res, robot_state);

      if (c_res.contact_count > 0)
      {
        visualization_msgs::MarkerArray single_array;
        collision_detection::getCollisionMarkersFromContacts(single_array, planning_scene->getPlanningFrame(),
                                                             c_res.contacts);
        combined_array.markers.insert(combined_array.markers.end(), single_array.markers.begin(),
                                      single_array.markers.end());
      }
    }  // end for
    // publish marker array
    visual_tools_[6]->publishMarkers(combined_array);

    BOLT_ERROR("checkMoveItPathSolution: Completed listing of explanations for invalid states.");
  }

  return false;
}

bool BoltBaxter::getRandomState(moveit::core::RobotStatePtr &robot_state)
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

void BoltBaxter::testMotionValidator(std::size_t indent)
{
  // THIS FUNCTION BROKEN BECAUSE moveit_core SAYS "FCL continuous collision checking not yet implemented"

  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);

  // moveit::core::RobotStatePtr start = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  // moveit::core::RobotStatePtr goal = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  moveit_start_->setToRandomPositions(planning_jmg_);
  moveit_goal_->setToRandomPositions(planning_jmg_);

  // visual_tools_[6]->publishRobotState(moveit_start_, rvt::GREEN);
  // visual_moveit_goal_->publishRobotState(moveit_goal_, rvt::ORANGE);

  // Check for collision between to states
  bool verbose = true;
  collision_detection::CollisionResult res;
  planning_scene->checkCollision(validity_checker_->collision_request_with_distance_verbose_, res, *moveit_start_);
  std::cout << "start state in collision: " << res.collision << std::endl;

  collision_detection::CollisionRequest req;
  req.group_name = planning_group_name_;
  req.verbose = true;

  // Check motion
  planning_scene->getCollisionWorld()->checkCollision(req, res, *planning_scene->getCollisionRobot(), *moveit_start_,
                                                      *moveit_goal_);

  std::cout << "motion in collision: " << res.collision << std::endl;
}

void BoltBaxter::mirrorGraph(std::size_t indent)
{
  // Choose planning group
  both_arms_jmg_ = robot_model_->getJointModelGroup(both_arms_group_name_);
  left_arm_jmg_ = robot_model_->getJointModelGroup(opposite_arm_name_);

  // Setup space
  bolt_moveit::ModelBasedStateSpaceSpecification both_arms_mbss_spec(robot_model_, both_arms_jmg_);
  bolt_moveit::ModelBasedStateSpaceSpecification left_arm_mbss_spec(robot_model_, left_arm_jmg_);

  // Construct the state space we are planning in
  both_arms_state_space_ = bolt_moveit::chooseModelSizeStateSpace(both_arms_mbss_spec);
  left_arm_state_space_ = bolt_moveit::chooseModelSizeStateSpace(left_arm_mbss_spec);

  both_arms_state_space_->setup();
  both_arms_state_space_->setName(both_arms_group_name_);
  left_arm_state_space_->setup();
  left_arm_state_space_->setName(opposite_arm_name_);

  // SpaceInfo
  ob::SpaceInformationPtr both_arms_space_info = std::make_shared<ob::SpaceInformation>(both_arms_state_space_);
  ob::SpaceInformationPtr left_arm_space_info = std::make_shared<ob::SpaceInformation>(left_arm_state_space_);
  both_arms_space_info->setup();
  left_arm_space_info->setup();

  // Create state validity checking for both arms
  bolt_moveit::StateValidityCheckerPtr both_arms_validity_checker = std::make_shared<bolt_moveit::StateValidityChecker>(
      both_arms_group_name_, both_arms_space_info, *current_state_, psm_->getPlanningScene(), both_arms_state_space_);
  both_arms_space_info->setStateValidityChecker(both_arms_validity_checker);

  // Create state validity checking for left arm
  bolt_moveit::StateValidityCheckerPtr left_arm_validity_checker = std::make_shared<bolt_moveit::StateValidityChecker>(
      opposite_arm_name_, left_arm_space_info, *current_state_, psm_->getPlanningScene(), left_arm_state_space_);
  left_arm_space_info->setStateValidityChecker(left_arm_validity_checker);

  // Set the database file location
  const std::string file_path = getPlannerFilePath(both_arms_group_name_, indent);

  // Test all verticies
  if (false)
  {
    BOLT_INFO(true, "TESTING ALL VERTICES ON OTHER ARM");
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);
    bolt_->getSparseMirror()->checkValidityOfArmMirror(both_arms_space_info, left_arm_space_info, indent);

    BOLT_INFO(true, "Finished, shutting down");
    exit(0);
  }

  // Set callback for how to combine two arms into one state
  bolt_->getSparseMirror()->setCombineStatesCallback(boost::bind(&BoltBaxter::combineStates, this, _1, _2));

  // Mirror graph
  // TODO: using unsafe non-locking version of planning scene, need to use something like
  // TODO: validity_checker_->setPlanningScene(scene);
  bolt_->getSparseMirror()->mirrorGraphDualArm(both_arms_space_info, left_arm_space_info, file_path, indent);
  BOLT_INFO(true, "Done mirroring graph!");
}

ob::State *BoltBaxter::combineStates(const ob::State *state1, const ob::State *state2)
{
  /* Notes:
     state1
     state space: space_
     jmg: planning_jmg_
     state2
     state space: left_arm_state_space_
     jmg: left_arm_jmg_
     return state
     state space: both_arms_state_space_
     jmg: both_arms_jmg_
  */

  ob::State *both_arms_state = both_arms_state_space_->allocState();

  // Get the values of the individual states
  std::vector<double> values1, values2;
  si_->getStateSpace()->copyToReals(values1, state1);
  si_->getStateSpace()->copyToReals(values2, state2);

  // Set the vectors for each joint model group
  // TODO: its possible the vectors do not align correctly for some robots, but I'm not sure
  mirror_state_->setJointGroupPositions(planning_jmg_, values1);
  mirror_state_->setJointGroupPositions(left_arm_jmg_, values2);

  // Fill the state with current values
  both_arms_state_space_->copyToOMPLState(both_arms_state, *mirror_state_);

  if (false)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    space_->printState(state1);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    left_arm_state_space_->printState(state2);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    both_arms_state_space_->printState(both_arms_state);

    visual_->viz1()->prompt("compare combination");
  }

  return both_arms_state;
}

void BoltBaxter::benchmarkMemoryAllocation(std::size_t indent)
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("BoltBaxter: Running memory allocation benchmark");

  // std::size_t numStates = 10000000;
  std::size_t numStates = 2;
  std::size_t dim = 14;
  std::size_t tests = 2;

  bolt_moveit::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, planning_jmg_);
  bolt_moveit::ModelBasedStateSpace space_old(mbss_spec);
  // bolt_moveit::ModelBasedStateSpacePtr space = bolt_moveit::chooseModelSizeStateSpace(mbss_spec);

  // METHOD 1
  ros::Time start_time = ros::Time::now();  // Benchmark runtime
  for (std::size_t test = 0; test < tests; ++test)
  {
    // Allocate
    std::vector<ob::State *> states;
    for (std::size_t i = 0; i < numStates; ++i)
      states.push_back(space_old.allocState());

    // Free
    for (std::size_t i = 0; i < numStates; ++i)
      space_old.freeState(states[i]);
  }
  BOLT_INFO(true, "Old state - Total time: " << (ros::Time::now() - start_time).toSec() << " seconds");

  visual_->viz1()->prompt("finished running");

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void BoltBaxter::loadScene(std::size_t indent)
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

void BoltBaxter::loadOfficeScene(std::size_t indent)
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

void BoltBaxter::loadAmazonScene(std::size_t indent)
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

void BoltBaxter::loadBin(double y, std::size_t indent)
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

void BoltBaxter::saveIMarkersToFile(std::size_t indent)
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

void BoltBaxter::viewIMarkersFromFile(std::size_t indent)
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

void BoltBaxter::loadIMarkersFromFile(std::vector<moveit::core::RobotStatePtr> &robot_states, std::string file_name,
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

void BoltBaxter::loadIMarkers(std::size_t indent)
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

robot_trajectory::RobotTrajectoryPtr BoltBaxter::processSimpleSolution(std::size_t indent)
{
  BOLT_FUNC(true, "processSimpleSolution()");

  og::PathGeometric &path = static_cast<og::PathGeometric &>(*simple_setup_->getProblemDefinition()->getSolutionPath());

  last_plan_path_length_ = path.length();

  // Convert trajectory from OMPL to MoveIt! format
  robot_trajectory::RobotTrajectoryPtr trajectory;
  const double speed = 0.025;
  if (!space_->convertPathToRobotState(path, planning_jmg_, trajectory, speed))
  {
    BOLT_ERROR("Unable to convert path");
    return false;
  }

  // Interpolate and parameterize
  if (connect_to_hardware_)  // if running on hardware, add accel/vel
  {
    const double velocity_scaling_factor = velocity_scaling_factor_;
    const bool use_interpolation = true;
    planning_interface_->convertRobotStatesToTraj(trajectory, planning_jmg_, velocity_scaling_factor,
                                                  use_interpolation);
  }
  return trajectory;
}

robot_trajectory::RobotTrajectoryPtr BoltBaxter::processSegments(std::size_t indent)
{
  BOLT_FUNC(true, "processSegments()");

  // Visualize if not already done so within BoltPlanner
  if (!bolt_->getBoltPlanner()->visualizeRawTrajectory_)
    bolt_->getBoltPlanner()->visualizeRaw(indent);
  if (!bolt_->getBoltPlanner()->visualizeSmoothTrajectory_)
    bolt_->getBoltPlanner()->visualizeSmoothed(indent);

  // Get solution segments
  std::vector<og::PathGeometricPtr> model_sol_segments = bolt_->getBoltPlanner()->getModelSolSegments();
  robot_trajectory::RobotTrajectoryPtr combined_traj =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_jmg_);

  // For each segment of trajectory
  for (std::size_t i = 0; i < model_sol_segments.size(); ++i)
  {
    og::PathGeometricPtr path_segment = model_sol_segments[i];

    // Convert trajectory from OMPL to MoveIt! format
    robot_trajectory::RobotTrajectoryPtr traj_segment;
    const double speed = 0.025;
    if (!space_->convertPathToRobotState(*path_segment, planning_jmg_, traj_segment, speed))
    {
      BOLT_ERROR("Unable to convert path");
      return false;
    }

    // Check/test the solution for errors
    // if (!checkMoveItPathSolution(traj_segment))
    // {
    //   BOLT_WARN(true, "Invalid path");
    // }

    // Loop through each state in subtrajectory
    if (false)
      for (std::size_t i = 0; i < traj_segment->getWayPointCount(); ++i)
      {
        std::cout << "i: " << i << std::endl;
        visual_tools_[6]->publishRobotState(traj_segment->getWayPoint(i), rvt::BLUE);

        visual_->viz1()->prompt("next step");
      }

    // For the cartesian path, go real slow
    double velocity_scaling_factor = velocity_scaling_factor_;
    if (i == 1)
      velocity_scaling_factor = 0.1;

    // Interpolate and parameterize
    const bool use_interpolation = false;
    if (connect_to_hardware_)  // if running on hardware, add accel/vel
      planning_interface_->convertRobotStatesToTraj(traj_segment, planning_jmg_, velocity_scaling_factor,
                                                    use_interpolation);

    // Add to combined traj
    const double dt = i == 0 ? 0.0 : 1.0;  // Quick pause between segments except first one
    combined_traj->append(*traj_segment, dt);
  }

  // Get the resulting length
  last_plan_path_length_ = bolt_->getBoltPlanner()->getSmoothedModelSolPath()->length();

  return combined_traj;
}

bool BoltBaxter::chooseStartGoal(std::size_t run_id, std::size_t indent)
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

bool BoltBaxter::setSingleStartFromIMarker(std::size_t indent)
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

bool BoltBaxter::setSingleGoalFromIMarker(std::size_t indent)
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

void BoltBaxter::displayDisjointSets(std::size_t indent)
{
  std::cout << std::endl;
  BOLT_INFO(true, "Displaying disjoint sets ----------- ");
  ot::bolt::SparseDisjointSetsMap disjointSets;
  bolt_->getSparseGraph()->getDisjointSets(disjointSets, indent);
  bolt_->getSparseGraph()->printDisjointSets(disjointSets, indent);
  bolt_->getSparseGraph()->visualizeDisjointSets(disjointSets, indent);
}

std::string BoltBaxter::getPlannerFilePath(const std::string &planning_group_name, std::size_t indent)
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

void BoltBaxter::doPostProcessing(std::size_t indent)
{
  // Lock the planning scene for read-only while a plan is solved - no outside node should be allowed to modify
  // This is achieved by updating the version of the planning_scene inside StateValidityChecker
  psm::LockedPlanningSceneRO lscene(psm_);
  const ps::PlanningSceneConstPtr &scene = static_cast<const ps::PlanningSceneConstPtr &>(lscene);
  validity_checker_->setPlanningScene(scene);

  if (is_bolt_)
  {
    // TODO: using unsafe non-locking version of planning scene, need to use something like
    // TODO: validity_checker_->setPlanningScene(scene);
    bolt_->doPostProcessing(indent);
  }
  else if (is_thunder_)
  {
    thunder_->doPostProcessing();
  }
  else if (is_lightning_)
  {
    lightning_->doPostProcessing();
  }
}

void BoltBaxter::loadSPARS2Data(std::size_t indent)
{
  using namespace rosparam_shortcuts;
  std::size_t error = 0;

  double sparseDeltaFraction_;
  double denseDeltaFraction_;
  double nearSamplePointsMultiple_;
  double stretchFactor_;
  double penetrationOverlapFraction_;
  bool useL2Norm_;
  {
    ros::NodeHandle rpnh(nh_, "sparse_criteria");
    error += !get(name_, rpnh, "sparse_delta_fraction", sparseDeltaFraction_);
    error += !get(name_, rpnh, "dense_delta_fraction", denseDeltaFraction_);
    error += !get(name_, rpnh, "near_sample_points_multiple", nearSamplePointsMultiple_);
    error += !get(name_, rpnh, "stretch_factor", stretchFactor_);
    error += !get(name_, rpnh, "penetration_overlap_fraction", penetrationOverlapFraction_);
    error += !get(name_, rpnh, "use_l2_norm", useL2Norm_);
  }

  // Mimmic bolt method for calculating
  ompl::tools::bolt::SparseFormula formulas;
  static bool verboseOnce = true;
  formulas.calc(si_, stretchFactor_, sparseDeltaFraction_, penetrationOverlapFraction_, nearSamplePointsMultiple_,
                useL2Norm_, verboseOnce, indent);
  if (verboseOnce)
    verboseOnce = false;  // only show this once

  spars2_->setSparseDeltaFraction(sparseDeltaFraction_);
  spars2_->setDenseDeltaFraction(denseDeltaFraction_);
  spars2_->setStretchFactor(formulas.stretchFactor_);  // uses same method as Bolt to calculate

  {
    ros::NodeHandle rpnh(nh_, "sparse_generator");
    std::size_t temp, total_failures;
    error += !get(name_, rpnh, "terminate_after_failures", temp);
    total_failures = temp;
    error += !get(name_, rpnh, "fourth_criteria_after_failures", temp);
    total_failures += temp;
    // total_failures *= 2; // Because SPARS2 keeps failing the optimiality test with "no neighbors found"
    std::cout << " Found total failures: " << total_failures << std::endl;
    spars2_->setMaxFailures(total_failures);
  }

  // Clearance
  // {
  //   ros::NodeHandle rpnh(nh_, "sparse_graph");
  //   double clearance;
  //   error += !get(name_, rpnh, "obstacle_clearance", clearance);
  //   spars2_->setClearance(clearance);
  // }

  shutdownIfError(name_, error);
}

void BoltBaxter::log(bool solved, std::size_t indent)
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

void BoltBaxter::processAndExecute(robot_trajectory::RobotTrajectoryPtr execution_traj, std::size_t indent)
{
  // Execute
  if (connect_to_hardware_)
  {
    bool wait_for_execution = true;
    execution_interface_->executeTrajectory(execution_traj, planning_jmg_, wait_for_execution);
  }
  else  // Simulation
  {
    // visual_tools_[6]->deleteAllMarkers();
    // visual_tools_[6]->publishTrajectoryLine(execution_traj, planning_jmg_, rvt::LIME_GREEN);
    // visual_tools_[6]->trigger();
    bool blocking = false;
    visual_tools_[6]->publishTrajectoryPath(execution_traj, blocking);
  }
}

void BoltBaxter::generateMultiGoalsFromIK(std::size_t run_id, std::size_t indent)
{
  const double half_shelf_depth = -0.44;     // brings arrows to edge of shelf in x direction
  const double horizontal_distance = 0.275;  // left and right cubbys
  const double height_of_shelf = 0.24;
  const double penetration_depth = penetration_dist_;  // test_var_; // m

  double z = (run_id % num_rows_) * height_of_shelf;

  Eigen::Affine3d left_gripper = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  left_gripper.translation().x() = half_shelf_depth + penetration_depth;
  left_gripper.translation().y() = -horizontal_distance;  // left cubby
  left_gripper.translation().z() = z;
  left_gripper = common_transform_ * left_gripper;
  visual_tools_[6]->publishZArrow(left_gripper, rvt::BLUE, rvt::LARGE);

  Eigen::Affine3d right_gripper = Eigen::Affine3d::Identity() * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
  right_gripper.translation().x() = half_shelf_depth + penetration_depth;
  right_gripper.translation().y() = horizontal_distance;  // right cubby
  right_gripper.translation().z() = z;
  right_gripper = common_transform_ * right_gripper;
  visual_tools_[6]->publishZArrow(right_gripper, rvt::GREEN, rvt::LARGE);

  // Visualize
  visual_tools_[6]->trigger();

  // IK Solve
  EigenSTL::vector_Affine3d poses;
  poses.push_back(right_gripper);
  poses.push_back(left_gripper);

  std::shared_ptr<ompl::base::GoalStates> goals = std::make_shared<ompl::base::GoalStates>(si_);
  for (std::size_t i = 0; i < 10; ++i)
  {
    // Randomize so we get different pose
    // TODO: discretize instead of random, but that is more work...
    imarker_goal_->getRobotStateNonConst()->setToRandomPositions(planning_jmg_);
    imarker_goal_->setFromPoses(poses, planning_jmg_);

    space_->copyToOMPLState(ompl_goal_, *imarker_goal_->getRobotState());
    goals->addState(ompl_goal_);  // this command clones the state / does not own our memory
  }
  ompl::base::GoalPtr goal = std::dynamic_pointer_cast<ompl::base::Goal>(goals);
  simple_setup_->setGoal(goals);
}

void BoltBaxter::visualizeRawSolutionLine(std::size_t indent)
{
  og::PathGeometric &path = static_cast<og::PathGeometric &>(*simple_setup_->getProblemDefinition()->getSolutionPath());
  visual_->viz6()->deleteAllMarkers();
  visual_->viz6()->path(&path, ot::MEDIUM, ot::BLUE, ot::CYAN);
  visual_->viz6()->trigger();
}

void BoltBaxter::visualizeSmoothSolutionLine(std::size_t indent)
{
  og::PathGeometric &path = static_cast<og::PathGeometric &>(*simple_setup_->getProblemDefinition()->getSolutionPath());
  visual_->viz6()->path(&path, ot::SMALL, ot::GREEN, ot::LIME_GREEN);
  visual_->viz6()->trigger();
}

}  // namespace moveit_shelf_picking
