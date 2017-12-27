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

  // run type
  error += !rosparam_shortcuts::get(name_, rpnh, "auto_run", auto_run_);
  error += !rosparam_shortcuts::get(name_, rpnh, "num_problems", num_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "headless", headless_);
  error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_goal_list", imarker_goal_list_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_start_list", imarker_start_list_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "arm_jmgs", arm_jmgs);
  error += !rosparam_shortcuts::get(name_, rpnh, "ee_tip_links", ee_tip_links);
  error += !rosparam_shortcuts::get(name_, rpnh, "seed_random", seed_random);
  // collision
  error += !rosparam_shortcuts::get(name_, rpnh, "scene_type", scene_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "distance_to_shelf", distance_to_shelf_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_shelf_noise", use_shelf_noise_);
  error += !rosparam_shortcuts::get(name_, rpnh, "bin_height", bin_height_);
  error += !rosparam_shortcuts::get(name_, rpnh, "num_rows", num_rows_);
  error += !rosparam_shortcuts::get(name_, rpnh, "penetration_start", penetration_start_);
  // execution
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

  // Load more visual tool objects
  loadVisualTools(indent);

  // Add a collision objects
  loadScene(indent);
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
    MoveItVisualToolsPtr moveit_visual = std::make_shared<MoveItVisualTools>(base_frame, marker_topic, planning_scene_monitor_);
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
    ROS_INFO_STREAM("Penetration distance: " << penetration_dist_);
    std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ " << std::endl;

    // Run the demo the desired number of times
    for (std::size_t run_id = 0; run_id < num_problems_; ++run_id)
    {
      if (!ros::ok())  // Check if user wants to shutdown
        break;

      std::cout << std::endl;
      std::cout << "***************************************************************" << std::endl;
      ROS_INFO_STREAM("Planning " << run_id + 1 << " out of " << num_problems_
                                  << " for penetration distance: " << penetration_dist_);
      std::cout << "***************************************************************" << std::endl;

      // -----------------------------------------------------
      // -----------------------------------------------------
      bool solved = plan(run_id, indent);
      // -----------------------------------------------------
      // -----------------------------------------------------

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

  return true;
}

bool BaxterBenchmark::plan(std::size_t run_id, std::size_t indent)
{
  // Setup -----------------------------------------------------------

  // Generate start/goal pairs
  while (!chooseStartGoal(run_id, indent))
  {
    ROS_WARN_STREAM("Invalid start/goal found, trying again");

    // Move the shelf
    loadAmazonScene(indent);
    visual_tools_[6]->triggerPlanningSceneUpdate();
  }


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
  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

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
      ROS_WARN_STREAM("Taking long time to find valid random state");
  }

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
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);  // Read/write lock
    collision_detection::AllowedCollisionMatrix &collision_matrix = planning_scene->getAllowedCollisionMatrixNonConst();
    collision_matrix.setEntry("wall", "pedestal", true);
  }
}

void BaxterBenchmark::loadOfficeScene(std::size_t indent)
{
  // planning_scene_monitor_->updateFrameTransforms();

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

    ROS_INFO_STREAM("loadAmazon() x_noise = " << x_noise << "; y_noise = " << y_noise
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

  Eigen::Affine3d mesh_centroid = Eigen::Affine3d::Identity();

  // Calculate offset
  mesh_centroid = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  mesh_centroid.translation().x() = distance_to_shelf_ * 0.65;
  mesh_centroid.translation().y() = y;
  mesh_centroid.translation().z() = bin_height_;

  const std::string object_name = "bin" + std::to_string(y);

  // Add mesh to scene
  visual_tools_[6]->publishCollisionMesh(mesh_centroid, object_name, collision_mesh_path, rvt::RED);
}

bool BaxterBenchmark::chooseStartGoal(std::size_t run_id, std::size_t indent)
{
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
      // if (imarker_start_states_.empty())
      //   loadIMarkersFromFile(imarker_start_states_, imarker_start_list_name_, indent);

      // start_state_id_ = run_id % imarker_start_states_.size();
      // ROS_INFO_STREAM("chooseStartGoal: total start states: " << imarker_start_states_.size() << " run_id: " << run_id
      //                                                         << " start_state_id: " << start_state_id_);

      // imarker_start_->setRobotState(imarker_start_states_[start_state_id_]);

      // // choose goal
      // if (imarker_goal_states_.empty())
      //   loadIMarkersFromFile(imarker_goal_states_, imarker_goal_list_name_, indent);

      // goal_state_id_ = run_id % imarker_goal_states_.size();
      // ROS_INFO_STREAM("chooseStartGoal: total goal states: " << imarker_goal_states_.size() << " run_id: " << run_id
      //                                                        << " goal_state_id: " << goal_state_id_);

      // imarker_goal_->setRobotState(imarker_goal_states_[goal_state_id_]);

      // // Apply imarker start/goal to OMPL SimpleSetup
      // if (!setSingleStartFromIMarker(indent))
      //   return false;
      // if (!setSingleGoalFromIMarker(indent))
      //   return false;
    }
    break;
    // from SRDF ------------------------------------------------
    case 3:
      ROS_WARN_STREAM("NOT IMPLEMENTED");
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
      // if (imarker_start_states_.empty())
      //   loadIMarkersFromFile(imarker_start_states_, imarker_start_list_name_, indent);

      // start_state_id_ = run_id % imarker_start_states_.size();
      // ROS_INFO_STREAM("chooseStartGoal: total start states: " << imarker_start_states_.size() << " run_id: " << run_id
      //                                                         << " start_state_id: " << start_state_id_);
      // imarker_start_->setRobotState(imarker_start_states_[start_state_id_]);

      // // Apply imarker start to OMPL SimpleSetup, **but not goal**
      // if (!setSingleStartFromIMarker(indent))
      //   return false;

      // // choose goal and set in OMPL SimpleSetup
      // generateMultiGoalsFromIK(run_id, indent);
      break;
    default:
      ROS_ERROR_STREAM("Unknown problem type");
      return false;
  }

  return true;
}

bool BaxterBenchmark::setSingleStartFromIMarker(std::size_t indent)
{
  // Validate with collision checking
  static const bool VERBOSE = true;
  if (!imarker_start_->isStateValid(VERBOSE))
  {
    ROS_WARN_STREAM("Invalid start state");
    return false;
  }

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
    ROS_WARN_STREAM("Invalid goal state");
    return false;
  }

  // // TODO: delete (temporary)
  // imarker_goal_->publishRobotState();

  // planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);  // Read only lock
  // if (!planning_scene->isStateValid(*imarker_goal_->getRobotState(), "", true))
  // {
  //   ROS_WARN_STREAM("Invalid goal state MoveIt! 1");
  //   exit(-1);
  // }

  // if (!planning_scene->isStateValid(*imarker_goal_->getRobotState(), planning_group_name_, true))
  // {
  //   ROS_WARN_STREAM("Invalid goal state MoveIt! 2");
  //   exit(-1);
  // }

  // collision_detection::CollisionRequest collision_request_simple_verbose;
  // collision_request_simple_verbose.group_name = planning_group_name_;
  // collision_request_simple_verbose.verbose = true;
  // collision_detection::CollisionResult res;
  // planning_scene_monitor_->getPlanningScene()->checkCollision(collision_request_simple_verbose, res, *imarker_goal_->getRobotState());
  // if (res.collision)
  // {
  //   ROS_WARN_STREAM("Invalid goal state OMPL 1");
  //   exit(-1);
  // }

  // if (!validity_checker_->isValid(ompl_goal_))
  // {
  //   ROS_WARN_STREAM("Invalid goal state OMPL 2");
  //   exit(-1);
  // }

  // Visualize
  if (visualize_start_goal_states_)
    imarker_goal_->publishRobotState();

  return true;
}


}  // namespace moveit_shelf_picking
