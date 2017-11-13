// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Other tools
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

static const std::string PLANNING_SCENE_MONITOR_NAME = "MoveItBasePlanningScene";
static const std::string ROBOT_DESCRIPTION = "robot_description";

namespace psm = planning_scene_monitor;
namespace rml = robot_model_loader;
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "demo");
  ROS_INFO_STREAM_NAMED("main", "Starting demo...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh("~");

  rml::RobotModelLoaderPtr rml = std::make_shared<rml::RobotModelLoader>(ROBOT_DESCRIPTION);

  // METHOD 1 -------------------------------
  // robot_model::RobotModelPtr robot_model = rml->getModel();
  // planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
  // boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener(nh));
  // psm::PlanningSceneMonitorPtr psm =
  //     std::make_shared<psm::PlanningSceneMonitor>(planning_scene, rml, tf, PLANNING_SCENE_MONITOR_NAME);
  // METHOD 2 --------------------------------
  psm::PlanningSceneMonitorPtr psm = std::make_shared<psm::PlanningSceneMonitor>(rml);

  psm::PlanningSceneMonitor::SceneUpdateType event = psm::PlanningSceneMonitor::UPDATE_NONE;
  // publish_geometry_updates
  event = (psm::PlanningSceneMonitor::SceneUpdateType)((int)event | (int)psm::PlanningSceneMonitor::UPDATE_GEOMETRY);
  // publish_state_updates
  event = (psm::PlanningSceneMonitor::SceneUpdateType)((int)event | (int)psm::PlanningSceneMonitor::UPDATE_STATE);
  // publish_transforms_updates
  event = (psm::PlanningSceneMonitor::SceneUpdateType)((int)event | (int)psm::PlanningSceneMonitor::UPDATE_TRANSFORMS);

  if (!psm->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED("main", "Planning scene not configured");
    exit(-1);
  }

  // Optional monitors to start:
  // psm->startStateMonitor("/robot/joint_states", "");
  // psm->getPlanningScene()->setName("bolt_scene");
  psm->startPublishingPlanningScene(event, "/swri/planning_scene");
  // ros::Duration(0.5).sleep(); // give time for publisher to start

  // Create initial robot state
  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(psm->getPlanningScene()->getRobotModel());
  robot_state->setToDefaultValues();

  for (std::size_t i = 0; i < 10; ++i)
  {
    // Clear planning scene
    {
      ROS_INFO_STREAM_NAMED("main", "Clearing planning scene");
      planning_scene_monitor::LockedPlanningSceneRW scene(psm);
      scene->removeAllCollisionObjects();
    }
    psm->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    ros::Duration(0.5).sleep();  // 2hz refresh rate of psm

    // Check collision
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(psm);  // Read only lock
      collision_detection::CollisionResult res;
      collision_detection::CollisionRequest req;
      req.group_name = "";
      req.verbose = true;
      locked_planning_scene->checkCollision(req, res, *robot_state);
      if (res.collision)
        ROS_ERROR_STREAM_NAMED("main", "In collision 1 - bad");
      else
        ROS_INFO_STREAM_NAMED("main", "Not in collision 1 - good");
    }

    // Create obstacle
    moveit_msgs::CollisionObject collision_obj;
    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = "world";
    collision_obj.id = "box";
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    collision_obj.primitives[0].dimensions.resize(
        geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1;
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1;
    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0].orientation.w = 1;

    // Add obstacle to planning scene
    psm->updateFrameTransforms();
    psm->lockSceneWrite();
    psm->getPlanningScene()->processCollisionObjectMsg(collision_obj);
    psm->unlockSceneWrite();
    psm->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Create request
    collision_detection::CollisionRequest req;
    req.group_name = "";
    req.verbose = false;

    // Check collision
    {
      ros::Duration(1.0).sleep();
      collision_detection::CollisionResult res;
      psm->getPlanningScene()->checkCollision(req, res, *robot_state);
      if (res.collision)
        ROS_INFO_STREAM_NAMED("main", "In collision 2 - good");
      else
        ROS_ERROR_STREAM_NAMED("main", "Not in collision 2 - bad");
    }

    // Check collision
    {
      ros::Time start_time0 = ros::Time::now();                                  // Benchmark runtime
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(psm);  // Read only lock
      ROS_INFO_STREAM_NAMED("main", "Total time: " << (ros::Time::now() - start_time0).toSec() << " seconds");

      collision_detection::CollisionResult res;
      locked_planning_scene->checkCollision(req, res, *robot_state);
      if (res.collision)
        ROS_INFO_STREAM_NAMED("main", "In collision 3 - good");
      else
        ROS_ERROR_STREAM_NAMED("main", "Not in collision 3 - bad");
    }

    ros::Duration(1.0).sleep();

    // Check collision
    {
      planning_scene_monitor::LockedPlanningSceneRO locked_planning_scene(psm);  // Read only lock
      collision_detection::CollisionResult res;
      locked_planning_scene->checkCollision(req, res, *robot_state);
      if (res.collision)
        ROS_INFO_STREAM_NAMED("main", "In collision 4 - good");
      else
        ROS_ERROR_STREAM_NAMED("main", "Not in collision 4 - bad");
    }
  }

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
