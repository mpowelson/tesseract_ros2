/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2013, Ioan A. Sucan
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "tesseract_rviz/tesseract_scene_plugin/tesseract_scene_display.h"
#include "tesseract_rviz/render_tools/env/env_visualization.h"
#include "tesseract_rviz/render_tools/env/env_link.h"
#include "tesseract_rviz/render_tools/octomap_render.h"
#include "tesseract_rviz/render_tools/state_visualization.h"

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace tesseract_rviz
{
// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
TesseractSceneDisplay::TesseractSceneDisplay(bool listen_to_planning_scene, bool show_scene_robot)
  : Display(), model_is_loading_(false), tesseract_scene_needs_render_(true), current_scene_time_(0.0f)
{
  move_group_ns_property_ = new rviz::StringProperty("Move Group Namespace",
                                                     "",
                                                     "The name of the ROS namespace in "
                                                     "which the move_group node is running",
                                                     this,
                                                     SLOT(changedMoveGroupNS()),
                                                     this);
  robot_description_property_ = new rviz::StringProperty("Robot Description",
                                                         "robot_description",
                                                         "The name of the ROS parameter where the URDF for the robot "
                                                         "is loaded",
                                                         this,
                                                         SLOT(changedRobotDescription()),
                                                         this);

  if (listen_to_planning_scene)
    planning_scene_topic_property_ =
        new rviz::RosTopicProperty("Planning Scene Topic",
                                   "move_group/monitored_planning_scene",
                                   ros::message_traits::datatype<moveit_msgs::PlanningScene>(),
                                   "The topic on which the moveit_msgs::PlanningScene messages are "
                                   "received",
                                   this,
                                   SLOT(changedPlanningSceneTopic()),
                                   this);
  else
    planning_scene_topic_property_ = nullptr;

  // Planning scene category
  // -------------------------------------------------------------------------------------------
  scene_category_ = new rviz::Property("Scene Geometry", QVariant(), "", this);

  scene_name_property_ = new rviz::StringProperty("Scene Name",
                                                  "(noname)",
                                                  "Shows the name of the planning scene",
                                                  scene_category_,
                                                  SLOT(changedSceneName()),
                                                  this);
  scene_name_property_->setShouldBeSaved(false);
  scene_enabled_property_ = new rviz::BoolProperty("Show Scene Geometry",
                                                   true,
                                                   "Indicates whether planning scenes should be displayed",
                                                   scene_category_,
                                                   SLOT(changedSceneEnabled()),
                                                   this);

  scene_alpha_property_ = new rviz::FloatProperty("Scene Alpha",
                                                  0.9f,
                                                  "Specifies the alpha for the scene geometry",
                                                  scene_category_,
                                                  SLOT(changedSceneAlpha()),
                                                  this);
  scene_alpha_property_->setMin(0.0);
  scene_alpha_property_->setMax(1.0);

  scene_color_property_ = new rviz::ColorProperty("Scene Color",
                                                  QColor(50, 230, 50),
                                                  "The color for the planning scene obstacles (if a color is not "
                                                  "defined)",
                                                  scene_category_,
                                                  SLOT(changedSceneColor()),
                                                  this);

  octree_render_property_ = new rviz::EnumProperty("Voxel Rendering",
                                                   "Occupied Voxels",
                                                   "Select voxel type.",
                                                   scene_category_,
                                                   SLOT(changedOctreeRenderMode()),
                                                   this);

  octree_render_property_->addOption("Occupied Voxels", OCTOMAP_OCCUPIED_VOXELS);
  octree_render_property_->addOption("Free Voxels", OCTOMAP_FREE_VOXELS);
  octree_render_property_->addOption("All Voxels", OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz::EnumProperty(
      "Voxel Coloring", "Z-Axis", "Select voxel coloring mode", scene_category_, SLOT(changedOctreeColorMode()), this);

  octree_coloring_property_->addOption("Z-Axis", OCTOMAP_Z_AXIS_COLOR);
  octree_coloring_property_->addOption("Cell Probability", OCTOMAP_PROBABLILTY_COLOR);

  scene_display_time_property_ = new rviz::FloatProperty("Scene Display Time",
                                                         0.2f,
                                                         "The amount of wall-time to wait in between rendering "
                                                         "updates to the planning scene (if any)",
                                                         scene_category_,
                                                         SLOT(changedSceneDisplayTime()),
                                                         this);
  scene_display_time_property_->setMin(0.0001);

  if (show_scene_robot)
  {
    robot_category_ = new rviz::Property("Scene Robot", QVariant(), "", this);

    scene_robot_visual_enabled_property_ = new rviz::BoolProperty("Show Robot Visual",
                                                                  true,
                                                                  "Indicates whether the robot state specified by the "
                                                                  "planning scene "
                                                                  "should be "
                                                                  "displayed as defined for visualisation purposes.",
                                                                  robot_category_,
                                                                  SLOT(changedSceneRobotVisualEnabled()),
                                                                  this);

    scene_robot_collision_enabled_property_ = new rviz::BoolProperty("Show Robot Collision",
                                                                     false,
                                                                     "Indicates whether the robot state specified by "
                                                                     "the planning scene "
                                                                     "should be "
                                                                     "displayed as defined for collision detection "
                                                                     "purposes.",
                                                                     robot_category_,
                                                                     SLOT(changedSceneRobotCollisionEnabled()),
                                                                     this);

    robot_alpha_property_ = new rviz::FloatProperty("Robot Alpha",
                                                    1.0f,
                                                    "Specifies the alpha for the robot links",
                                                    robot_category_,
                                                    SLOT(changedRobotSceneAlpha()),
                                                    this);
    robot_alpha_property_->setMin(0.0);
    robot_alpha_property_->setMax(1.0);

    attached_body_color_property_ = new rviz::ColorProperty("Attached Body Color",
                                                            QColor(150, 50, 150),
                                                            "The color for the attached bodies",
                                                            robot_category_,
                                                            SLOT(changedAttachedBodyColor()),
                                                            this);
  }
  else
  {
    robot_category_ = nullptr;
    scene_robot_visual_enabled_property_ = nullptr;
    scene_robot_collision_enabled_property_ = nullptr;
    robot_alpha_property_ = nullptr;
    attached_body_color_property_ = nullptr;
  }
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
TesseractSceneDisplay::~TesseractSceneDisplay()
{
  clearJobs();

  tesseract_scene_render_.reset();
  if (context_ && context_->getSceneManager() && tesseract_scene_node_)
    context_->getSceneManager()->destroySceneNode(tesseract_scene_node_->getName());
  if (planning_scene_robot_)
    planning_scene_robot_.reset();
  planning_scene_monitor_.reset();
}

void TesseractSceneDisplay::clearJobs()
{
  background_process_.clear();
  {
    boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
    main_loop_jobs_.clear();
  }
}

void TesseractSceneDisplay::onInitialize()
{
  Display::onInitialize();

  // the scene node that contains everything
  tesseract_scene_node_ = scene_node_->createChildSceneNode();

  if (robot_category_)
  {
    planning_scene_robot_.reset(
        new RobotStateVisualization(tesseract_scene_node_, context_, "Planning Scene", robot_category_));
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
    changedRobotSceneAlpha();
    changedAttachedBodyColor();
  }
}

void TesseractSceneDisplay::reset()
{
  tesseract_scene_render_.reset();
  if (planning_scene_robot_)
    planning_scene_robot_->clear();

  addBackgroundJob(boost::bind(&TesseractSceneDisplay::loadRobotModel, this), "loadRobotModel");
  Display::reset();

  if (planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
  }
}

void TesseractSceneDisplay::addBackgroundJob(const boost::function<void()>& job, const std::string& name)
{
  background_process_.addJob(job, name);
}

void TesseractSceneDisplay::spawnBackgroundJob(const boost::function<void()>& job) { boost::thread t(job); }
void TesseractSceneDisplay::addMainLoopJob(const boost::function<void()>& job)
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  main_loop_jobs_.push_back(job);
}

void TesseractSceneDisplay::waitForAllMainLoopJobs()
{
  boost::unique_lock<boost::mutex> ulock(main_loop_jobs_lock_);
  while (!main_loop_jobs_.empty())
    main_loop_jobs_empty_condition_.wait(ulock);
}

void TesseractSceneDisplay::executeMainLoopJobs()
{
  main_loop_jobs_lock_.lock();
  while (!main_loop_jobs_.empty())
  {
    boost::function<void()> fn = main_loop_jobs_.front();
    main_loop_jobs_.pop_front();
    main_loop_jobs_lock_.unlock();
    try
    {
      fn();
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
    }
    main_loop_jobs_lock_.lock();
  }
  main_loop_jobs_empty_condition_.notify_all();
  main_loop_jobs_lock_.unlock();
}

const planning_scene_monitor::PlanningSceneMonitorPtr& TesseractSceneDisplay::getPlanningSceneMonitor()
{
  return planning_scene_monitor_;
}

const std::string TesseractSceneDisplay::getMoveGroupNS() const { return move_group_ns_property_->getStdString(); }
const robot_model::RobotModelConstPtr& TesseractSceneDisplay::getRobotModel() const
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->getRobotModel();
  else
  {
    static robot_model::RobotModelConstPtr empty;
    return empty;
  }
}

bool TesseractSceneDisplay::waitForCurrentRobotState(const ros::Time& t)
{
  if (planning_scene_monitor_)
    return planning_scene_monitor_->waitForCurrentRobotState(t);
  return false;
}

planning_scene_monitor::LockedPlanningSceneRO TesseractSceneDisplay::getPlanningSceneRO() const
{
  return planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
}

planning_scene_monitor::LockedPlanningSceneRW TesseractSceneDisplay::getPlanningSceneRW()
{
  return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
}

void TesseractSceneDisplay::changedAttachedBodyColor() { queueRenderSceneGeometry(); }
void TesseractSceneDisplay::changedSceneColor() { queueRenderSceneGeometry(); }
void TesseractSceneDisplay::changedMoveGroupNS()
{
  if (isEnabled())
    reset();
}

void TesseractSceneDisplay::changedRobotDescription()
{
  if (isEnabled())
    reset();
}

void TesseractSceneDisplay::changedSceneName()
{
  planning_scene_monitor::LockedPlanningSceneRW ps = getPlanningSceneRW();
  if (ps)
    ps->setName(scene_name_property_->getStdString());
}

void TesseractSceneDisplay::renderPlanningScene()
{
  if (tesseract_scene_render_ && tesseract_scene_needs_render_)
  {
    QColor color = scene_color_property_->getColor();
    rviz::Color env_color(color.redF(), color.greenF(), color.blueF());
    if (attached_body_color_property_)
      color = attached_body_color_property_->getColor();
    rviz::Color attached_color(color.redF(), color.greenF(), color.blueF());

    try
    {
      const planning_scene_monitor::LockedPlanningSceneRO& ps = getPlanningSceneRO();
      tesseract_scene_render_->render(ps,
                                      env_color,
                                      attached_color,
                                      static_cast<OctreeVoxelRenderMode>(octree_render_property_->getOptionInt()),
                                      static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt()),
                                      scene_alpha_property_->getFloat());
    }
    catch (std::exception& ex)
    {
      ROS_ERROR("Caught %s while rendering planning scene", ex.what());
    }
    tesseract_scene_needs_render_ = false;
    tesseract_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
  }
}

void TesseractSceneDisplay::changedSceneAlpha() { queueRenderSceneGeometry(); }
void TesseractSceneDisplay::changedRobotSceneAlpha()
{
  if (planning_scene_robot_)
  {
    planning_scene_robot_->setAlpha(robot_alpha_property_->getFloat());
    queueRenderSceneGeometry();
  }
}

void TesseractSceneDisplay::changedPlanningSceneTopic()
{
  if (planning_scene_monitor_ && planning_scene_topic_property_)
  {
    planning_scene_monitor_->startSceneMonitor(planning_scene_topic_property_->getStdString());
    std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
    if (!getMoveGroupNS().empty())
      service_name = ros::names::append(getMoveGroupNS(), service_name);
    planning_scene_monitor_->requestPlanningSceneState(service_name);
  }
}

void TesseractSceneDisplay::changedSceneDisplayTime() {}
void TesseractSceneDisplay::changedOctreeRenderMode() {}
void TesseractSceneDisplay::changedOctreeColorMode() {}
void TesseractSceneDisplay::changedSceneRobotVisualEnabled()
{
  if (isEnabled() && planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
  }
}

void TesseractSceneDisplay::changedSceneRobotCollisionEnabled()
{
  if (isEnabled() && planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
  }
}

void TesseractSceneDisplay::changedSceneEnabled()
{
  if (planning_scene_render_)
    planning_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());
}

void TesseractSceneDisplay::setGroupColor(Robot* robot, const std::string& group_name, const QColor& color)
{
  if (getRobotModel())
  {
    const robot_model::JointModelGroup* jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string>& links = jmg->getLinkModelNamesWithCollisionGeometry();
      for (std::size_t i = 0; i < links.size(); ++i)
        setLinkColor(robot, links[i], color);
    }
  }
}

void TesseractSceneDisplay::unsetAllColors(Robot* robot)
{
  if (getRobotModel())
  {
    const std::vector<std::string>& links = getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    for (std::size_t i = 0; i < links.size(); ++i)
      unsetLinkColor(robot, links[i]);
  }
}

void TesseractSceneDisplay::unsetGroupColor(rviz::Robot* robot, const std::string& group_name)
{
  if (getRobotModel())
  {
    const robot_model::JointModelGroup* jmg = getRobotModel()->getJointModelGroup(group_name);
    if (jmg)
    {
      const std::vector<std::string>& links = jmg->getLinkModelNamesWithCollisionGeometry();
      for (std::size_t i = 0; i < links.size(); ++i)
        unsetLinkColor(robot, links[i]);
    }
  }
}

void TesseractSceneDisplay::setLinkColor(const std::string& link_name, const QColor& color)
{
  if (planning_scene_robot_)
    setLinkColor(&planning_scene_robot_->getRobot(), link_name, color);
}

void TesseractSceneDisplay::unsetLinkColor(const std::string& link_name)
{
  if (planning_scene_robot_)
    unsetLinkColor(&planning_scene_robot_->getRobot(), link_name);
}

void TesseractSceneDisplay::setLinkColor(Robot* robot, const std::string& link_name, const QColor& color)
{
  RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor(color.redF(), color.greenF(), color.blueF());
}

void TesseractSceneDisplay::unsetLinkColor(Robot* robot, const std::string& link_name)
{
  RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
planning_scene_monitor::PlanningSceneMonitorPtr TesseractSceneDisplay::createPlanningSceneMonitor()
{
  return planning_scene_monitor::PlanningSceneMonitorPtr(
      new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString(),
                                                       context_->getFrameManager()->getTFClientPtr(),
                                                       getNameStd() + "_planning_scene_monitor"));
}

void TesseractSceneDisplay::clearRobotModel()
{
  tesseract_scene_render_.reset();
  planning_scene_monitor_.reset();  // this so that the destructor of the
                                    // PlanningSceneMonitor gets called before a
                                    // new
                                    // instance of a scene monitor is constructed
}

void TesseractSceneDisplay::loadRobotModel()
{
  // wait for other robot loadRobotModel() calls to complete;
  boost::mutex::scoped_lock _(robot_model_loading_lock_);
  model_is_loading_ = true;

  // we need to make sure the clearing of the robot model is in the main thread,
  // so that rendering operations do not have data removed from underneath,
  // so the next function is executed in the main loop
  addMainLoopJob(boost::bind(&TesseractSceneDisplay::clearRobotModel, this));

  waitForAllMainLoopJobs();

  planning_scene_monitor::PlanningSceneMonitorPtr psm = createPlanningSceneMonitor();
  if (psm->getPlanningScene())
  {
    planning_scene_monitor_.swap(psm);
    addMainLoopJob(boost::bind(&TesseractSceneDisplay::onRobotModelLoaded, this));
    setStatus(rviz::StatusProperty::Ok, "PlanningScene", "Planning Scene Loaded Successfully");
    waitForAllMainLoopJobs();
  }
  else
  {
    setStatus(rviz::StatusProperty::Error, "PlanningScene", "No Planning Scene Loaded");
  }

  if (planning_scene_monitor_)
    planning_scene_monitor_->addUpdateCallback(
        boost::bind(&TesseractSceneDisplay::sceneMonitorReceivedUpdate, this, _1));

  model_is_loading_ = false;
}

void TesseractSceneDisplay::onRobotModelLoaded()
{
  changedPlanningSceneTopic();
  tesseract_scene_render_.reset(new PlanningSceneRender(tesseract_scene_node_, context_, planning_scene_robot_));
  tesseract_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

  const planning_scene_monitor::LockedPlanningSceneRO& ps = getPlanningSceneRO();
  if (planning_scene_robot_)
  {
    planning_scene_robot_->load(*getRobotModel()->getURDF());
    robot_state::RobotState* rs = new robot_state::RobotState(ps->getCurrentState());
    rs->update();
    planning_scene_robot_->update(robot_state::RobotStateConstPtr(rs));
  }

  bool oldState = scene_name_property_->blockSignals(true);
  scene_name_property_->setStdString(ps->getName());
  scene_name_property_->blockSignals(oldState);
}

void TesseractSceneDisplay::sceneMonitorReceivedUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  onSceneMonitorReceivedUpdate(update_type);
}

void TesseractSceneDisplay::onSceneMonitorReceivedUpdate(
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
{
  bool oldState = scene_name_property_->blockSignals(true);
  getPlanningSceneRW()->getCurrentStateNonConst().update();
  scene_name_property_->setStdString(getPlanningSceneRO()->getName());
  scene_name_property_->blockSignals(oldState);

  planning_scene_needs_render_ = true;
}

void TesseractSceneDisplay::onEnable()
{
  Display::onEnable();

  addBackgroundJob(boost::bind(&TesseractSceneDisplay::loadRobotModel, this), "loadRobotModel");

  if (planning_scene_robot_)
  {
    planning_scene_robot_->setVisible(true);
    planning_scene_robot_->setVisualVisible(scene_robot_visual_enabled_property_->getBool());
    planning_scene_robot_->setCollisionVisible(scene_robot_collision_enabled_property_->getBool());
  }
  if (tesseract_scene_render_)
    tesseract_scene_render_->getGeometryNode()->setVisible(scene_enabled_property_->getBool());

  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void TesseractSceneDisplay::onDisable()
{
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->stopSceneMonitor();
    if (tesseract_scene_render_)
      tesseract_scene_render_->getGeometryNode()->setVisible(false);
  }
  if (planning_scene_robot_)
    planning_scene_robot_->setVisible(false);
  Display::onDisable();
}

void TesseractSceneDisplay::queueRenderSceneGeometry() { tesseract_scene_needs_render_ = true; }
void TesseractSceneDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  executeMainLoopJobs();

  if (planning_scene_monitor_)
    updateInternal(wall_dt, ros_dt);
}

void TesseractSceneDisplay::updateInternal(float wall_dt, float ros_dt)
{
  current_scene_time_ += wall_dt;
  if (current_scene_time_ > scene_display_time_property_->getFloat())
  {
    renderPlanningScene();
    calculateOffsetPosition();
    current_scene_time_ = 0.0f;
  }
}

void TesseractSceneDisplay::load(const rviz::Config& config) { Display::load(config); }
void TesseractSceneDisplay::save(rviz::Config config) const { Display::save(config); }
// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void TesseractSceneDisplay::calculateOffsetPosition()
{
  if (!getRobotModel())
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(getRobotModel()->getModelFrame(), ros::Time(0), position, orientation);

  tesseract_scene_node_->setPosition(position);
  tesseract_scene_node_->setOrientation(orientation);
}

void TesseractSceneDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}

}  // namespace tesseract_rviz
