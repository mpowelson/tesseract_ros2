/**
 * @file manipulation_widget.h
 * @brief A manipulators widget for moving the robot around in rviz.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_RVIZ_MANIPULATION_WIDGET_H
#define TESSERACT_RVIZ_MANIPULATION_WIDGET_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
#include <rviz_common//panel_dock_widget.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract/tesseract.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread/mutex.hpp>
#include <tesseract_kinematics/core/inverse_kinematics.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#endif

#include <tesseract_rviz/property/button_property.h>
#include <tesseract_rviz/render_tools/visualization_widget.h>
#include <tesseract_rviz/interactive_marker/interactive_marker.h>

namespace rviz
{
class Property;
class RosTopicProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class StringProperty;
}  // namespace rviz

namespace Ogre
{
class SceneNode;
}

namespace tesseract_rviz
{
class ManipulationWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<ManipulationWidget>;
  using ConstSharedPtr = std::shared_ptr<const ManipulationWidget>;

  enum class ManipulatorState
  {
    START = 0,
    END = 1
  };

  ManipulationWidget(rviz_common::properties::Property* widget, rviz_common::Display* display);

  virtual ~ManipulationWidget();

  void onInitialize(Ogre::SceneNode* root_node,
                    rviz_common::DisplayContext* context,
                    VisualizationWidget::Ptr visualization,
                    tesseract::Tesseract::Ptr tesseract,
                    ros::NodeHandle update_nh,
                    ManipulatorState state,
                    QString joint_state_topic);

  void onEnable();
  void onDisable();
  void onUpdate(float wall_dt);
  void onReset();

  void onNameChange(const QString& name);

Q_SIGNALS:
  void availableManipulatorsChanged(QStringList manipulators);
  void availableTCPLinksChanged(QStringList tcp_links);

public
  Q_SLOT : void enableCartesianManipulation(bool enabled);
  void enableJointManipulation(bool enabled);
  void resetToCurrentState();
  bool changeManipulator(QString manipulator);
  bool changeTCP(QString tcp_link);

private Q_SLOTS:
  void changedManipulator();
  void changedTCP();
  void changedJointStateTopic();
  void changedCartesianMarkerScale();
  void changedCartesianManipulationEnabled();
  void changedJointMarkerScale();
  void changedJointManipulationEnabled();
  void clickedResetToCurrentState();
  void markerFeedback(std::string reference_frame,
                      Eigen::Isometry3d transform,
                      Eigen::Vector3d mouse_point,
                      bool mouse_point_valid);
  void jointMarkerFeedback(std::string joint_name,
                           std::string reference_frame,
                           Eigen::Isometry3d transform,
                           Eigen::Vector3d mouse_point,
                           bool mouse_point_valid);
  //  void trajectorySliderPanelVisibilityChange(bool enable);

protected:
  Ogre::SceneNode* root_interactive_node_;
  rviz_common::properties::Property* widget_;
  rviz_common::Display* display_;
  rviz_common::DisplayContext* context_;
  VisualizationWidget::Ptr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
  ros::NodeHandle nh_;
  ManipulatorState state_;
  InteractiveMarker::Ptr interactive_marker_;
  std::map<std::string, InteractiveMarker::Ptr> joint_interactive_markers_;
  std::vector<std::string> manipulators_;
  tesseract_kinematics::InverseKinematics::Ptr inv_kin_;
  Eigen::VectorXd inv_seed_;
  int env_revision_;
  std::unordered_map<std::string, double> joints_;
  tesseract_environment::EnvState::Ptr env_state_;
  Eigen::Isometry3d tcp_;

  ros::Publisher joint_state_pub_;
  QStringList available_manipulators_;
  QStringList available_tcp_links_;

  //  TrajectoryPanel* trajectory_slider_panel_;
  //  rviz::PanelDockWidget* trajectory_slider_dock_panel_;

  // Properties
  bool enabled_;
  ButtonProperty* main_property_;
  rviz_common::properties::EnumProperty* manipulator_property_;
  rviz_common::properties::RosTopicProperty* joint_state_topic_property_;
  rviz_common::properties::BoolProperty* cartesian_manipulation_property_;
  rviz_common::properties::BoolProperty* joint_manipulation_property_;
  rviz_common::properties::FloatProperty* cartesian_marker_scale_property_;
  rviz_common::properties::FloatProperty* joint_marker_scale_property_;
  rviz_common::properties::EnumProperty* tcp_property_;
  rviz_common::properties::Property* joint_values_property_;
};
}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_MANIPULATION_WIDGET_H
