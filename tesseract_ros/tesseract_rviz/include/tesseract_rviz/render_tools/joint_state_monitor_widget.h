#ifndef TESSERACT_RVIZ_STATE_MONITORING_H
#define TESSERACT_RVIZ_STATE_MONITORING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rviz_common/display.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifndef Q_MOC_RUN
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/JointState.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>
#endif

#include <tesseract_rviz/render_tools/visualization_widget.h>

namespace rviz_common::properties
{
class Property;
class RosTopicProperty;
}  // namespace rviz

namespace tesseract_rviz
{
class JointStateMonitorWidget : public QObject
{
  Q_OBJECT

public:
  using Ptr = std::shared_ptr<JointStateMonitorWidget>;
  using ConstPtr = std::shared_ptr<const JointStateMonitorWidget>;

  JointStateMonitorWidget(rviz_common::properties::Property* widget, rviz::Display* display);

  virtual ~JointStateMonitorWidget();

  void onInitialize(VisualizationWidget::Ptr visualization,
                    tesseract::Tesseract::Ptr tesseract,
                    rviz_common::DisplayContext* context,
                    ros::NodeHandle update_nh);

  void onEnable();
  void onDisable();
  void onUpdate();
  void onReset();

private Q_SLOTS:
  void changedJointStateTopic();

protected:
  rviz_common::properties::Property* widget_;
  rviz::Display* display_;
  VisualizationWidget::Ptr visualization_;
  tesseract::Tesseract::Ptr tesseract_;
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_subscriber_;
  bool update_required_;

  rviz_common::properties::Property* main_property_;
  rviz::RosTopicProperty* joint_state_topic_property_;

  void newJointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);

  bool isUpdateRequired(const sensor_msgs::JointState& joint_state);
};
}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_STATE_MONITORING_H
