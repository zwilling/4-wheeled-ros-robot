/*
 * gui_plugin.cpp
 *
 *  Created on: 20.03.2017
 *      Author: zwilling
 */

#include "gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <QSpinBox>
#include <tf/transform_datatypes.h>

namespace qbot_4wheeled_robot {

  GuiPlugin::GuiPlugin() : rqt_gui_cpp::Plugin(), widget_(0)
  {
    // give QObjects reasonable names
    setObjectName("QBotGUI");
  }

  GuiPlugin::~GuiPlugin()
  {}

  void GuiPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    //connect button push callback
    connect(ui_.start_button, SIGNAL(pressed()), this, SLOT(on_start_button_push()));

    //init publisher
    navigation_pub_ = getNodeHandle().advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  }

  void GuiPlugin::shutdownPlugin()
  {
    //clean up
    navigation_pub_.shutdown();
  }

  void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
  {
    // nothing to save up to now
  }

  void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
  {
    // nothing to restore up to now

  }

  void GuiPlugin::on_start_button_push()
  {
    //button was pushed
    //build navigation goal message from values in SpinBoxes
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = ui_.doubleSpinBox_x->value();
    msg.pose.position.y = ui_.doubleSpinBox_y->value();
    //get Quaternion from yaw to write it into the msg
    tf::Quaternion q;
    q.setRPY(0.0 , 0.0, ui_.doubleSpinBox_ori->value());
    tf::quaternionTFToMsg(q, msg.pose.orientation);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    //send
    navigation_pub_.publish(msg);
  }

} /* namespace qbot_4wheeled_robot */

PLUGINLIB_DECLARE_CLASS(qbot_4wheeled_robot, GuiPlugin, qbot_4wheeled_robot::GuiPlugin, rqt_gui_cpp::Plugin)
