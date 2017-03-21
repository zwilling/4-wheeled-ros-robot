/*
 * gui_plugin.cpp
 *
 *  Created on: 20.03.2017
 *      Author: zwilling
 */

#include "gui_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <stdio.h>

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
}

void GuiPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void GuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void GuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

} /* namespace qbot_4wheeled_robot */

PLUGINLIB_DECLARE_CLASS(qbot_4wheeled_robot, GuiPlugin, qbot_4wheeled_robot::GuiPlugin, rqt_gui_cpp::Plugin)
