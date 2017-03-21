/*
 * gui_plugin.h
 *
 *  Created on: 20.03.2017
 *      Author: zwilling
 */

#ifndef SRC_RQT_QBOT_GUI_PLUGIN_H_
#define SRC_RQT_QBOT_GUI_PLUGIN_H_

#include <rqt_gui_cpp/plugin.h>
#include <ui_gui_plugin.h>
#include <QWidget>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace qbot_4wheeled_robot {

/**
 * Gui Plugin for sending a navigation command
 */
class GuiPlugin : public rqt_gui_cpp::Plugin
{
	Q_OBJECT
public:
	GuiPlugin();
	virtual ~GuiPlugin();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);
	virtual void shutdownPlugin();
	virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
	virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
	Ui::GuiPlugin ui_;
	QWidget* widget_;
	ros::Publisher navigation_pub_;

protected slots:
	//callback function for button push
	virtual void on_start_button_push();
};

} /* namespace qbot_4wheeled_robot */

#endif /* SRC_RQT_QBOT_GUI_PLUGIN_H_ */
