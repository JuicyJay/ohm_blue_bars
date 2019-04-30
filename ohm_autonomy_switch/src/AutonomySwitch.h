/*
 * GuiPathRepeat.h
 *
 *  Created on: Apr 1, 2019
 *      Author: phil
 */

#ifndef OHM_PATH_REPEAT_SRC_GUIPATHREPEAT_H_
#define OHM_PATH_REPEAT_SRC_GUIPATHREPEAT_H_

#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <QtWidgets/QWidget>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include "ui_autonomy_switch.h"

namespace ohm_autonomy
{

class AutonomySwitch : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  AutonomySwitch();
  virtual ~AutonomySwitch();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
public slots:
 void engageAutonomy(bool onOff);
private:
 void callBackJoy(const sensor_msgs::Joy& joy);
 void callBackVelAutonoy(const geometry_msgs::Twist& vel);
 void callBackVelTeleop(const geometry_msgs::Twist& vel);
 void callbackTimer(const ros::TimerEvent&);
 void activateOverride(void);
 void activateAutonomy(void);
 bool checkMovement(const sensor_msgs::Joy& msg);
  ros::NodeHandle _nh;
  QWidget* _widgetMain;
  Ui::AutonomySwitchGui* _guiUi;
  ros::Publisher _pubVel;
  ros::Subscriber _subsVelAutonomy;
  geometry_msgs::Twist _twistAutonomy;
  ros::Subscriber _subsVelTeleop;
  geometry_msgs::Twist _twistTeleop;
  ros::Time _lastSignalTeleop;
  ros::Subscriber _subsJoy;
  ros::Timer _timer;
  double _timeoutThresh;
  bool _autonomous;
};

} /* namespace ohm_autonomy */

#endif /* OHM_PATH_REPEAT_SRC_GUIPATHREPEAT_H_ */
