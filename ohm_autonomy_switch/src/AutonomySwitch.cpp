/*
 * AutonomySwitch.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: phil
 */

#include "AutonomySwitch.h"
#include <pluginlib/class_list_macros.h>
#include <QtCore/QStringList>

#define MOVE_THRESH 0.05

namespace ohm_autonomy
{

AutonomySwitch::AutonomySwitch():
        _autonomous(false),
        _timeoutThresh(0.0),
        _lastSignalTeleop(ros::Time::now())
{

}
AutonomySwitch::~AutonomySwitch()
{

}

void AutonomySwitch::initPlugin(qt_gui_cpp::PluginContext& context)
{
  QStringList argv = context.argv();
  _widgetMain = new QWidget;
  _guiUi = new Ui::AutonomySwitchGui;
  _guiUi->setupUi(_widgetMain);
  context.addWidget(_widgetMain);
  std::string topicVelAutonomy;
  std::string topicVelTeleop;
  std::string topicJoy;
  std::string topicVelOut;
  double rate = 0.0;


  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("topic_vel_autonomy", topicVelAutonomy, "vel/autonomy");
  prvNh.param<std::string>("topic_vel_teleop",   topicVelTeleop, "vel/teleop");
  prvNh.param<std::string>("topic_joy", topicJoy, "joy");
  prvNh.param<std::string>("topic_vel_out", topicVelOut, "/robot0/cmd_vel");
  prvNh.param<double>("rate", rate, 20.0);
  prvNh.param<double>("timeout_joy_input", _timeoutThresh, 0.5);

  _subsVelAutonomy = _nh.subscribe(topicVelAutonomy, 1, &AutonomySwitch::callBackVelAutonoy, this);
  _subsVelTeleop   = _nh.subscribe(topicVelTeleop,   1, &AutonomySwitch::callBackVelTeleop,  this);
  _subsJoy         = _nh.subscribe(topicJoy,         1, &AutonomySwitch::callBackJoy,        this);

  _pubVel = _nh.advertise<geometry_msgs::Twist>(topicVelOut, 1);

  connect(_guiUi->pushButton, SIGNAL(toggled(bool)), this, SLOT(engageAutonomy(bool)));
  _timer = _nh.createTimer(ros::Duration(1.0 / rate), &AutonomySwitch::callbackTimer, this);
  //_widgetMain->show();
}

void AutonomySwitch::shutdownPlugin()
{

}
void AutonomySwitch::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{

}

void AutonomySwitch::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{

}

void AutonomySwitch::engageAutonomy(bool onOff)
{
 if(onOff)
   this->activateAutonomy();
 else
   this->activateOverride();
}

void AutonomySwitch::callBackJoy(const sensor_msgs::Joy& joy)
{
  if(this->checkMovement(joy))
    this->activateOverride();
  _lastSignalTeleop = ros::Time::now();
}

void AutonomySwitch::callBackVelAutonoy(const geometry_msgs::Twist& vel)
{
  _twistAutonomy = vel;
}

void AutonomySwitch::callBackVelTeleop(const geometry_msgs::Twist& vel)
{
  _twistTeleop = vel;
}

void AutonomySwitch::activateOverride(void)
{
  _autonomous = false;
  _guiUi->pushButton->setChecked(false);
  std::cout << __PRETTY_FUNCTION__ << " disengage autonomy " << std::endl;
}

void AutonomySwitch::activateAutonomy(void)
{
  _autonomous = true;
  std::cout << __PRETTY_FUNCTION__ << " engage autonomy " << std::endl;
}

void AutonomySwitch::callbackTimer(const ros::TimerEvent&)
{
  if((ros::Time::now() - _lastSignalTeleop).toSec() > _timeoutThresh)
  {
    if(_autonomous)
      this->activateOverride();
  }
  geometry_msgs::Twist vel;
  if(_autonomous)
  {
    vel = _twistAutonomy;
  }
  else
  {
    vel = _twistTeleop;
  }
  _pubVel.publish(vel);
}

bool AutonomySwitch::checkMovement(const sensor_msgs::Joy& msg)
{
  static sensor_msgs::Joy joyLast = msg;
  for(unsigned int i = 0; i < msg.axes.size(); i++)
    if((std::abs(msg.axes[i]) - std::abs(joyLast.axes[i])) > MOVE_THRESH)
      return true;

  for(unsigned int i = 0; i < msg.buttons.size(); i++)
    if(msg.buttons[i] && !joyLast.buttons[i])
      return true;

  return false;
}

}
PLUGINLIB_EXPORT_CLASS(ohm_autonomy::AutonomySwitch, rqt_gui_cpp::Plugin)
