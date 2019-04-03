/*"
 * GuiPathRepeat.cpp
 *
 *  Created on: Apr 1, 2019
 *      Author: phil
 */

#include "GuiPathRepeat.h"
#include <string>
#include <std_msgs/Bool.h>
#include <pluginlib/class_list_macros.h>
#include <rona_msgs/NodeCtrlSRV.h>

namespace ohm_autonomy
{

GuiPathRepeat::GuiPathRepeat():
    _widgetMain(NULL),
    _guiUi(NULL)
{
  setObjectName("PathRepeatGui");

}

GuiPathRepeat::~GuiPathRepeat()
{
  // TODO Auto-generated destructor stub
}

void GuiPathRepeat::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ros::NodeHandle prvNh("~");
  std::string topicSetEndMarker;
  std::string topicStartPathRepeat;
  std::string topicRonaCtrl;
  prvNh.param<std::string>("topic_set_end_marker", topicSetEndMarker, "/rona/set_end_point");
  prvNh.param<std::string>("topic_start_path_repeat", topicStartPathRepeat, "/rona/start_repeat");
  prvNh.param<std::string>("topic_rona_ctrl", topicRonaCtrl, "/rona/move/node_ctrl");
  _pubSetEndMarker = _nh.advertise<std_msgs::Bool>(topicSetEndMarker, 1);
  _pubStartPathRepeat = _nh.advertise<std_msgs::Bool>(topicStartPathRepeat, 1);
  _clientMoveCtrl = _nh.serviceClient<rona_msgs::NodeCtrlSRV>(topicRonaCtrl);
  _widgetMain = new QWidget;
  _guiUi = new Ui::PathRepeatUi;
  _guiUi->setupUi(_widgetMain);
  context.addWidget(_widgetMain);

  connect(_guiUi->pushButtonSetEndMarker, SIGNAL(clicked()), this, SLOT(setEndPoint()));
  connect(_guiUi->pushButtonStartPathRepeat, SIGNAL(clicked()), this, SLOT(startPathRepeat()));
  connect(_guiUi->pushButtonPauseStart, SIGNAL(toggled(bool)), this, SLOT(pauseUnPause(bool)));
}

void GuiPathRepeat::shutdownPlugin()
{
  _pubSetEndMarker.shutdown();
  _pubStartPathRepeat.shutdown();
  _nh.shutdown();
}

void GuiPathRepeat::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{

}

void GuiPathRepeat::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{

}

void GuiPathRepeat::setEndPoint(void)
{
  std_msgs::Bool setEndPoint;
  setEndPoint.data = true;
  _pubSetEndMarker.publish(setEndPoint);
}

void GuiPathRepeat::startPathRepeat(void)
{
  std_msgs::Bool startRepeating;
    startRepeating.data = true;
    _pubStartPathRepeat.publish(startRepeating);
}

void GuiPathRepeat::pauseUnPause(bool checked)
{
  std::cout << __PRETTY_FUNCTION__ << "" << std::endl;
  rona_msgs::NodeCtrlSRV ctrlService;
  if(!checked)
    ctrlService.request.ctrl.cmd = rona_msgs::NodeCtrl::PAUSE;
  else
    ctrlService.request.ctrl.cmd = rona_msgs::NodeCtrl::CONTINUE;
  if(!_clientMoveCtrl.call(ctrlService))
    std::cout << __PRETTY_FUNCTION__ << " error calling service" << std::endl;
}

} /* namespace ohm_autonomy */
PLUGINLIB_EXPORT_CLASS(ohm_autonomy::GuiPathRepeat, rqt_gui_cpp::Plugin)
