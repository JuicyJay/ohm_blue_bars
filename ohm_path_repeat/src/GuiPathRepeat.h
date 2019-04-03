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
#include "ui_path_repeat.h"

namespace ohm_autonomy
{

class GuiPathRepeat : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  GuiPathRepeat();
  virtual ~GuiPathRepeat();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
public slots:
  void setEndPoint(void);
  void startPathRepeat(void);
  void pauseUnPause(bool checked);
private:
  ros::NodeHandle _nh;
  ros::Publisher _pubStartPathRepeat;
  ros::Publisher _pubSetEndMarker;
  QWidget* _widgetMain;
  Ui::PathRepeatUi* _guiUi;
  ros::ServiceClient _clientMoveCtrl;
};

} /* namespace ohm_autonomy */

#endif /* OHM_PATH_REPEAT_SRC_GUIPATHREPEAT_H_ */
