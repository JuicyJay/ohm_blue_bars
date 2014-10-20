/*
 * main_window.hpp
 *
 *  Created on: 16.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_VIZ_MAIN_WINDOW_HPP_
#define OHM_CORTEX_SRC_VIZ_MAIN_WINDOW_HPP_


#include <QMainWindow>
#include <QGraphicsView>
#include <QTimer>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "Node.h"

#include "StateViz.h"


namespace Ui{
   class MainWindow;
}

class MainWindow : public QMainWindow
{
   Q_OBJECT
public:
   MainWindow(QWidget* parent = 0);
   virtual ~MainWindow(void);
   void setNodeHandle(ros::NodeHandle nh) { _nh = nh; }


   void stateCallback(std_msgs::String::ConstPtr msg);

private slots:
   void slot_tick(void);
   void slot_update(void);
   void slot_force(QString state);

private:
   ros::Subscriber      _state_sub;
   ros::ServiceClient   _state_force_clt;

   Ui::MainWindow*      _ui;
   QGraphicsScene*      _scene;

   ros::NodeHandle      _nh;

   QVector<Node*>       _nodes;
   QList<StateViz>      _states;

   QTimer _timer;
};




#endif /* OHM_CORTEX_SRC_VIZ_MAIN_WINDOW_HPP_ */
