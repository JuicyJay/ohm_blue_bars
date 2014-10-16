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
   void tick(void);

private:
   ros::Subscriber _state_sub;

   Ui::MainWindow*      _ui;
   QGraphicsScene*      _scene;

   ros::NodeHandle      _nh;

   Node* node1;
   Node* node2;


   QTimer _timer;
};




#endif /* OHM_CORTEX_SRC_VIZ_MAIN_WINDOW_HPP_ */
