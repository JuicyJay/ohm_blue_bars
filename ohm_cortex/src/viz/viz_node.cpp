 // ros includes
#include <ros/ros.h>

// qt includes
#include <QtGui>
#include <QApplication>
#include "main_window.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_viz_node");
   ros::NodeHandle nh("~");

   QApplication app(argc, argv);
   MainWindow w;
   w.setNodeHandle(nh);
   w.show();

   app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
   int result = app.exec();

	return result;
}
