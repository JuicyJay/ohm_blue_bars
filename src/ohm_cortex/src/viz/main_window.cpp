/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "main_window.hpp"

#include "../../ui_main_window.h"

#include "Node.h"
#include "Link.h"


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent),
	  _ui(new Ui::MainWindow),
	  _scene(new QGraphicsScene(0, 0, 600, 500))
{
	_ui->setupUi(this);


	_ui->graphicsView->setScene(_scene);

	node1 = new Node(QPointF(40.0, 40.0));
	node2 = new Node(QPointF(40.0, 80.0));

   node1->setText("ping");
	node2->setText("pong");

	Link* link = new Link(node1, node2);
//
	_scene->addItem(node1);
	_scene->addItem(node2);
	_scene->addItem(link);

	_state_sub =  _nh.subscribe("/ohm_statemachine_node/state", 20, &MainWindow::stateCallback, this);

	qDebug() << __PRETTY_FUNCTION__;


   this->connect(&_timer, SIGNAL(timeout()), this, SLOT(tick()));
   _timer.start(10);


//	while(ros::ok())
//	{
//	    ros::spinOnce();
//	}

}

MainWindow::~MainWindow()
{

}

void MainWindow::stateCallback(std_msgs::String::ConstPtr msg)
{
   qDebug() << __PRETTY_FUNCTION__;

   if(msg->data == "ping")
   {
      node1->setBackgroundColor(Qt::red);
      node2->setBackgroundColor(Qt::blue);
   }
   else
   {
      node1->setBackgroundColor(Qt::blue);
      node2->setBackgroundColor(Qt::red);
   }
   _scene->update();

}

void MainWindow::tick(void)
{
   _ui->graphicsView->update();
    ros::spinOnce();
}



