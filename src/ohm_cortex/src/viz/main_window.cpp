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

#include <QFileDialog>

#include "Node.h"
#include "Link.h"

#include "ohm_cortex/Force.h"


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent),
	  _ui(new Ui::MainWindow),
	  _scene(new QGraphicsScene(0, 0, 600, 500))
{
	_ui->setupUi(this);

	_ui->graphicsView->setScene(_scene);

	this->slot_update();

	for(unsigned int i=0 ; i<_states.size() ; i++)
	{
	    Node* n= new Node(QPointF(40.0 * i, 40.0));
	    n->setText(_states[i].getName());
	   _nodes.push_back(n);
      _scene->addItem(n);
	}

	for(unsigned int i=0 ; i<_states.size() ; i++) {
	   for(unsigned int k=0 ; k<_states[i].getLinks().size() ; k++) {
	      Link* l = new Link(_nodes[i], _nodes[_states[i].getLinks()[k].getId()]);
	      _scene->addItem(l);
	   }
	}

	_state_sub       =  _nh.subscribe("/ohm_statemachine_node/state", 1, &MainWindow::stateCallback, this);
	_state_force_clt =  _nh.serviceClient<ohm_cortex::Force>("/ohm_statemachine_node/force_state");

   connect(&_timer,           SIGNAL(timeout()),   this, SLOT(slot_tick()));
   connect(_ui->actionUpdate, SIGNAL(triggered()), this, SLOT(slot_update()));

   // connect all checkboxes to the same slot
   Q_FOREACH(Node* node, _nodes) {
       connect(node, SIGNAL(force(QString)), this, SLOT(slot_force(QString)));
   }

   _timer.start(100);


}

MainWindow::~MainWindow()
{
   // nothing to do
}

void MainWindow::stateCallback(std_msgs::String::ConstPtr msg)
{
   std::string str = msg->data;
   QString state = QString::fromStdString(str);

   qDebug() << "state: " << state;

   for(unsigned int i=0 ; i<_nodes.size() ; i++)
   {
      qDebug() << "node: " << _nodes[i]->getText() << " state: " << state; ;
      if(_nodes[i]->getText().toLower() == state.toLower()) _nodes[i]->setBackgroundColor(Qt::red);
      else                                                  _nodes[i]->setBackgroundColor(Qt::white);
   }

   _scene->update();
}

void MainWindow::slot_tick(void)
{
    ros::spinOnce();
}

void MainWindow::slot_update(void)
{
   QDir includeDir;
   QString path = QFileDialog::getExistingDirectory (this, tr("Directory"), includeDir.path());

   // check if valid folder was selected
   if ( path.isNull() == false ) includeDir.setPath(path);

   QStringList files = includeDir.entryList().filter("cpp");

   /*
    * For every file we gernate a state
    */
   for(unsigned int i=0; i<files.size() ; ++i)
   {
      StateViz s;
      s.setName(files.at(i));
      s.setId(i);
      _states.push_back(s);
   }

   for (int i = 0; i < files.size(); ++i)
   {
      QString fileName = files.at(i);
      QFile f(QString(path + "/" + fileName));

      // error handling if file does not exist
      if(!f.exists()) { qDebug() << "file does not exist"; return; }

      f.open(QIODevice::ReadOnly | QFile::Text);
      QTextStream in(&f);
      QString text(in.readAll());

      for(unsigned int k=0 ; k<files.size(); ++k) {
         QString header       = files.at(k);
         QString searchHeader = header.replace(".cpp", ".h");
         if(text.contains(QString(searchHeader))) {
            if(searchHeader != fileName.replace(".cpp", ".h")) {
               _states[i].addLink(_states[k]);
            }
         }
      }
      f.close();
   }

//   /*
//    * Debug information
//    */
//   for(unsigned int i=0; i<_states.size() ; i++)   {
//      qDebug() << _states[i].getName() << "\t size: " << _states[i].getLinks().size();
//      for(unsigned int k=0 ; k<_states[i].getLinks().size() ; k++){
//         qDebug() << "--> " << _states[i].getLinks()[k].getName();
//      }
//   }
}


void MainWindow::slot_force(QString state)
{
   qDebug() << __PRETTY_FUNCTION__;

   ohm_cortex::Force srv;
   srv.request.state = state.toLower().toStdString();


   if(_state_force_clt.call(srv))
   {
      // everything ok
   }
   else
   {
      ROS_ERROR("Failed to call service");
   }
}



