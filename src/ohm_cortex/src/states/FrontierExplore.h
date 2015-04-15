/*
 * FrontierExplore.h
 *
 *  Created on: 30.01.2015
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_
#define OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_

#include "../IState.h"

#include "geometry_msgs/PoseArray.h"

/**
 * @namespace  autonohm
 */
namespace autonohm
{

/**
 * @author Michael Schmidpeter
 *
 * @brief Singelton class for Data Stuff... e.g. Publisher and Subscriber
 */
class FrontierExploreModel
{
public:

   static FrontierExploreModel* getInstance()
   {
      if(!_instance)
         _instance = new FrontierExploreModel();
      return _instance;
   }

   ~FrontierExploreModel() { }

private:
   static FrontierExploreModel* _instance;
   //ros
   ros::Subscriber _sub_frontier;

   //data
   geometry_msgs::PoseArray _frontiers;


   FrontierExploreModel()
   {

   }
   FrontierExploreModel(FrontierExploreModel& fmodel) { }


};


/**
 * @class   FrontierExplore
 * @author  Christian Pfitzner, Michael Schmidpeter
 * @date    2015-01-30
 *
 * @brief   State class for
 */
class FrontierExplore: IState
{
public:
   /**
    * Default constructor
    */
   FrontierExplore(void);
   /**
    * Default destructor
    */
   virtual ~FrontierExplore(void);

   // PROCESSING
   /**
    * Function for processing
    */
   void process(void);

private:
   ros::NodeHandle* _nh;
   ros::Publisher _state_pub;

   ros::ServiceClient _target_srv_client;
};

} /* namespace autonohm */

class FrontierExplore
{
public:
   FrontierExplore();
   virtual ~FrontierExplore();
};
#endif /* OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_ */

