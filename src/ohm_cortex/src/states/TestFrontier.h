
#ifndef OHM_AUTONOMY_SRC_OHM_CORTEX_SRC_STATES_TESTFRONTIER_H_
#define OHM_AUTONOMY_SRC_OHM_CORTEX_SRC_STATES_TESTFRONTIER_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "../IState.h"
#include "../Context.h"

#include "FrontierExplore.h"

namespace autonohm
{

class TestFrontier: public IState
{
public:
   TestFrontier();
   virtual ~TestFrontier();

   void process(void);

private:
   ros::NodeHandle* _nh;
   ros::Subscriber _subInt;

   bool _rdy;

   void subInt_callback(const std_msgs::Int32& msg);

};

} /* namespace autonohm */

#endif /* OHM_AUTONOMY_SRC_OHM_CORTEX_SRC_STATES_TESTFRONTIER_H_ */
