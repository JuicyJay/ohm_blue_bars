#include "TestFrontier.h"

namespace autonohm
{

TestFrontier::TestFrontier(): _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("new State TestFronteir");
   _subInt = _nh->subscribe("frontier/test", 1, &TestFrontier::subInt_callback, this);
   _rdy = false;
}

TestFrontier::~TestFrontier()
{

}

void TestFrontier::process(void)
{
   _rdy = true;
   //do nothing
}

void TestFrontier::subInt_callback(const std_msgs::Int32& msg)
{
   if(!_rdy)
      return;

   ROS_INFO("TRIGGER Frontier: %d",msg.data);

   Context::getInstance()->setState(new FrontierExplore(msg.data, new TestFrontier));
   delete this;
}

} /* namespace autonohm */
