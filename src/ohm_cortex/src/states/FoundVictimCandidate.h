#ifndef ___FOUND_VICTIM_CANDIDATE_H___
#define ___FOUND_VICTIM_CANDIDATE_H___

#include "../IState.h"

#include <geometry_msgs/Point.h>
#include <ohm_perception/Victim.h>

/**
 * @namespace autonohm
 */
namespace autonohm {

class FoundVictimCandidate : public IState
{
public:
    /**
     * Default constructor
     */
    FoundVictimCandidate(const geometry_msgs::Point& goal);
    /**
     * Default destructor
     */
    virtual ~FoundVictimCandidate(void);

    /**
     * Function for processing
     */
    void process(void);

private:

    void callbackVictimResponse(const ohm_perception::Victim& msg);

    ros::NodeHandle* _nh;

    ros::Publisher _state_pub;
    ros::Publisher _pubGoal;
    ros::Subscriber _subVictimResponse;
    ros::ServiceClient _srvSensorHeadMode;

    ros::Time _stamp;
    bool _response;
    const geometry_msgs::Point _goal;
};

} // end namespace autonohm

#endif
