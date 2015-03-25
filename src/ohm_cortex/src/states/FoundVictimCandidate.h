#ifndef ___FOUND_VICTIM_CANDIDATE_H___
#define ___FOUND_VICTIM_CANDIDATE_H___

#include "../IState.h"

#include <ohm_perception/Victim.h>
#include <ohm_perception/GetVictim.h>
#include <ohm_autonomy/MoveRobot.h>

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
    FoundVictimCandidate(const ohm_perception::Victim& goal);
    /**
     * Default destructor
     */
    virtual ~FoundVictimCandidate(void);

    /**
     * Function for processing
     */
    void process(void);

private:

    void callbackMoveRobot(const ohm_autonomy::MoveRobot& msg);

    ros::NodeHandle* _nh;

    ros::Publisher _state_pub;
    ros::Publisher _pubGoal;
    ros::Subscriber _subMoveRobot;
    ros::ServiceClient _srvSensorHeadMode;
    ros::ServiceClient _srvVictimStack;

    ros::Time _stamp;
    const ohm_perception::Victim _goal;
};

} // end namespace autonohm

#endif
