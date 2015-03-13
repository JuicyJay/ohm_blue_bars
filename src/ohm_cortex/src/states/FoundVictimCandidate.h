#ifndef ___FOUND_VICTIM_CANDIDATE_H___
#define ___FOUND_VICTIM_CANDIDATE_H___

#include "../IState.h"

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
    FoundVictimCandidate(void);
    /**
     * Default destructor
     */
    virtual ~FoundVictimCandidate(void);

    /**
     * Function for processing
     */
    void process(void);

private:
    ros::NodeHandle* _nh;

    ros::Publisher _state_pub;
    ros::Time _stamp;
};

} // end namespace autonohm

#endif
