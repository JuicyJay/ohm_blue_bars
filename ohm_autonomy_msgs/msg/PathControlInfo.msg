int8 MOVING          = 1
int8 PAUSE           = 2
int8 STOP            = 4
int8 ARRIVED         = 8 
int8 NO_PATH         = 16

float64 pathLenght
float64 pathLenght_remaining
uint32  numWaypoints
uint32  currentWaypoint
bool    doEndRotate
uint8   state
string  state_str
