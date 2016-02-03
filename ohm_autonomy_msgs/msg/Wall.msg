# This is a message for walls.
# Enum Orientation
uint8 NONE  = 0
uint8 UP    = 1
uint8 DOWN  = 2
uint8 LEFT  = 4
uint8 RIGHT = 8

uint32 id
uint8 orientation
geometry_msgs/Point center
geometry_msgs/Point v
geometry_msgs/Point n
float32 length

