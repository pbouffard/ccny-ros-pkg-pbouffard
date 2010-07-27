#    struct CONTROLLER_OUTPUT
#    {
#      //attitude controller outputs; 0..200 = -100 ..+100%
#      int nick;
#      int roll;
#      int yaw;
#      //current thrust (height controller output); 0..200 = 0..100%
#      int thrust;
#    };
int32 pitch
int32 roll
int32 yaw
int32 thrust
