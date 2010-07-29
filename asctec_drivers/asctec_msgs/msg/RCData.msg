#    struct RC_DATA
#    {
#      //channels as read from R/C receiver
#      unsigned short channels_in[8];
#      //channels bias free, remapped and scaled to 0..4095
#      unsigned short channels_out[8];
#     //Indicator for valid R/C receiption
#      unsigned char lock;
#    };
Header header
uint16[8] channels_in
uint16[8] channels_out
uint8 lock
