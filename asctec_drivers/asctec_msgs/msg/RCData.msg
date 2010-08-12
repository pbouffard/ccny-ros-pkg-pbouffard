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
# channels as read from R/C receiver
int16[8] channels_in

#channels bias free, remapped and scaled to 0..4095
int16[8] channels_out

#indicator for valid R/C receiption
int16 lock
