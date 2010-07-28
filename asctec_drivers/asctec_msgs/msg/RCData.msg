# channels as read from R/C receiver
int16[8] channels_in

#channels bias free, remapped and scaled to 0..4095
int16[8] channels_out

#indicator for valid R/C receiption
int16 lock
