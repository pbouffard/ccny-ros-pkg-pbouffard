Header header
# serial command (=Scientific Interface)
# Pitch input: -2047 .. 2047 (0 = neutral)
int16 pitch
# Roll input: -2047 .. 2047 (0 = neutral)
int16 roll
# R/C Stick input: -2047 .. 2047 (0 = neutral)
int16 yaw
# Collective: 0 .. 4095 (= 0 .. 100%)
int16 thrust
# control byte:
#    bit 0: pitch control enabled
#    bit 1: roll control enabled
#    bit 2: yaw control enabled
#    bit 3: thrust control enabled
#  These bits can be used to only enable one axis at a time and thus to control
#  the other axes manually. This usually helps a lot to set up and finetune
#  controllers for each axis seperately.
int16 ctrl
int16 chksum
