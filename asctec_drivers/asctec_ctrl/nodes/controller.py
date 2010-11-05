#!/usr/bin/env python
# AscTec Autopilot Console Controller
# Copyright (C) 2010, CCNY Robotics Lab
# William Morris <morris@ee.ccny.cuny.edu>
#
# http://robotics.ccny.cuny.edu
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

################################
# This code may not be pretty but it does not work yet.
# Commands
# q = Quit
# z = zero control outputs
# w = increase thrust
# s = decrease thrust

import roslib; roslib.load_manifest('asctec_ctrl')
import rospy
import os
import signal
import curses

from asctec_msgs.msg import CtrlInput

myscreen = curses.initscr()

def talker():
    pub = rospy.Publisher('/CtrlInput', CtrlInput)
    rospy.init_node('asctec_ctrl')
    thrust = 0
    while not rospy.is_shutdown():
        k = myscreen.getch()
        if k == ord('q'):
            break
        if k == ord('z'):
            thrust = 0
        if k == ord('w'):
            thrust = thrust + 16
        if k == ord('s'):
            thrust = thrust - 16
        if thrust > 4096:
            thrust = 4096
        if thrust < 0:
            thrust = 0
        pitch = 0
        roll = 0
        yaw = 0
        # ctrl
        #   bit 0: pitch control enabled
        #   bit 1: roll control enabled
        #   bit 2: yaw control enabled
        #   bit 3: thrust control enabled
        # These bits can be used to only enable one axis at a time
        # and thus to control the other axes manually. This usually
        # helps a lot to set up and finetune controllers for each
        # axis seperately.
        ctrl = int(0b1000)
        csum = pitch + roll + yaw + thrust + int(ctrl) - 21846
        cin = CtrlInput()
        cin.pitch, cin.roll, cin.yaw, cin.thrust, cin.ctrl, cin.chksum = pitch,roll,yaw,thrust,ctrl,csum
        pub.publish(cin)
        rospy.sleep(0.05)
    curses.nocbreak(); myscreen.keypad(0); curses.echo(); curses.curs_set(1)
    curses.endwin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
