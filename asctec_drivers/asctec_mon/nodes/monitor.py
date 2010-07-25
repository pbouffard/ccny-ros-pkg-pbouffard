#!/usr/bin/env python
# AscTec Console Monitor
################################
# This code may not be pretty but is seems to work.
import roslib; roslib.load_manifest('asctec_mon')
import rospy
import curses

from asctec_msgs.msg import LLStatus
from asctec_msgs.msg import IMUCalcData

myscreen = curses.initscr()
curses.start_color()
curses.halfdelay(1)
curses.noecho()
curses.curs_set(0)
(maxx,maxy) = myscreen.getmaxyx()
llwin = curses.newwin(11, maxy, maxx-11, 0)
imuwin = curses.newwin(maxx-11, maxy, 0, 0)
alarm = 0;
alarm_count = 0;
alarm_interval = 10;

def drawSignedVal(r,c,w,val,val_max,val_min,big):
    center = int(w/2)
    if (val > val_max):
        val = val_max
    if (val < val_min):
        val = val_min
    if big:
        # Draw Top
        imuwin.addch(r, c, curses.ACS_ULCORNER)
        for n in range(c+1, c+w):
            if (n == c+center):
                imuwin.addch(r, n, curses.ACS_TTEE)
            else:
                imuwin.addch(r, n, curses.ACS_HLINE)
        imuwin.addch(r, c+w, curses.ACS_URCORNER)
        r = r + 1

    # Draw Middle
    imuwin.addch(r, c, curses.ACS_VLINE)
    bar = int(float(val / val_max * center))
    if (bar == 0):
        imuwin.addch(r, c+center, curses.ACS_VLINE)
    elif (bar >= 0):
        imuwin.addstr(r, c+center, " "*bar, curses.color_pair(4))
        imuwin.addch(r, c+center, curses.ACS_VLINE,curses.color_pair(4))
    else:
        imuwin.addstr(r, c+center+bar+1, " "*(-1*bar), curses.color_pair(4))
        imuwin.addch(r, c+center, curses.ACS_VLINE,curses.color_pair(4))
    imuwin.addstr(r, c+1, str(bar))
    imuwin.addch(r, c+w, curses.ACS_VLINE)
    r = r + 1

    if big:
        # Draw Bottom
        imuwin.addch(r, c, curses.ACS_LLCORNER)
        for n in range(c+1, c+w):
            if (n == c+center):
                imuwin.addch(r, n, curses.ACS_BTEE)
            else:
                imuwin.addch(r, n, curses.ACS_HLINE)
        imuwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawBattery(r,c,w,battery_val):
    global alarm
    # Battery Settings
    # Taken from http://en.wikipedia.org/wiki/Lithium-ion_polymer_battery
    battery_max = 12.7   # Maximum Voltage
    battery_warn = 10.0  # Warning Voltage
    battery_min = 8.4    # Minimum Voltage

    # Draw Top
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        llwin.addch(r, n, curses.ACS_HLINE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    # Draw Middle
    llwin.addch(r, c, curses.ACS_VLINE)
    b = int((battery_val - battery_min)/(battery_max-battery_min)*w)
    if battery_val > battery_warn: 
        alarm = 0
        llwin.addstr(r, c+1, " " * b, curses.color_pair(4))
    else:
        alarm = 1
        llwin.addstr(r, c+1, " " * b, curses.color_pair(5))
    llwin.addch(r, c+w, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        llwin.addch(r, n, curses.ACS_HLINE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawStatusMode(r,c,w,data):
    # Draw Top
    size = int(w / 3)-1
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_TTEE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    size = size + 2
    pos = c+(size*0)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if (data.compass_enabled):
        llwin.addstr(r,pos+1,"Compass",curses.color_pair(3)|curses.A_BOLD)
    else:
        llwin.addstr(r,pos+1,"Compass",curses.color_pair(0))

    pos = c+(size*1)
    llwin.addch(r, pos, curses.ACS_VLINE)
    llwin.addstr(r,pos+1,"Flight Time: "+str(data.up_time)+" sec",curses.color_pair(0))

    pos = c+(size*2)
    llwin.addch(r, pos, curses.ACS_VLINE)
    llwin.addstr(r,pos+1,"CPU: "+str(data.cpu_load),curses.color_pair(0))

    pos = c+w
    llwin.addch(r, pos, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    size = int(w / 3)-1
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_BTEE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def drawFlightMode(r,c,w,flightMode):
    # Draw Top
    size = int(w / 5)-1
    llwin.addch(r, c, curses.ACS_ULCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_TTEE)
    llwin.addch(r, c+w, curses.ACS_URCORNER)
    r = r + 1

    size = size + 2
    # There are 5 Flight Modes but the bit index of the serial active
    # mode is currently unknown
    pos = c+(size*0)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b01111111)!=0b11111111):
        llwin.addstr(r,pos+1,"Emergency",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Emergency",curses.color_pair(2)|curses.A_BOLD)

    pos = c+(size*1)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11111101)!=0b11111111):
        llwin.addstr(r,pos+1,"Height Control",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Height Control",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*2)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11111011)!=0b11111111):
        llwin.addstr(r,pos+1,"GPS Mode",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"GPS Mode",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*3)
    llwin.addch(r, pos, curses.ACS_VLINE)
    if ((flightMode|0b11011111)!=0b11111111):
        llwin.addstr(r,pos+1,"Serial Enable",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Serial Enable",curses.color_pair(3)|curses.A_BOLD)

    pos = c+(size*4)
    llwin.addch(r, pos, curses.ACS_VLINE)
    # FIXME: This is probably the wrong bitmask
    if ((flightMode|0b10111111)!=0b11111111):
        llwin.addstr(r,pos+1,"Serial Active",curses.color_pair(0))
    else:
        llwin.addstr(r,pos+1,"Serial Active",curses.color_pair(3)|curses.A_BOLD)
    pos = c+w
    llwin.addch(r, pos, curses.ACS_VLINE)
    r = r + 1

    # Draw Bottom
    size = int(w / 5)-1
    llwin.addch(r, c, curses.ACS_LLCORNER)
    for n in range(c+1, c+w):
        if ((n%(size+c))-2):
          llwin.addch(r, n, curses.ACS_HLINE)
        else:
          llwin.addch(r, n, curses.ACS_BTEE)
    llwin.addch(r, c+w, curses.ACS_LRCORNER)

def imu_callback(data):
    imuwin.clear()
    (imu_maxx,imu_maxy) = imuwin.getmaxyx()
    imu_maxy = imu_maxy - 2 # remove space for left and right border
    gcol = 25
    imuwin.border(0)
    imuwin.addstr(0, 1, "AscTec Quadrotor Console Monitor", curses.color_pair(1)|curses.A_BOLD)

    pos = 1
    if (imu_maxx > 16):
        pos_inc = 3
        big = 1
    else:
        pos_inc = 1
        big = 0

    # Height Graph
    ################################
    height = float(data.height)/1000.0
    imuwin.addstr(pos+big, 2, 'Height: {0:.3f}m'.format(height))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),height,10.0,-10.0,big)
    pos = pos + pos_inc
    
    # Roll Graph
    ################################
    roll = float(data.angle_roll)/1000.0
    imuwin.addstr(pos+big, 2, 'Roll: {0:.3f}deg'.format(roll))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),roll,90.0,-90.0,big)
    pos = pos + pos_inc

    # Pitch Graph
    ################################
    pitch = float(data.angle_nick)/1000.0
    imuwin.addstr(pos+big, 2, 'Pitch: {0:.3f}deg'.format(pitch))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),pitch,180.0,-180.0,big)
    pos = pos + pos_inc

    # Yaw Graph
    ################################
    yaw = float(data.angle_yaw)/1000.0 -180
    imuwin.addstr(pos+big, 2, 'Fused Yaw: {0:.3f}deg'.format(yaw))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),yaw,180.0,-180.0,big)
    pos = pos + pos_inc

    # Compass Graph
    ################################
    mag = float(data.mag_heading)/1000.0 -180
    imuwin.addstr(pos+big, 2, 'Compass: {0:.3f}deg'.format(mag))
    drawSignedVal(pos,gcol,imu_maxy-(gcol+1),mag,180.0,-180.0,big)
    pos = pos + pos_inc

def callback(data):
    llwin.clear()
    (maxx,maxy) = llwin.getmaxyx()
    maxy = maxy - 2 # remove space for left and right border
    gcol = 20
    llwin.border(0)

    # Battery Monitor
    ################################
    battery_val = float(data.battery_voltage_1)/1000.0
    llwin.addstr(2, 2, 'Battery: {0:.3f}V'.format(battery_val))
    drawBattery(1,gcol,maxy-(gcol+1),float(data.battery_voltage_1)/1000)

    # Flight Mode Monitor
    ################################
    drawFlightMode(4,2,maxy-3,data.flightMode)

    # Status Monitor
    ################################
    drawStatusMode(7,2,maxy-3,data)

def listener():
    global imuwin, maxx, maxy
    global alarm, alarm_count, alarm_interval

    rospy.init_node('cursed_controller', anonymous=True)
    rospy.Subscriber("/autopilot/LL_STATUS", LLStatus, callback)
    rospy.Subscriber("/autopilot/IMU_CALCDATA", IMUCalcData, imu_callback)
    curses.init_pair(1, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_BLACK, curses.COLOR_WHITE)
    curses.init_pair(5, curses.COLOR_BLACK, curses.COLOR_RED)
    r = rospy.Rate(10) # 10hz
    (maxx,maxy) = myscreen.getmaxyx()
    while not rospy.is_shutdown():
        c = myscreen.getch()
        if c == ord('f'): curses.flash()
        elif c == ord('b'): curses.beep()
        elif c == ord('q'): break  # Exit the while()
        elif c == curses.KEY_HOME: x = y = 0
        (current_maxx,current_maxy) = myscreen.getmaxyx()
        if (current_maxx != maxx or current_maxy != maxy):
            (maxx,maxy) = myscreen.getmaxyx()
            llwin.mvwin(maxx-11, 0)
            imuwin = curses.newwin(maxx-11, maxy, 0, 0)
        if (alarm):
	    alarm_count = alarm_count + 1
            if (alarm_count == alarm_interval):
                alarm_count = 0
                curses.flash()
                curses.beep()
         
        imuwin.refresh()
        llwin.refresh()
        r.sleep()
    curses.nocbreak(); myscreen.keypad(0); curses.echo(); curses.curs_set(1)
    curses.endwin()

if __name__ == '__main__':
    listener()
