Read me for ground_station package
===============================

Contents.
---------
About this packages.
Basic Usage.

------------------------------------------------------------
------------------------------------------------------------
About this pkg:
------------------------------------------------------------
ground_station is a package for ROS that typicaly subscribe to
several topics and update an Gtk+ based GUI.
The GUI contains several widgets made to look like to real
flight instruments in order to allow easy feedbacks for the user/pilot.

ground_station pkg is a part of CCNY CityFlyer research project.
ground_station pkg is released under the GNU General Public License (GPL).
Please read the file COPYING.txt.
 
Copyright (C) 2010, CCNY Robotics Lab
This pkg was assembled by:
	Gautier Dumonteil <gautier.dumonteil@gmail.com>
	http://robotics.ccny.cuny.edu

------------------------------------------------------------
------------------------------------------------------------
Basic Usage:
------------------------------------------------------------
Download gpsd_viewer packages in your ROS packages directory
Then, in a shell:
	roscd ground_station/
	rosmake --rosdep-install
	
Run example:
	roscd ground_station/
	roslaunch demo/telemetry.launch

Run the gui:
	roscd ground_station/
	roslaunch launch/ground_station.launch
