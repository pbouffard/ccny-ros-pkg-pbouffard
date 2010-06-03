Read me for gpsd_viewer package
===============================

Contents.
---------
About this packages.
Requirements.
Basic Usage.

------------------------------------------------------------
------------------------------------------------------------
About this pkg:
------------------------------------------------------------
gpsd_viewer is a package for ROS that typicaly subscribe gpsd odometry
topic and using OsmGpsMap draw the gps pose in a map (default: Open Street Map)
gpsd_viewer is released under the GNU General Public License (GPL). Please read the file COPYING.txt.
 
Copyright (C) 2010, CCNY Robotics Lab
This pkg was assembled by:
	Gautier Dumonteil <gautier.dumonteil@gmail.com>
	http://robotics.ccny.cuny.edu

------------------------------------------------------------
------------------------------------------------------------
Requirements:
------------------------------------------------------------
gpsd_viewer requires OsmGpsMap libraries and include files to
be available in order to build. 
Make sure all packages dependances are check.
This pkg run curently only on 10.04 ubuntu due to lib change.

This is currently compatible with the UMD gpsd_client
http://www.ros.org/browse/details.php?name=gpsd_client
If you have trouble compiling gpsd_client try our git mirror
http://robotics.ccny.cuny.edu/git/umd-ros-pkg.git
------------------------------------------------------------
------------------------------------------------------------
Basic Usage:
------------------------------------------------------------
Download gpsd_viewer packages in your ROS packages directory
Then, in a shell:
	roscd gpsd_viewer/
	rosmake
	
Run the example:
	roscd gpsd_viewer/
	cd demo/
	./setup.sh
	roslaunch demo.launch
