Read me for osmgpsmap package
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
osmgpsmap is a package providing a local ROS installation of
osm-gps-map v0.72.
osm-gps-map is released under the GNU General Public License (GPL).
Please read the file COPYING.txt.
 
Copyright (C) 2010, CCNY Robotics Lab
This pkg was assembled by:
	Gautier Dumonteil <gautier.dumonteil@gmail.com>
	http://robotics.ccny.cuny.edu

------------------------------------------------------------
------------------------------------------------------------
Requirements:
------------------------------------------------------------
osmgpsmap requires libsoup2.4. 
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
Download osmgpsmap packages in your ROS packages directory
Then, in a shell:
	roscd osmgpsmap/
	rosdep install osmgpsmap
	make
	
or rosmake --rosdep-install osmgpsmap
