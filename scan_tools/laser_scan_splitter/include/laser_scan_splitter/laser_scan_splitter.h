/*
*  Laser Scan Splitter
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
*  http://robotics.ccny.cuny.edu
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H
#define LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// topic names

const std::string scanTopic_ = "scan";

class LaserScanSplitter
{
public:

  LaserScanSplitter ();
  virtual ~ LaserScanSplitter ();

private:

  // paramaters

  std::vector < std::string > publishedScanTopics_;
  std::vector < std::string > publishedLaserFrames_;
  std::vector < int >sizes_;
  unsigned int sizeSum_;

  // publishers & subscirbers

  ros::Subscriber scanSubscriber_;
  std::vector < ros::Publisher > scanPublishers_;

  void scanCallback (const sensor_msgs::LaserScanConstPtr & scan);
  void tokenize (const std::string & str, std::vector < std::string > &tokens);
};

#endif // LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H
