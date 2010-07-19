/*
*  New College Dataset Parser
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

#ifndef NCD_PARSER_NCD_PARSER
#define NCD_PARSER_NCD_PARSER

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

const double DEG_TO_RAD = 3.14159 / 180.0;

const double RANGE_MIN = 0.20;
const double RANGE_MAX = 50.0;

const std::string worldFrame_      = "map";
const std::string odomFrame_       = "odom";
const std::string leftLaserFrame_  = "laser_left";
const std::string rightLaserFrame_ = "laser_right";

class NCDParser
{
  private:

    char* filename_;
    double rate_;
    double start_, end_;
    double lastTime_;

    ros::Publisher  leftLaserPublisher_;
    ros::Publisher  rightLaserPublisher_;

    tf::TransformBroadcaster tfBroadcaster_;

    btTransform worldToOdom_;
    btTransform odomToLeftLaser_;
    btTransform odomToRightLaser_;

    void publishLaserMessage(const std::vector<std::string>& tokens,
                             const std::string& laserFrame,
                             const ros::Publisher& publisher);

    void publishTfMessages(const std::vector<std::string>& tokens);

    void tokenize (const std::string& str, 
                         std::vector <std::string> &tokens, 
                         std::string sentinel);

    std::vector<float> extractArray(std::string s, std::string pattern);

    double extractValue(std::string s, std::string pattern);

    void createOdomToLeftLaserTf();
    void createOdomToRightLaserTf();
  
  public:

    NCDParser(char* filename);
    virtual ~NCDParser();

    void launch();
};

#endif
