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
