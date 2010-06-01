#ifndef LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H
#define LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// topic names

const std::string scanTopic_ = "scan";

class LaserScanSplitter
{
	public:
  
    LaserScanSplitter();
    virtual ~LaserScanSplitter();

	private:

    // paramaters

    std::vector<std::string> publishedScanTopics_;    
    std::vector<std::string> publishedLaserFrames_;
    std::vector<int> sizes_;
    int sizeSum_;

		// publishers & subscirbers

    ros::Subscriber scanSubscriber_;
    std::vector<ros::Publisher> scanPublishers_;

		void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
    void tokenize(const std::string& str, std::vector<std::string>& tokens);
};

#endif // LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H

