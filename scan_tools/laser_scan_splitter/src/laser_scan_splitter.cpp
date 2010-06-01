#include "laser_scan_splitter/laser_scan_splitter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_splitter");
  LaserScanSplitter laserScanSplitter;
  ros::spin();
	return 0;
}

LaserScanSplitter::LaserScanSplitter()
{
	ROS_INFO("Starting LaserScanSplitter"); 

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** get paramters

  std::string sizesString;
  std::string framesString;
  std::string topicsString;

  if(!nh_private.getParam("topics", topicsString))
    topicsString = "scan1 scan2";
  if(!nh_private.getParam("frames", framesString))
    framesString = "laser laser";
  if(!nh_private.getParam("sizes", sizesString))
    sizesString = "256 256";
  
  // **** tokenize inputs
  tokenize(topicsString, publishedScanTopics_);
  tokenize(framesString, publishedLaserFrames_);

  std::vector<std::string> sizesTokens;
  tokenize(sizesString, sizesTokens);

  sizeSum_ = 0;
  for(int i = 0; i < sizesTokens.size(); i++)
  {
    sizes_.push_back(atoi(sizesTokens[i].c_str()));
    sizeSum_ += sizes_[i];
    if (sizes_[i] == 0) 
    {
      ROS_ERROR("LaserScanSplitter: Scan size cannot be zero. Quitting");
      exit(0);
    }
  }

  // **** check that topic, frames, and sizes vectors have same sizes
  if ((publishedScanTopics_.size() != publishedLaserFrames_.size()) ||
      (sizes_.size()               != publishedLaserFrames_.size()) )
  {
    ROS_ERROR("LaserScanSplitter: Invalid parameters. Quitting.");
    exit(0);
  }
 
  // **** subscribe to laser scan messages
  scanSubscriber_ = nh.subscribe(scanTopic_, 100,  &LaserScanSplitter::scanCallback, this);

  // **** advertise topics
  for (int i = 0; i < publishedScanTopics_.size(); i++)
  {
    scanPublishers_.push_back(ros::Publisher());
    scanPublishers_[i] = nh_private.advertise<sensor_msgs::LaserScan>(publishedScanTopics_[i], 10);
  }
}

LaserScanSplitter::~LaserScanSplitter()
{

}

void LaserScanSplitter::scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{

  // **** check for scan size

  if (sizeSum_ != scan->ranges.size())
  {
    ROS_WARN("LaserScanSplitter: Received a laser scan with size incompatible to input parameters. Skipping scan.");
    return;
  }

  // **** copy information over
  int r = 0;

  for (int i = 0; i < publishedScanTopics_.size(); i++)
  {
    sensor_msgs::LaserScan scanSegment;

    scanSegment.header          = scan->header;
    scanSegment.range_min       = scan->range_min;
    scanSegment.range_max       = scan->range_max;
    scanSegment.angle_increment = scan->angle_increment;
    scanSegment.time_increment  = scan->time_increment;
    scanSegment.scan_time       = scan->scan_time;
    scanSegment.header.frame_id = publishedLaserFrames_[i];

    scanSegment.angle_min = scan->angle_min + (scan->angle_increment * r);
    scanSegment.angle_max = scan->angle_min + (scan->angle_increment * (r + sizes_[i] - 1));

    // TODO - also copy intensity values

    for(int ii = 0; ii < sizes_[i]; ii++)
    {
      scanSegment.ranges.push_back(scan->ranges[r]);
      r++;
    }

    scanPublishers_[i].publish(scanSegment);
  }
}

void LaserScanSplitter::tokenize(const std::string& str, std::vector<std::string>& tokens)
{
  std::string::size_type lastPos = str.find_first_not_of(" ", 0);
  std::string::size_type pos     = str.find_first_of(" ", lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    std::string stringToken = str.substr(lastPos, pos - lastPos);
    tokens.push_back(stringToken);
    lastPos = str.find_first_not_of(" ", pos);
    pos = str.find_first_of(" ", lastPos);
  }
}
