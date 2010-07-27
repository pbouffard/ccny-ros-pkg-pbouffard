#include <karto_scan_matcher/karto_scan_matcher.h>
#include <geometry_msgs/Pose2D.h>
#include <boost/circular_buffer.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <ros/time.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace karto_scan_matcher
{

typedef boost::circular_buffer<ScanWithPose> ScanBuffer;
typedef boost::mutex::scoped_lock Lock;

class ScanMatcherNode
{
  private:

    ros::NodeHandle nh_;
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;
    ros::Subscriber scan_sub_;
    ros::Publisher  posePublisher_;

    boost::mutex mutex_;
    boost::shared_ptr<KartoScanMatcher> matcher_;
    
    int historyLength_;
    std::vector<double> searchSizes_;
    std::vector<double> resolutions_;
    int historySkip_;
    std::string mapFrame_;
    std::string odomFrame_;
    std::string worldFrame_;
    std::string baseFrame_;

    ScanBuffer* scansHistory_;
    int scansReceived_;
    geometry_msgs::Pose2D lastScanPose_; 
   
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void addToHistory(const sensor_msgs::LaserScan& scanMsg, const geometry_msgs::Pose2D& scanPose);
    void publishMapToOdomTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time);
    void tokenize (const std::string& str, std::vector<std::string>& tokens);

  public:

    ScanMatcherNode();
};

ScanMatcherNode::ScanMatcherNode()
{
  ROS_INFO("Starting ScanMatcherNode");

  ros::NodeHandle nh_private("~");

  std::string searchSizesString;
  std::string resolutionsString;
  std::vector<std::string> searchSizesTokens;
  std::vector<std::string> resolutionsTokens;

  if (!nh_private.getParam ("history_length", historyLength_))
    historyLength_ = 5;
  if (!nh_private.getParam ("search_sizes", searchSizesString))
    searchSizesString = "1.0";
  if (!nh_private.getParam ("resolutions", resolutionsString))
    resolutionsString = "0.03";
  if (!nh_private.getParam ("history_skip", historySkip_))
    historySkip_ = 0;
  if (!nh_private.getParam ("map_frame", mapFrame_))
    mapFrame_ = "map";
  if (!nh_private.getParam ("odom_frame", odomFrame_))
    odomFrame_ = "odom";
  if (!nh_private.getParam ("world_frame", worldFrame_))
    worldFrame_ = "world";
  if (!nh_private.getParam ("base_frame", baseFrame_))
    baseFrame_ = "base_link";

  tokenize(searchSizesString, searchSizesTokens);
  tokenize(resolutionsString, resolutionsTokens);

  for(unsigned int i = 0; i < searchSizesTokens.size(); ++i)
  {
    double size = std::strtod(searchSizesTokens[i].c_str(), NULL);
    searchSizes_.push_back(size);
    std::cout << "size: " << size << std::endl;
  }

  for(unsigned int i = 0; i < resolutionsTokens.size(); ++i)
  {
    double res = std::strtod(resolutionsTokens[i].c_str(), NULL);
    resolutions_.push_back(res);
    std::cout << "res: " << res << std::endl;
  }

  ROS_ASSERT (resolutions_.size() == searchSizes_.size());
  ROS_ASSERT (historyLength_ > 1);

  scansHistory_ = new ScanBuffer(historyLength_);
  scansReceived_ = 0;


  lastScanPose_.x     = 0.0;
  lastScanPose_.y     = 0.0;
  lastScanPose_.theta = 0.0;

  scan_sub_ = nh_.subscribe("scan", 10, &ScanMatcherNode::scanCallback, this);

  posePublisher_ = nh_.advertise<geometry_msgs::Pose2D>("pose2D", 10);
}

void ScanMatcherNode::scanCallback (const sensor_msgs::LaserScan& scanMsg)
{
  scansReceived_++;

  // **** If this is the first scan, initialize the matcher
  if (!matcher_.get()) 
  {
    matcher_.reset(new KartoScanMatcher(scanMsg, lastScanPose_, searchSizes_, resolutions_));
    ROS_INFO ("Initialized matcher");
  }

  if (scansHistory_->size() < historyLength_)
  {
    ROS_INFO("Received %d/%d.", scansHistory_->size(), historyLength_);
    if(scansReceived_ % historySkip_ == 0)
    {
      // create a scan
      Lock lock(mutex_);
      addToHistory(scanMsg, lastScanPose_);
    }
    return;
  }
  long start = clock();
  boost::mutex::scoped_lock lock(mutex_);

  std::vector<ScanWithPose> referenceScans(scansHistory_->begin(), scansHistory_->end());

  //**************


//  printf("matching\n");
  geometry_msgs::Pose2D estimatedPose = matcher_->scanMatch(scanMsg, lastScanPose_, referenceScans).first;
  int dur = (clock()-start) / 1000;

  printf("dur %d\n", dur);

/*  ROS_INFO("[[%d]] (%f, %f, %f)", dur, estimatedPose.x, 
                                     estimatedPose.y, 
                                     estimatedPose.theta);*/
  lastScanPose_     = estimatedPose;

  addToHistory(scanMsg, estimatedPose);

  publishMapToOdomTf(estimatedPose, scanMsg.header.stamp);
}

void ScanMatcherNode::addToHistory(const sensor_msgs::LaserScan& scanMsg, const geometry_msgs::Pose2D& scanPose)
{
  if(scansReceived_ % historySkip_ == 0)
  {
    ScanWithPose scanWithPose;
    scanWithPose.scan = scanMsg;
    scanWithPose.pose = scanPose;
    scansHistory_->push_back(scanWithPose);
  }
}

void ScanMatcherNode::publishMapToOdomTf(const geometry_msgs::Pose2D& estimatedPose, const ros::Time& time)
{
  btTransform transform;

  btQuaternion rotation;
  rotation.setRPY (0.0, 0.0, 0.0);
  transform.setRotation (rotation);

  btVector3 origin;
  origin.setValue (estimatedPose.x, estimatedPose.y, 0.0);
  transform.setOrigin (origin);

  transform = transform.inverse();

  tf::StampedTransform transformMsg (transform, time, worldFrame_, odomFrame_);
  tfBroadcaster_.sendTransform (transformMsg);

  // publish pose2D

  //posePublisher_.publish(estimatedPose);
}

void ScanMatcherNode::tokenize(const std::string& str, std::vector<std::string>& tokens)
{
  std::string::size_type lastPos = str.find_first_not_of (" ", 0);
  std::string::size_type pos = str.find_first_of (" ", lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos)
  {
    std::string stringToken = str.substr (lastPos, pos - lastPos);
    tokens.push_back (stringToken);
    lastPos = str.find_first_not_of (" ", pos);
    pos = str.find_first_of (" ", lastPos);
  }
}

} // namespace karto_scan_matcher

int main (int argc, char** argv)
{

  ros::init(argc, argv, "ScanMatcherNode");
  karto_scan_matcher::ScanMatcherNode node;
  ros::spin();
}
