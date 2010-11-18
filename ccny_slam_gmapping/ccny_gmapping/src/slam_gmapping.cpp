/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */

#include "slam_gmapping.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), transform_thread_(NULL)
{
  gsp_ = new GMapping::GridSlamProcessor();
  ROS_ASSERT(gsp_);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  gsp_laser_ = NULL;
  gsp_odom_ = NULL;

  got_first_scan_ = false;
  got_map_ = false;

  ros::NodeHandle private_nh_("~");

  // Parameters used by our GMapping wrapper
  if(!private_nh_.getParam("inverted_laser", inverted_laser_))
    inverted_laser_ = false;
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters used by GMapping itself
  maxUrange_ = 0.0;  maxRange_ = 0.0; // preliminary default, will be set in initMapper()
  if(!private_nh_.getParam("sigma", sigma_))
    sigma_ = 0.05;
  if(!private_nh_.getParam("kernelSize", kernelSize_))
    kernelSize_ = 1;
  if(!private_nh_.getParam("lstep", lstep_))
    lstep_ = 0.05;
  if(!private_nh_.getParam("astep", astep_))
    astep_ = 0.05;
  if(!private_nh_.getParam("iterations", iterations_))
    iterations_ = 5;
  if(!private_nh_.getParam("lsigma", lsigma_))
    lsigma_ = 0.075;
  if(!private_nh_.getParam("ogain", ogain_))
    ogain_ = 3.0;
  if(!private_nh_.getParam("lskip", lskip_))
    lskip_ = 0;
  if(!private_nh_.getParam("srr", srr_))
    srr_ = 0.1;
  if(!private_nh_.getParam("srt", srt_))
    srt_ = 0.2;
  if(!private_nh_.getParam("str", str_))
    str_ = 0.1;
  if(!private_nh_.getParam("stt", stt_))
    stt_ = 0.2;
  if(!private_nh_.getParam("linearUpdate", linearUpdate_))
    linearUpdate_ = 1.0;
  if(!private_nh_.getParam("angularUpdate", angularUpdate_))
    angularUpdate_ = 0.5;
  if(!private_nh_.getParam("temporalUpdate", temporalUpdate_))
    temporalUpdate_ = -1.0;
  if(!private_nh_.getParam("resampleThreshold", resampleThreshold_))
    resampleThreshold_ = 0.5;
  if(!private_nh_.getParam("particles", particles_))
    particles_ = 30;
  if(!private_nh_.getParam("xmin", xmin_))
    xmin_ = -100.0;
  if(!private_nh_.getParam("ymin", ymin_))
    ymin_ = -100.0;
  if(!private_nh_.getParam("xmax", xmax_))
    xmax_ = 100.0;
  if(!private_nh_.getParam("ymax", ymax_))
    ymax_ = 100.0;
  if(!private_nh_.getParam("delta", delta_))
    delta_ = 0.05;
  if(!private_nh_.getParam("occ_thresh", occ_thresh_))
    occ_thresh_ = 0.25;
  if(!private_nh_.getParam("llsamplerange", llsamplerange_))
    llsamplerange_ = 0.01;
  if(!private_nh_.getParam("llsamplestep", llsamplestep_))
    llsamplestep_ = 0.01;
  if(!private_nh_.getParam("lasamplerange", lasamplerange_))
    lasamplerange_ = 0.005;
  if(!private_nh_.getParam("lasamplestep", lasamplestep_))
    lasamplestep_ = 0.005;

  entropy_publisher_ = private_nh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamGMapping::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<laser_ortho_projector::LaserScanWithAngles>(node_, scanOrthoTopic_, 5);
  scan_filter_ = new tf::MessageFilter<laser_ortho_projector::LaserScanWithAngles>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamGMapping::laserCallback, this, _1));

  transform_thread_ = new boost::thread(boost::bind(&SlamGMapping::publishLoop, this, transform_publish_period));
}

void SlamGMapping::publishLoop(double transform_publish_period)
{
  if(transform_publish_period == 0) return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamGMapping::~SlamGMapping()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  delete gsp_;
  if(gsp_laser_)
    delete gsp_laser_;
  if(gsp_odom_)
    delete gsp_odom_;
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool SlamGMapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), t, base_frame_);

  //map_to_odom_mutex_.lock();

  tf::Stamped<btTransform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  
  //map_to_odom_mutex_.unlock();

  double roll, pitch, yaw;
  btMatrix3x3 m(odom_pose.getRotation());
  m.getRPY(roll, pitch, yaw);

  gmap_pose.x = odom_pose.getOrigin().x();
  gmap_pose.y = odom_pose.getOrigin().y();
  gmap_pose.z = odom_pose.getOrigin().z();  

  gmap_pose.roll  = roll;
  gmap_pose.pitch = pitch;
  gmap_pose.theta = yaw;

  last_odom_pose = odom_pose;

  return true;
}

bool SlamGMapping::initMapper(const laser_ortho_projector::LaserScanWithAngles& scan)
{
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<btTransform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = scan.header.frame_id;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  double yaw = tf::getYaw(laser_pose.getRotation());

  GMapping::OrientedPoint gmap_pose(laser_pose.getOrigin().x(),
                                    laser_pose.getOrigin().y(),
                                    yaw);
  ROS_DEBUG("laser's pose wrt base: %.3f %.3f %.3f",
            laser_pose.getOrigin().x(),
            laser_pose.getOrigin().y(),
            yaw);


  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~");
  if(!private_nh_.getParam("maxRange", maxRange_))
    maxRange_ = scan.range_max - 0.01;
  if(!private_nh_.getParam("maxUrange", maxUrange_))
    maxUrange_ = maxRange_;

  double scan_angles[scan.angles.size()];
  std::copy(scan.angles.begin(), scan.angles.end(), scan_angles);

  // The laser must be called "FLASER"
  gsp_laser_ = new GMapping::RangeSensor("FLASER",
                                         scan.ranges.size(),
                                         scan_angles,
                                         gmap_pose,
                                         0.0,
                                         maxRange_);

  ROS_ASSERT(gsp_laser_);

  GMapping::SensorMap smap;
  smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
  gsp_->setSensorMap(smap);

  gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);
  ROS_ASSERT(gsp_odom_);


  /// @todo Expose setting an initial pose
  GMapping::OrientedPoint initialPose;
  if(!getOdomPose(initialPose, scan.header.stamp))
    initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);

  gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                              kernelSize_, lstep_, astep_, iterations_,
                              lsigma_, ogain_, lskip_);

  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(true);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);

  // Call the sampling function once to set the seed.
  GMapping::sampleGaussian(1,time(NULL));

  ROS_INFO("Initialization complete");

  return true;
}

bool SlamGMapping::addScan(const laser_ortho_projector::LaserScanWithAngles& scan, GMapping::OrientedPoint& gmap_pose)
{
  if(!getOdomPose(gmap_pose, scan.header.stamp))
     return false;

  // update laser angles

  for (unsigned int i = 0; i < scan.angles.size(); i++)
    gsp_laser_->m_beams[i].pose.theta = scan.angles[i];

  gsp_laser_->updateBeamsLookup();

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  if (inverted_laser_) 
  {
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else 
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 gsp_laser_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(gmap_pose);
 
// **********************************

  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), scan.header.stamp, base_frame_);
  tf::Stamped<btTransform> pose3d;
  try
  {
    tf_.transformPose(odom_frame_, ident, pose3d);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute pose3d (%s)", e.what());
  }
  
  double roll, pitch, yaw;
  btMatrix3x3 m(pose3d.getRotation());
  m.getRPY(roll, pitch, yaw);

  GMapping::OrientedPoint gmap_pose_3d;
  gmap_pose_3d.x = pose3d.getOrigin().x();
  gmap_pose_3d.y = pose3d.getOrigin().y();
  gmap_pose_3d.z = pose3d.getOrigin().z();  

  gmap_pose_3d.roll  = roll;
  gmap_pose_3d.pitch = pitch;
  gmap_pose_3d.theta = yaw;
  
// *****************************************

  ros::Time startAddScan = ros::Time::now();
  bool processScanResult = gsp_->processScan(reading, gmap_pose_3d);
  ros::Duration durAddScan = ros::Time::now() - startAddScan;
  //ROS_INFO("processScan duration:\t\t %d", durAddScan.toNSec()/1000000);

  return processScanResult;
}

void SlamGMapping::cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud)
{
  ROS_INFO("Swisscallback");

  boost::mutex::scoped_lock(cloud_mutex_);

  lastCloud_ = *cloud;
}

void SlamGMapping::laserCallback(const laser_ortho_projector::LaserScanWithAngles::ConstPtr& scan)
{
  printf("---------------------- scan coming----------------------\n");
  ros::Time startAddScan = ros::Time::now();

  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // We can't initialize the mapper until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  GMapping::OrientedPoint odom_pose;


  bool addScanResult = addScan(*scan, odom_pose);

  if(addScanResult)
  {

    GMapping::OrientedPoint mpose   = gsp_->getParticles()[gsp_->getBestParticleIndex()].node->pose;
    GMapping::OrientedPoint mpose3d = gsp_->getParticles()[gsp_->getBestParticleIndex()].node->pose3d;

    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(
        odom_frame_,tf::Stamped<tf::Pose> (
          btTransform(
            tf::createQuaternionFromRPY(0, 0, mpose.theta),
            btVector3(mpose.x, mpose.y, 0)).inverse(),
          scan->header.stamp, 
          base_frame_),
        odom_to_map
        );
    }
    catch(tf::TransformException e){
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      ros::Time startUpdateMap = ros::Time::now();
      updateMap(*scan);
      ros::Duration durUpdateMap = ros::Time::now() - startUpdateMap;
      //ROS_INFO("UpdateMap duration:\t\t\t\t %d", durUpdateMap.toNSec()/1000000);

      last_map_update = scan->header.stamp;
    }
  }

  ros::Duration durAddScan = ros::Time::now() - startAddScan;
  ROS_INFO("AddScan duration:\t\t %d", durAddScan.toNSec()/1000000);
}

double
SlamGMapping::computePoseEntropy()
{
  double weight_total=0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    weight_total += it->weight;
  }
  double entropy = 0.0;
  for(std::vector<GMapping::GridSlamProcessor::Particle>::const_iterator it = gsp_->getParticles().begin();
      it != gsp_->getParticles().end();
      ++it)
  {
    if(it->weight/weight_total > 0.0)
      entropy += it->weight/weight_total * log(it->weight/weight_total);
  }
  return -entropy;
}

void SlamGMapping::updateMap(const laser_ortho_projector::LaserScanWithAngles& scan)
{
  boost::mutex::scoped_lock(map_mutex_);

  const GMapping::GridSlamProcessor::Particle &best =
          gsp_->getParticles()[gsp_->getBestParticleIndex()];
  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  const GMapping::ScanMatcherMap &smap = best.map;

  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    xmin_ = wmin.x; ymin_ = wmin.y;
    xmax_ = wmax.x; ymax_ = wmax.y;
    
    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              xmin_, ymin_, xmax_, ymax_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = xmin_;
    map_.map.info.origin.position.y = ymin_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

bool SlamGMapping::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void SlamGMapping::publishTransform()
{
  map_to_odom_mutex_.lock();
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

