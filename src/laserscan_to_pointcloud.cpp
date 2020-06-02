#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

//#include "tf/message_filter.h"
//#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <cmath>      // cos, sin


class LaserScanToPointCloud{

public:

  ros::NodeHandle nh;
  laser_geometry::LaserProjection  laserProjector;
  tf::TransformListener tfListener;
  ros::Subscriber laserSub;
  //tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher pointCloudPub;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    nh(n) //,
    //laserSub(nh, "front_scan", 1) //,   // node handle, topic, queue size
    //laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    /*laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    */
    laserSub = nh.subscribe("/X1/front_scan", 1, &LaserScanToPointCloud::laserScanCallback, this);
    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/my_cloud",1);
  }

  ~LaserScanToPointCloud(){}  // destructor

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    // @TODO: X1 is hard coded, must be changed to work with any robot
    if(!tfListener.waitForTransform(
        scan_in->header.frame_id,
        "X1/base_link",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0)))
    {
      return;
    }

     // http://wiki.ros.org/laser_geometry 
     sensor_msgs::PointCloud2 cloud;
     laserProjector.transformLaserScanToPointCloud( "X1/base_link", *scan_in, cloud, tfListener);
     pointCloudPub.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  ROS_INFO("constructed lstopc");
  
  ros::spin();
  
  return 0;
}