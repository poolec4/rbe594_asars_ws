#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

class LaserScanToPointCloud {

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  pcl::PCLPointCloud2 full_cloud;

  LaserScanToPointCloud(ros::NodeHandle n): 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_, tf_, "world", 10)
  {
    laser_notifier_.registerCallback( boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud",1);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    sensor_msgs::PointCloud2 cloud;
    
    try {
        projector_.transformLaserScanToPointCloud("world", *scan_in, cloud, tf_);
    }
    catch (tf::TransformException& e) {
        std::cout << e.what();
        return;
    }

	pcl::PCLPointCloud2 newPoints;
	pcl_conversions::toPCL(cloud, newPoints);
	pcl::concatenate(full_cloud, newPoints, full_cloud);
	pcl_conversions::fromPCL(full_cloud, cloud);

    scan_pub_.publish(cloud);

  }


	// TODO: make function to rasterize point cloud, transform to height map, and export


};


int main(int argc, char** argv){
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  ros::spin();
  
  return 0;
}