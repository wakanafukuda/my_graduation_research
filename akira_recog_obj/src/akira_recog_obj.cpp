#include "akira_recog_obj/akira_recog_obj.h"

namespace akira_recog_obj
{
  recogObjMainClass::recogObjMainClass ()
  {
    ROS_INFO ( "akira recognize objects nodelet start" );
  }

  recogObjMainClass::~recogObjMainClass ()
  {
    ROS_INFO ( "akira recognize objects nodelet stop" );
  }

  ros::Publisher pub_grab_position;
  ros::Subscriber sub_raw_clouds;

  bool makeFilter;
  
  void recogObjMainClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_grab_position = nh.advertise <sensor_msgs::PointCloud2> ( "grab_potision", 1 );
    sub_raw_clouds = nh.subscribe ( "/camera/depth_registered/points", 10, &recogObjMainClass::callback, this );

    makeFilter = false;
  }

  void recogObjMainClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds )
  {
    if ( !makeFilter )
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    
	filtering_clouds1 ( input_clouds, filtered_clouds );
	pub_grab_position.publish ( *filtered_clouds );
      }
    else
      {
	//
      }
  }
  
  void recogObjMainClass::filtering_clouds1 ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_clouds, *noisy_clouds );
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud ( noisy_clouds );
    sor.setMeanK ( 50 );
    sor.setStddevMulThresh ( 1.0 );
    sor.filter ( *filtered_clouds );
  }
  
}

PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::recogObjMainClass, nodelet::Nodelet )
