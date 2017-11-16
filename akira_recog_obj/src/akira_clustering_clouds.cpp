#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "akira_recog_obj/akira_clustering_clouds.h"

namespace akira_recog_obj
{
  clusteringCloudsClass::clusteringCloudsClass ()
  {
    ROS_INFO ( "akira_clustering_clouds nodelet start" );
  }
  
  clusteringCloudsClass::~clusteringCloudsClass ()
  {
    ROS_INFO ( "akira_clustering_clouds nodelet stop" );
  }

  ros::Subscriber sub_raw_obj;
  ros::Publisher pub_random_obj;
  
  void clusteringCloudsClass::onInit ()
  {
    ros::NodeHandle& nh = getMTNodeHandle ();
    sub_raw_obj = nh.subscribe ( "/out_obj" , 10, &clusteringCloudsClass::callback, this );
    pub_random_obj = nh.advertise <sensor_msgs::PointCloud2> ( "oub_random_obj", 1 );
  }
  
  void clusteringCloudsClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_cloud )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_object ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_cloud, *input_object );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> );
    tree->setInputCloud ( input_object );
    
    std::vector<pcl::PointIndices> object_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance ( 0.02 );//2cm
    ec.setMinClusterSize ( 100 );
    ec.setMaxClusterSize ( 25000 );
    ec.setSearchMethod ( tree );
    ec.setInputCloud ( input_object );
    ec.extract ( object_indices );

    /*
    for ( std::vector<pcl::PointIndices>::const_iterator it = object_indices.begin () ; it != object_indices.end (); ++it )
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster ( new pcl::PointCloud<pcl::PointXYZ> );
	for ( std::vector<int>::const_iterator pit = it->indices.begin () ; pit != it->indices.end () ; ++pit )
	  {
	    object_cluster->points.push_back ( input_object->points[ *pit ] );
	  }
	object_cluster->width = object_cluster->points.size ();
	object_cluster->height = 1;
	object_cluster->is_dense = true;
	
	sensor_msgs::PointCloud2::Ptr out_random_obj ( new sensor_msgs::PointCloud2 );
	pcl::toROSMsg ( *object_cluster, *out_random_obj );
	pub_random_obj.publish ( *out_random_obj );
      }
    */

    std::vector<pcl::PointIndices>::const_iterator it = object_indices.begin ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster ( new pcl::PointCloud<pcl::PointXYZ> );
    for ( std::vector<int>::const_iterator pit = it->indices.begin () ; pit != it->indices.end () ; ++pit )
      {
	object_cluster->points.push_back ( input_object->points[ *pit ] );
      }
    object_cluster->width = object_cluster->points.size ();
    object_cluster->height = 1;
    object_cluster->is_dense = true;

    sensor_msgs::PointCloud2::Ptr out_random_obj ( new sensor_msgs::PointCloud2 );
    pcl::toROSMsg ( *object_cluster, *out_random_obj );
    pub_random_obj.publish ( *out_random_obj );
    
  }
}

PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::clusteringCloudsClass, nodelet::Nodelet )
