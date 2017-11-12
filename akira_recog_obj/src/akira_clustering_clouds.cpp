#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include "akira_recog_obj/akira_clustering_clouds.h"

namespace akira_recog_obj
{
  class clusteringCloudsClass : public nodelet::Nodelet
  {
  public:
    clusteringCloudsClass ()
    {
      ROS_INFO ( "akira_clustering_clouds nodelet start" );
    }

    ~clusteringCloudsClass ()
    {
      ROS_INFO ( "akira_clustering_clouds nodelet stop" );
    }

    ros::Subscriber sub_raw_obj;
    
    void onInit ()
    {
      ros::NodeHandle& nh = getNodeHandle ();
      sub_raw_obj = nh.subscribe ( "/out_obj" , 10, &clusteringCloudsClass::callback, this );
    }

    void callback ( const sensor_msgs::PointCloud2::ConstPtr& raw_input_object )
    {
      
