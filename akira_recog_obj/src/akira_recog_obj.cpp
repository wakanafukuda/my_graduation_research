#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>

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
  
  void recogObjMainClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_grag_position = nh.advertise <sensor_msgs::PointCloud2> ( "grab_potision", 1 );
    sub_raw_clouds = nh.subscribe ( "/camera/depth_registered/points", 10, &recogObjMainClass::callback, this );
  }

  void recogObjMainClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds )
  {
    
