#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/time_cache.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <iostream>
#include <vector>


/*
class frameTransformListener
{
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  tf::TransformListener listener;

public:
  frameTransformListener ()
  {
    pub = nh.advertise <sensor_msgs::PointCloud2> ( "output", 1 );
    sub = nh.subscribe ( "/camera/depth/points", 1, &frameTransformListener::callback, this );
  }
  
  ~frameTransformListener () { }

  void callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds )
  {
    ROS_INFO ( "called" );
    ros::Time now = ros::Time::now ();
    sensor_msgs::PointCloud::Ptr temp_input_clouds ( new sensor_msgs::PointCloud );
    sensor_msgs::PointCloud::Ptr temp_transformed_clouds1 ( new sensor_msgs::PointCloud );
    sensor_msgs::PointCloud::Ptr temp_transformed_clouds2 ( new sensor_msgs::PointCloud );
    sensor_msgs::PointCloud2::Ptr transformed_clouds ( new sensor_msgs::PointCloud2 );

    try
      {
	sensor_msgs::convertPointCloud2ToPointCloud ( *input_clouds, *temp_input_clouds );
	listener.waitForTransform ( "camera_depth_optical_frame", "camera_link", now, ros::Duration ( 3.0 ) );
	listener.transformPointCloud ( "camera_link", now, *temp_input_clouds, "camera_depth_optical_frame", *temp_transformed_clouds1 );
	listener.waitForTransform ( "camera_link", "base_frame", now, ros::Duration ( 3.0 ) );
	listener.transformPointCloud ( "base_frame", now, *temp_transformed_clouds1, "camera_link", *temp_transformed_clouds2 );
	sensor_msgs::convertPointCloudToPointCloud2 ( *temp_transformed_clouds2, *transformed_clouds );
	pub.publish ( *transformed_clouds );
      }
    catch ( tf::TransformException& ex )
      {
	ROS_ERROR ( "%s", ex.what () );
	ros::Duration ( 1.0 ).sleep ();
      }
  }
};
*/

namespace akira_recog_obj
{
  class frameTransformListener
  {
  public:
    ros::NodeHandle nh;
    ros::Publisher pub_obj;
    ros::Subscriber sub_obj_array;

    tf::TransformListener listener;
    
    frameTransformListener ()
    {
      pub_obj = nh.advertise <sensor_msgs::PointCloud2> ( "akira/tabletop/clusters", 1 );
      sub_obj_array = nh.subscribe ( "/tabletop/clusters", 1, &frameTransformListener::callback, this );
    }

    ~frameTransformListener () { }

    void callback ( const visualization_msgs::MarkerArray::Ptr& input_data )
    {
      ros::Time now = ros::Time::now ();
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_obj_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      std::vector<geometry_msgs::Point> temp;
      std_msgs::Header input_data_header;
      for ( auto it = std::begin ( input_data->markers ) ; it != std::end ( input_data->markers ) ; ++it )
	{
	  input_data_header = it->header;
	  for ( auto its = std::begin ( it->points ) ; its != std::end ( it->points ) ; ++its )
	    {
	      geometry_msgs::Point temp_point;
	      temp_point.x = its->x;
	      temp_point.y = its->y;
	      temp_point.z = its->z;
	      temp.push_back ( temp_point );
	    }
	}
      for ( auto it = temp.begin () ; it != temp.end () ; ++it )
	input_obj_data->push_back ( pcl::PointXYZ ( it->x, it->z, it->z ) );
      
      sensor_msgs::PointCloud2::Ptr temp_obj_pointcloud2 ( new sensor_msgs::PointCloud2 );
      pcl::toROSMsg ( *input_obj_data, *temp_obj_pointcloud2 );
      sensor_msgs::PointCloud::Ptr temp_obj_pointcloud ( new sensor_msgs::PointCloud );
      sensor_msgs::convertPointCloud2ToPointCloud ( *temp_obj_pointcloud2, *temp_obj_pointcloud );
      temp_obj_pointcloud->header = input_data_header;
      sensor_msgs::PointCloud2::Ptr transformed_clouds ( new sensor_msgs::PointCloud2 );

      try
	{
	  sensor_msgs::PointCloud::Ptr temp_transformed_pointcloud_num1 ( new sensor_msgs::PointCloud );
	  sensor_msgs::PointCloud::Ptr temp_transformed_pointcloud_num2 ( new sensor_msgs::PointCloud );
	  listener.waitForTransform ( temp_obj_pointcloud->header.frame_id, "camera_link", now, ros::Duration ( 3.0 ) );
	  listener.transformPointCloud ( "camera_link", now, *temp_obj_pointcloud, temp_obj_pointcloud->header.frame_id, *temp_transformed_pointcloud_num1 );
	  listener.waitForTransform ( "camera_link", "base_frame", now, ros::Duration ( 3.0 ) );
	  listener.transformPointCloud ( "base_frame", now, *temp_transformed_pointcloud_num1, "camera_link", *temp_transformed_pointcloud_num2 );
	  sensor_msgs::convertPointCloudToPointCloud2 ( *temp_transformed_pointcloud_num2, *transformed_clouds );
	  pub_obj.publish ( *transformed_clouds );
	}
      catch ( tf::TransformException& ex )
	{
	  ROS_ERROR ( "%s", ex.what () );
	  ros::Duration ( 1.0 ).sleep ();
	}
    }
  };
}
      


int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_tf_listener" );
  akira_recog_obj::frameTransformListener tl;
  ros::Duration ( 10.0 ).sleep ();

  ros::spin ();
  return 0;
}
