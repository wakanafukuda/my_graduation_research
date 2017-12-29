#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/time_cache.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

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
    
int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_tf_listener" );
  frameTransformListener tl;
  ros::Duration ( 10.0 ).sleep ();

  ros::spin ();
  return 0;
}
