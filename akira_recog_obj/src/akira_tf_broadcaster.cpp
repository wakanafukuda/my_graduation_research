#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_tf_broadcaster" );
  ros::NodeHandle nh;

  tf::TransformBroadcaster br_camera_to_base_frame;
  tf::TransformBroadcaster br_base_frame_to_base_link;
  tf::Transform transform_camera_to_base_frame;
  tf::Transform transform_base_frame_to_base_link;
  
  ros::Rate rate( 10.0 );
  while ( nh.ok () )
    {
      transform_camera_to_base_frame.setOrigin ( tf::Vector3 ( 0.0, 0.0, -0.5 ) );//x y z
      transform_camera_to_base_frame.setRotation ( tf::createQuaternionFromRPY ( 0, 50, 0 ) );//roll pitch yaw
      br_camera_to_base_frame.sendTransform ( tf::StampedTransform ( transform_camera_to_base_frame, ros::Time::now (), "camera_link", "base_frame" ) );
      
      transform_base_frame_to_base_link.setOrigin ( tf::Vector3 ( 0.0, 0.0, -1.0 ) );
      transform_base_frame_to_base_link.setRotation ( tf::createQuaternionFromRPY ( 0, 0, 0 ) );
      br_base_frame_to_base_link.sendTransform ( tf::StampedTransform ( transform_base_frame_to_base_link, ros::Time::now (), "base_frame", "base_link" ) );
      
      rate.sleep ();
    }
  return 0;
}
