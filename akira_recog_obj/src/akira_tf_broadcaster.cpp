#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_tf_broadcaster" );
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate( 10.0 );
  while ( nh.ok () )
    {
      transform.setOrigin ( tf::Vector3 ( 0.0, 0.0, -1.5 ) );//x y z
      transform.setRotation ( tf::createQuaternionFromRPY( 0, -10, 0 ) );//roll pitch yaw
      br.sendTransform ( tf::StampedTransform ( transform, ros::Time::now (), "camera_link", "base_link" ) );

      rate.sleep ();
    }
  return 0;
}
