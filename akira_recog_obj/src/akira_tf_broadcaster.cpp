#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>
#include <object_recognition_msgs/TableArray.h>

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_tf_broadcaster" );
  ros::NodeHandle nh;

  //tf::TransformBroadcaster br_base_frame_to_base_link;
  //tf::TransformBroadcaster br_base_link_to_camera_link;
  //tf::Transform transform_base_frame_to_base_link;
  //tf::Transform transform_base_link_to_camera_link;

  tf::TransformBroadcaster br_camera_link_to_table;
  tf::Transform tf_camera_link_to_table;
  
  ros::Rate rate ( 5.0 );
  while ( nh.ok () )
    {
      //transform_base_frame_to_base_link.setRotation ( tf::createQuaternionFromRPY ( 0, 0, 0 ) );
      //transform_base_frame_to_base_link.setOrigin ( tf::Vector3 ( 0.0, 0.0, 1.0 ) ); // x y z
      //br_base_frame_to_base_link.sendTransform ( tf::StampedTransform ( transform_base_frame_to_base_link, ros::Time::now (), "base_frame", "base_link" ) );
      //transform_base_link_to_camera_link.setRotation ( tf::createQuaternionFromRPY ( 0, ( 3.14 / 180 ) * 50, 0 ) );
      //transform_base_link_to_camera_link.setOrigin ( tf::Vector3 ( 0.0, 0.0, 0.2 ) );
      //br_base_link_to_camera_link.sendTransform ( tf::StampedTransform ( transform_base_link_to_camera_link, ros::Time::now (), "base_link", "camera_link" ) );

      object_recognition_msgs::TableArray::ConstPtr table_array = ros::topic::waitForMessage <object_recognition_msgs::TableArray> ( "table_array", nh, ros::Duration ( 2.0 ) );
      if ( !table_array )
	{
	  ROS_INFO ( "No Table Array" );
	}
      else
	{
	  geometry_msgs::Pose table_pose = table_array->tables[ 0 ].pose;
	  tf_camera_link_to_table.setRotation ( tf::Quaternion ( table_pose.quaternion.x, table_pose.quaternion.y, table_pose.quaternion.z ) );
	  tf_camera_link_to_table.setOrigin ( tf::Vector3 ( table_pose.position.x, table_pose.position.y, table_pose.position.z ) );
	  br_camera_link_to_table.sendTransform ( tf::StampedTransform ( tf_camera_link_to_table, ros::Time::now (), "camera_link", "table" ) );

	}
      rate.sleep ();
    }
  return 0;
}
