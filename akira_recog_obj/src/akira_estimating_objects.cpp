#include <ros/ros.h>

#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <sstream>
#include <vector>
#include <array>

namespace akira_recog_obj
{
  class estObjClass
  {
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub_obj_ary;
    ros::Publisher pub_obj_shape;
    ros::Publisher pub_obj_line;
    ros::Publisher pub_pcl_obj;
  
    estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node start." );
      sub_obj_ary = nh.subscribe ( "/tabletop/clusters", 1, &estObjClass::estObjCb, this );
      pub_obj_shape = nh.advertise <std_msgs::String> ( "object_shape", 1 );
      pub_obj_line = nh.advertise <visualization_msgs::Marker> ( "object_line", 1 );
      pub_pcl_obj = nh.advertise <sensor_msgs::PointCloud2> ( "object_pcl", 1 );
    }
    ~estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node stop." );
    }

    void estObjCb ( const visualization_msgs::MarkerArray::Ptr& raw_data )
    {
      /*
      std_msgs::String msg;
      std::stringstream ss;
      ss << raw_data->markers[ 0 ].header.frame_id;
      msg.data = ss.str ();
      ROS_INFO ( "%s", msg.data.c_str () );

      for ( auto it = std::begin ( raw_data->markers ) ; it != std::end ( raw_data->markers ) ; ++it )
	{
	  ROS_INFO ( "%f %f %f", it->points[ 0 ].x, it->points[ 0 ].y, it->points[ 0 ].z );
	}
      */
      std::vector<geometry_msgs::Pose> obj_pose;
      pcl::PointCloud<pcl::PointXYZ>::Ptr obj_data ( new pcl::PointCloud<pcl::PointXYZ> );
      for ( auto it = std::begin ( raw_data->markers ) ; it != std::end ( raw_data->markers ) ; ++it )
	{
	  obj_pose.push_back ( it->pose );
	  for ( auto its = std::begin ( it->points ) ; its != std::end ( it->points ) ; ++its )
	    {
	      obj_data->push_back ( pcl::PointXYZ ( its->x, its->y, its->z ) );
	    }
	}
      
      obj_data->header.frame_id = "camera_link";
      sensor_msgs::PointCloud2::Ptr output ( new sensor_msgs::PointCloud2 );
      pcl::toROSMsg ( *obj_data, *output );
      pub_pcl_obj.publish ( *output );
    }
  };
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_estimating_objects" );
  akira_recog_obj::estObjClass obj;

  ros::spin ();
  return 0;
}
