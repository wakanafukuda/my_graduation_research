#include <ros/ros.h>

//*** for messageType ***
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <object_recognition_msgs/TableArray.h>
//*** for messageType ***

//*** for conversion *** 
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
//*** for conversion ***

//*** for affine transform ***
#include <pcl/common/transforms.h>
//*** for affine transform ***

//*** for c++ stl ***
//#include <sstream>
#include <algorithm>
#include <vector>
//#include <array>
//*** for c++ stl ***

//*** for ros::topic class ***
#include <ros/topic.h>
//*** for ros::topic class ***

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
    ros::Publisher pub_pcl_trans;
  
    estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node start." );
      sub_obj_ary = nh.subscribe ( "/tabletop/clusters", 1, &estObjClass::estObjCb, this );
      pub_obj_shape = nh.advertise <std_msgs::String> ( "object_shape", 1 );
      pub_obj_line = nh.advertise <visualization_msgs::Marker> ( "object_line", 1 );
      pub_pcl_obj = nh.advertise <sensor_msgs::PointCloud2> ( "object_pcl", 1 );
      pub_pcl_trans = nh.advertise <sensor_msgs::PointCloud2> ( "trans_pcl", 1 );
    }
    ~estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node stop." );
    }

    void estObjCb ( const visualization_msgs::MarkerArray::Ptr& input_data )
    {
      /*
      std_msgs::String msg;
      std::stringstream ss;
      ss << input_data->markers[ 0 ].header.frame_id;
      msg.data = ss.str ();
      ROS_INFO ( "%s", msg.data.c_str () );

      for ( auto it = std::begin ( input_data->markers ) ; it != std::end ( input_data->markers ) ; ++it )
	{
	  ROS_INFO ( "%f %f %f", it->points[ 0 ].x, it->points[ 0 ].y, it->points[ 0 ].z );
	}
      */
      
      //object_recognition_msgs::TableArray::ConstPtr table_array = ros::topic::waitForMessage <object_recognition_msgs::TableArray>( "table_array", nh, ros::Duration ( 2.0 ) );
      //object_recognition_msgs::TableArray::ConstPtr table_array ( new object_recognition_msgs::TableArray );
      //table_array = ros::topic::waitForMessage <object_recognition_msgs::TableArray> ( "table_array", nh, ros::Duration ( 2.0 ) );
      pcl::PointCloud<pcl::PointXYZ>::Ptr obj_sorted_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      pcl::PointCloud<pcl::PointXYZ>::Ptr obj_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      sensor_msgs::PointCloud2::Ptr transformed_output ( new sensor_msgs::PointCloud2 );
      std::vector<geometry_msgs::Point> temp;

      for ( auto it = std::begin ( input_data->markers ) ; it != std::end ( input_data->markers ) ; ++it )
	{

	  for ( auto its = std::begin ( it->points ) ; its != std::end ( it->points ) ; ++its )
	    {
	      geometry_msgs::Point temp_point;
	      temp_point.x = its->x;
	      temp_point.y = its->y;
	      temp_point.z = its->z;
	      temp.push_back ( temp_point );
	    }
	  std::sort ( temp.begin (), temp.end (), [] ( const geometry_msgs::Point& a, const geometry_msgs::Point& b ) { return a.z < b.z; } );
	  
	  for ( auto its = temp.begin () ; its != temp.end () ; ++its )
	    {
	      obj_sorted_data->push_back ( pcl::PointXYZ ( its->x, its->y, its->z ) );
	    }
	  /*	  
	  geometry_msgs::Point& obj_data_max = temp.back ();
	  geometry_msgs::Point& obj_data_min = temp.front ();
	  double obj_data_middle1 = ( obj_data_max.z - obj_data_min.z ) / 3.0;
	  double obj_data_middle2 = obj_data_middle1 * 2.0;
	  for ( auto its = temp.begin () ; its != temp.end () ; ++its )
	    {
	      if ( ( obj_data_middle1 > its->z ) || ( obj_data_middle2 < its->z ) )
		{
		  obj_data->push_back ( pcl::PointXYZ ( its->x, its->y, its->z ) );
		}
	    }
	  */
	  
	  object_recognition_msgs::TableArray::ConstPtr table_array = ros::topic::waitForMessage <object_recognition_msgs::TableArray>( "table_array", nh, ros::Duration ( 2.0 ) );
	  if ( !table_array )
	    {
	      ROS_INFO ( "No Table Array" );
	    }
	  else
	    {
	      geometry_msgs::Pose table_pose = table_array->tables[ 0 ].pose;
	      float theta = M_PI / 4;
	      Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
	      transform.translation () << -table_pose.position.x, -table_pose.position.y, -table_pose.position.z;
	      transform.rotate ( Eigen::AngleAxisf ( theta, Eigen::Vector3f::UnitY () ) );
	      pcl::transformPointCloud ( *obj_sorted_data, *transformed_data, transform );
	      transformed_data->header.frame_id = "camera_llink";
	      pcl::toROSMsg ( *transformed_data, *transformed_output );
	      pub_pcl_trans.publish ( *transformed_output );
	    }
	}      
      
      obj_sorted_data->header.frame_id = "camera_link";
      sensor_msgs::PointCloud2::Ptr output ( new sensor_msgs::PointCloud2 );
      pcl::toROSMsg ( *obj_sorted_data, *output );
      pub_pcl_obj.publish ( *output );

      std::vector<geometry_msgs::Point>( ).swap ( temp );
      //delete obj_data;
      //delete output;
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
