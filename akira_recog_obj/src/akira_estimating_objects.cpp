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
#include <iostream>
#include <algorithm>
#include <vector>
//#include <array>
//*** for c++ stl ***

//*** for ros::topic class ***
#include <ros/topic.h>
//*** for ros::topic class ***

namespace akira_recog_obj
{
  class object_data_set
  {
  public:
    double bottom_y_max;
    double bottom_y_min;
    double up_y_max;
    double up_y_min;
    double x_max;
    double x_min;
    double z_max;
    double z_min;
    std::size_t size;
    
    object_data_set ()
    {
      bottom_y_max = -100;
      bottom_y_min = 100;
      up_y_max = -100;
      up_y_min = 100;
      x_max = -100;
      x_min = 100;
      z_max = -100;
      z_min = 100;
    }

    ~object_data_set () { }
  };
  
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
      //pcl::PointCloud<pcl::PointXYZ>::Ptr obj_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_data ( new pcl::PointCloud<pcl::PointXYZ> () );
      sensor_msgs::PointCloud2::Ptr transformed_output ( new sensor_msgs::PointCloud2 );
      std::vector<geometry_msgs::Point> temp;

      object_recognition_msgs::TableArray::ConstPtr table_array = ros::topic::waitForMessage <object_recognition_msgs::TableArray>( "table_array", nh, ros::Duration ( 2.0 ) );
      if ( !table_array )
	{
	  ROS_INFO ( "No Table Array" );
	}
      else
	{     
	  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_sorted_data ( new pcl::PointCloud<pcl::PointXYZ> () );
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
	    }
	  //std::sort ( temp.begin (), temp.end (), [] ( const geometry_msgs::Point& a, const geometry_msgs::Point& b ) { return a.z < b.z; } );// from min to max	  
	  for ( auto its = temp.begin () ; its != temp.end () ; ++its )
	    {
	      obj_sorted_data->push_back ( pcl::PointXYZ ( its->x, its->y, its->z ) );
	    }

	  object_data_set obj_data;
	  obj_data.size = temp.size ();
	  std::cout << "size: " << obj_data.size << std::endl;
	  //std::size_t obj_data_size = temp.size ();
	  //std::cout << "size: " << obj_data_size << std::endl;
	  std::vector<geometry_msgs::Point>( ).swap ( temp );
	  
	  //geometry_msgs::Pose table_pose = table_array->tables[ 0 ].pose;
	  float theta_y = M_PI / 2, theta_z = - M_PI / 2;
	  Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
	  transform.rotate ( Eigen::AngleAxisf ( theta_y, Eigen::Vector3f::UnitY () ) );
	  transform.rotate ( Eigen::AngleAxisf ( theta_z, Eigen::Vector3f::UnitZ() ) );
	  pcl::transformPointCloud ( *obj_sorted_data, *transformed_data, transform );

	  double bottom_max = -100;
	  double bottom_min = 100;
	  double up_max = -100;
	  double up_min = 100;
	  double center_x_max = -100;
	  double center_x_min = 100;
	  double height_max = -100;
	  double height_min = 100;
	  for ( pcl::PointCloud<pcl::PointXYZ>::iterator it = transformed_data->begin () ; it != transformed_data->end () ; ++it )
	    {
	      /*
	      if ( it->x > center_x_max )
		center_x_max = it->x;
	      else if ( it->x < center_x_min )
		center_x_min = it->x;
	      
	      if ( it->z > height_max )
		height_max = it->z;
	      else if ( it->z < height_min )
		height_min = it->z;
	      */
	      if ( it->x > obj_data.x_max )
		obj_data.x_max = it->x;
	      else if ( it->x < obj_data.x_min )
		obj_data.x_min = it->x;
	      
	      if ( it->z > obj_data.z_max )
		obj_data.z_max = it->z;
	      else if ( it->z < obj_data.z_min )
		obj_data.z_min = it->z;
	    }
	  
	  for ( pcl::PointCloud<pcl::PointXYZ>::iterator it = transformed_data->begin () ; it != transformed_data->end () ; ++it )
	    {
	      /*
	      if ( it->z < ( height_min + 0.02 ) )
		{
		  if ( it->y > bottom_max )
		    bottom_max = it->y;
		  else if ( it->y < bottom_min )
		    bottom_min = it->y;
		}
	      else if ( it->z > ( height_max - 0.02 ) )
		{
		  if ( it->y > up_max )
		    up_max = it->y;
		  else if ( it->y < up_min )
		    up_min = it->y;
		}
	      */
	      if ( it->z < ( obj_data.z_min + 0.02 ) )
		{
		  if ( it->y > obj_data.bottom_y_max )
		    obj_data.bottom_y_max = it->y;
		  else if ( it->y < obj_data.bottom_y_min )
		    obj_data.bottom_y_min = it->y;
		}
	      else if ( it->z > ( obj_data.z_max - 0.02 ) )
		{
		  if ( it->y > obj_data.up_y_max )
		    obj_data.up_y_max = it->y;
		  else if ( it->y < obj_data.up_y_min )
		    obj_data.up_y_min = it->y;
		}
	    }
	  
	  printf ( "x_max: %f, x_min: %f\nbottom_y_max: %f, bottom_y_min: %f\n up_y_max: %f, up_y_min: %f\nz_max: %f, z_min: %f\n", obj_data.x_max, obj_data.x_min, obj_data.bottom_y_max, obj_data.bottom_y_min, obj_data.up_y_max, obj_data.up_y_min, obj_data.z_max, obj_data.z_min );
	  
	  transformed_data->header.frame_id = "camera_link";
	  pcl::toROSMsg ( *transformed_data, *transformed_output );
	  pub_pcl_trans.publish ( *transformed_output );	  
	}
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
