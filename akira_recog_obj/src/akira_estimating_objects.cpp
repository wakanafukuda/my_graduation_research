#include <ros/ros.h>
#include <stdlib.h>

//*** for mkdir ***
#include <sys/stat.h>
#include <sys/types.h>
//*** for mkdir ***

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
#include <string>
#include <cmath>
#include <fstream>
//*** for c++ stl ***

//*** for ros::topic class ***
#include <ros/topic.h>
//*** for ros::topic class ***

namespace akira_recog_obj
{
  class object_data_set
  {
  public:
    std::string name;
    double bottom_y_max, bottom_y_min;
    double bottom_diameter, bottom_radius;
    double up_y_max, up_y_min;
    double up_diameter, up_radius;
    double x_max, x_min;
    double depth;
    double z_max, z_min;
    double height;
    double angle;
    double bottom_center_x, bottom_center_y;
    double up_center_x, up_center_y;
    std::size_t size;
    
    object_data_set ()
    {
      name = "no_name";
      bottom_y_max = -100; bottom_y_min = 100; bottom_diameter = 0;
      up_y_max = -100; up_y_min = 100; up_diameter = 0;
      x_max = -100; x_min = 100; depth = 0;
      z_max = -100; z_min = 100; height = 0;
      angle = 0;
      bottom_center_x = 0; bottom_center_y = 0;
      up_center_x = 0; up_center_y = 0;
    }

    ~object_data_set () { }

    void calc_parameters ()
    {
      bottom_diameter = ( std::abs ( bottom_y_max - bottom_y_min ) ) * 100;
      bottom_radius = bottom_diameter / 2;
      up_diameter = ( std::abs ( up_y_max - up_y_min ) ) * 100;
      up_radius = up_diameter / 2;
      depth = ( std::abs ( x_max - x_min ) ) * 100;
      height = ( std::abs ( z_max - z_min ) ) * 100;
      bottom_center_x = up_center_x = ( x_max + x_min ) / 2;
      bottom_center_y = ( bottom_y_max + bottom_y_min ) / 2;
      up_center_y = ( up_y_max + up_y_min ) / 2;

      if ( std::abs ( bottom_diameter - up_diameter ) < 1.0 )
	{
	  name.erase ( name.begin (), name.end () );
	  name = "clinder";
	  angle = 90;
	}
      else if ( ( bottom_diameter - up_diameter ) > 1.0 )
	{
	  name.erase ( name.begin (), name.end () );
	  name = "truncated_cone";
	  angle = atan2 ( height, ( bottom_radius - up_radius ) );
	  angle = angle * ( 180 / M_PI );
	}
      else if ( ( up_diameter - bottom_diameter ) > 1.0 )
	{
	  name.erase ( name.begin (), name.end () );
	  name = "reverse_truncated_cone";
	  angle = atan2 ( height, ( up_radius - bottom_radius ) );
	  angle = 180 - angle * ( 180 / M_PI );
	}
    }
    
    void write_data ( std::string& dir_name, std::string& filename )
    {
      std::ofstream ofs;
      ofs.open ( dir_name + filename, std::ios::app );
      if ( !ofs )
	{
	  std::cout << "This program couldn't open " << filename << std::endl;
	  exit ( 1 );
	}
      else
	{
	  if ( filename == "/bottom_diameter.txt" )
	    ofs << bottom_diameter << std::endl;
	  else if ( filename == "/up_diameter.txt" )
	    ofs << up_diameter << std::endl;
	  else if ( filename == "/depth.txt" )
	    ofs << depth << std::endl;
	  else if ( filename == "/height.txt" )
	    ofs << height << std::endl;
	  else if ( filename == "/angle.txt" )
	    ofs << angle << std::endl;
	  else if ( filename == "/name.txt" )
	    ofs << name << std::endl;
	  else
	    std::cout << "This program won't prepare such a file." << std::endl;	  
	}
    }
    
  };
  
  class estObjClass
  {
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub_obj_ary;
    ros::Publisher pub_pcl_trans;
    object_data_set obj_data;
    
    std::string dir_name;
    std::string bottom_diameter_file;
    std::string up_diameter_file;
    std::string depth_file;
    std::string height_file;
    std::string angle_file;
    std::string obj_name_file;
  
    estObjClass ( std::string& temp_dir_name )
    {
      ROS_INFO ( "akira estimating objects node start." );
      sub_obj_ary = nh.subscribe ( "/tabletop/clusters", 1, &estObjClass::estObjCb, this );
      pub_pcl_trans = nh.advertise <sensor_msgs::PointCloud2> ( "trans_pcl", 1 );
      dir_name = temp_dir_name;
      bottom_diameter_file = "/bottom_diameter.txt";
      up_diameter_file = "/up_diameter.txt";
      depth_file = "/depth.txt";
      height_file = "/height.txt";
      angle_file = "/angle.txt";
      obj_name_file = "/name.txt";
    }
    
    ~estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node stop." );
    }

    void estObjCb ( const visualization_msgs::MarkerArray::Ptr& input_data )
    {
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
	  for ( auto its = temp.begin () ; its != temp.end () ; ++its )
	    {
	      obj_sorted_data->push_back ( pcl::PointXYZ ( its->x, its->y, its->z ) );
	    }

	  //object_data_set obj_data;
	  obj_data.size = temp.size ();
	  std::cout << "size: " << obj_data.size << std::endl;
	  std::vector<geometry_msgs::Point>( ).swap ( temp );
	  
	  //geometry_msgs::Pose table_pose = table_array->tables[ 0 ].pose;
	  float theta_y = M_PI / 2, theta_z = - M_PI / 2;
	  Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
	  transform.rotate ( Eigen::AngleAxisf ( theta_y, Eigen::Vector3f::UnitY () ) );
	  transform.rotate ( Eigen::AngleAxisf ( theta_z, Eigen::Vector3f::UnitZ() ) );
	  pcl::transformPointCloud ( *obj_sorted_data, *transformed_data, transform );

	  for ( pcl::PointCloud<pcl::PointXYZ>::iterator it = transformed_data->begin () ; it != transformed_data->end () ; ++it )
	    {
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

	  obj_data.calc_parameters ();
	  std::cout << "bottom_diameter: " << obj_data.bottom_diameter << ", up_diameter: " << obj_data.up_diameter << std::endl;
	  std::cout << "depth: " << obj_data.depth << ", height: " << obj_data.height << std::endl;
	  if ( obj_data.name.length () > 0 )
	    std::cout << "name: " << obj_data.name << ", angle: " << obj_data.angle << std::endl;
	  obj_data.write_data ( dir_name, bottom_diameter_file );
	  obj_data.write_data ( dir_name, up_diameter_file );
	  obj_data.write_data ( dir_name, depth_file );
	  obj_data.write_data ( dir_name, height_file );
	  obj_data.write_data ( dir_name, angle_file );
	  obj_data.write_data ( dir_name, obj_name_file );
	  
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

  std::string dir_name( "data" );
  const char* dir_name_ptr = dir_name.c_str ();
  for ( int i = 1 ; ; ++i )
    {
      dir_name = dir_name + ( std::to_string ( i ) );
      struct stat buf;
      if ( ( stat ( dir_name_ptr, &buf ) ) == 0 )
	{
	  std::cout << "Directory " << dir_name << " exists." << std::endl;
	  for ( std::string::reverse_iterator rit = dir_name.rbegin () ; rit != dir_name.rend () ; ++rit )
	    {
	      if ( *rit == 'a' )
		break;
	      else
		dir_name.pop_back ();
	    }
	}
      else
	{
	  std::cout << "Directory " << dir_name << " doesn't exits. This program will make it." << std::endl;
	  if ( ( mkdir ( dir_name_ptr, 0755 ) ) == 0 )
	    {
	      std::cout << "Directory " << dir_name << " was made." << std::endl;
	      break;
	    }
	  else
	    {
	      std::cout << "Directory " << dir_name << " couldn't be made. This program will shutdown." << std::endl;
	      exit ( 1 );
	    }
	}
    }
	
  akira_recog_obj::estObjClass obj ( dir_name );

  ros::spin ();
  return 0;
}
