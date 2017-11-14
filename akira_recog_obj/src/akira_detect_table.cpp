#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include "akira_recog_obj/akira_detect_table.h"

namespace akira_recog_obj
{
  detectTableClass::detectTableClass ()
  {
    ROS_INFO ( "akira_detect_table nodelet start" );
  }
  
  detectTableClass::~detectTableClass ()
  {
    ROS_INFO ( "akira_detect_table nodelet stop" );
  }
  
  ros::Publisher pub_raw_obj;
  //ros::Publisher pub_raw_table;
  //ros::Publisher pub_table_coefficients;
  ros::Subscriber sub_raw_cloud;

  double begin;
  double now;
  double before;
  double sum_time;

  int32_t counter;
  
  void detectTableClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_raw_obj = nh.advertise <sensor_msgs::PointCloud2> ( "out_obj" , 1 );
    //pub_raw_table = nh.advertise <sensor_msgs::PointCloud2> ( "out_table" , 1 );
    sub_raw_cloud = nh.subscribe ( "/camera/depth_registered/points", 10, &detectTableClass::callback, this );

    begin = ros::Time::now().toSec();
    now = 0;
    before = 0;
    sum_time = 0;
    counter = 0;
  }
  
  void detectTableClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_cloud )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_cloud, *noisy_cloud );
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud ( noisy_cloud );
    sor.setMeanK ( 50 );
    sor.setStddevMulThresh ( 1.0 );
    sor.filter ( *noisy_cloud );
    sensor_msgs::PointCloud2::Ptr noise_filtered_cloud ( new sensor_msgs::PointCloud2 );
    pcl::toROSMsg ( *noisy_cloud, *noise_filtered_cloud );
    
    pcl::PCLPointCloud2::Ptr raw_cloud ( new pcl::PCLPointCloud2 );
    pcl::PCLPointCloud2ConstPtr raw_cloudPtr ( raw_cloud );
    pcl::PCLPointCloud2 raw_cloud_filtered;//消すとコアダンプ起きる
    pcl_conversions::toPCL ( *noise_filtered_cloud, *raw_cloud );
    
    pcl::VoxelGrid<pcl::PCLPointCloud2> vgf;
    vgf.setInputCloud ( raw_cloudPtr );
    vgf.setLeafSize ( 0.01, 0.01, 0.01 );//値を小さくすると細かくなる
    vgf.filter ( raw_cloud_filtered );    
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromPCLPointCloud2 ( raw_cloud_filtered, *voxeled_cloud );

    pcl::PointCloud<pcl::PointXYZ>::Ptr through_z ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud ( voxeled_cloud );
    pass.setFilterFieldName ( "z" );
    pass.setFilterLimits ( 0, 0.8 );
    pass.filter ( *through_z );
    
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    //pcl_msgs::ModelCoefficients::Ptr ros_coefficients ( new pcl_msgs::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::SACSegmentation <pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_PERPENDICULAR_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( 1.5 );//0.05
    seg.setAxis ( Eigen::Vector3f ( 0.0, 1.0, 0.0 ) );// frame_id: camera_depth_optical_frame
    seg.setEpsAngle ( 90.0f * ( M_PI / 180.0f ) ); //30.0f * ( M_PI / 180.0f )
    seg.setMaxIterations ( 300 );//500
    //seg.setInputCloud ( voxeled_cloud->makeShared () );
    seg.setInputCloud ( through_z->makeShared () );
    seg.segment ( *inliers, *coefficients );
    
    //pcl_conversions::fromPCL ( *coefficients, *ros_coefficients );
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    //pcl::PointCloud<pcl::PointXYZ>::Ptr table_cloud ( new pcl::PointCloud<pcl::PointXYZ> );

    sensor_msgs::PointCloud2::Ptr out_obj ( new sensor_msgs::PointCloud2 );
    //sensor_msgs::PointCloud2::Ptr out_table ( new sensor_msgs::PointCloud2 );

    if ( inliers->indices.size () == 0 )
      {
	std::cerr << "No inliers." << std::endl;
      }
    else
      {
	extract.setInputCloud ( voxeled_cloud );
	extract.setIndices ( inliers );
	extract.setNegative ( true );
	extract.filter ( *object_cloud );
	
	//extract.setNegative ( false );
	//extract.filter ( *table_cloud );
	
	pcl::toROSMsg ( *object_cloud, *out_obj );
	//pcl::toROSMsg ( *table_cloud, *out_table );

	pub_raw_obj.publish ( *out_obj );
	//pub_raw_table.publish ( *out_table );

	if ( counter < 500 )
	  {
	    now = ros::Time::now().toSec();
	    if ( counter == 0 )
	      {
		sum_time += ( now - begin );
	      }
	    else if ( counter > 0 )
	      {
		sum_time += ( now - before );
	      }
	    before = now;
	    ++counter;
	    ROS_INFO ( "1 image is processed per %f sec", ( sum_time / counter ) );
	  }
      }
  }
}

PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::detectTableClass, nodelet::Nodelet )
