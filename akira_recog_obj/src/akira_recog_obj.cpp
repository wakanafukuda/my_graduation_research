#include <stdio.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/filter.h>

#include "akira_recog_obj/akira_recog_obj.h"

namespace akira_recog_obj
{
  recogObjMainClass::recogObjMainClass ()
  {
    ROS_INFO ( "akira_recog_obj nodelet start" );
  }
  
  recogObjMainClass::~recogObjMainClass ()
  {
    ROS_INFO ( "akira_recog_obj nodelet stop" );
  }
  
  ros::Publisher pub_obj;
  ros::Publisher pub_table;
  ros::Subscriber sub;
  
  void recogObjMainClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_obj = nh.advertise <sensor_msgs::PointCloud2> ( "out_obj" , 1 );
    pub_table = nh.advertise <sensor_msgs::PointCloud2> ( "out_table" , 1 );
    sub = nh.subscribe ( "/camera/depth_registered/points", 10, &recogObjMainClass::callback, this );
  }
  
  void recogObjMainClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_cloud )
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noisy_cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::fromROSMsg ( *input_cloud, *noisy_cloud );
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud ( noisy_cloud );
    sor.setMeanK ( 50 );
    sor.setStddevMulThresh ( 1.0 );
    sor.filter ( *noisy_cloud );
    sensor_msgs::PointCloud2::Ptr noise_filtered_cloud ( new sensor_msgs::PointCloud2 );
    pcl::toROSMsg ( *noisy_cloud, *noise_filtered_cloud );

    
    pcl::PCLPointCloud2::Ptr raw_cloud ( new pcl::PCLPointCloud2 );
    pcl::PCLPointCloud2ConstPtr raw_cloudPtr ( raw_cloud );
    pcl::PCLPointCloud2 raw_cloud_filtered;//消すとコアダンプ起きる
    //    pcl_conversions::toPCL ( *input_cloud, *raw_cloud );//ノイズ除去無の時の変換
    pcl_conversions::toPCL ( *noise_filtered_cloud, *raw_cloud );
    pcl::VoxelGrid<pcl::PCLPointCloud2> vgf;
    vgf.setInputCloud ( raw_cloudPtr );
    vgf.setLeafSize ( 0.01, 0.01, 0.01 );//値を小さくすると細かくなる
    vgf.filter ( raw_cloud_filtered );
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxeled_cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::fromPCLPointCloud2 ( raw_cloud_filtered, *voxeled_cloud );
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );

    pcl::SACSegmentation <pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_PARALLEL_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( 0.05 );
    seg.setAxis ( Eigen::Vector3f ( 1.0, 1.0, 0.0 ) );
    seg.setInputCloud ( voxeled_cloud->makeShared () );
    seg.segment ( *inliers, *coefficients );
    
    pcl_msgs::ModelCoefficients::Ptr ros_coefficients ( new pcl_msgs::ModelCoefficients );
    pcl_conversions::fromPCL ( *coefficients, *ros_coefficients );
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud ( new pcl::PointCloud<pcl::PointXYZRGB> );

    sensor_msgs::PointCloud2::Ptr out_obj ( new sensor_msgs::PointCloud2 );
    sensor_msgs::PointCloud2::Ptr out_table ( new sensor_msgs::PointCloud2 );

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
	
	for ( size_t i = 0 ; i < object_cloud->points.size () ; i++ )
	  {
	    object_cloud->points[ i ].r = 255;
	    object_cloud->points[ i ].g = 0;
	    object_cloud->points[ i ].b = 0;
	  }
	
	extract.setNegative ( false );
	extract.filter ( *table_cloud );
	
	for ( size_t i = 0 ; i < table_cloud->points.size () ; i++ )
	  {
	    table_cloud->points[ i ].r = 0;
	    table_cloud->points[ i ].g = 255;
	    table_cloud->points[ i ].b = 0;
	  }

	pcl::toROSMsg ( *object_cloud, *out_obj );
	pcl::toROSMsg ( *table_cloud, *out_table );

	pub_obj.publish ( *out_obj );
	pub_table.publish ( *out_table );
      }
    
  }
}
    
    PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::recogObjMainClass, nodelet::Nodelet )

//"nodelet/Tutorials/Porting nodes to nodelets"の差分
//2015年以降は PLUGINLIB_DECLARE_CLASS の代わりに PLUGINLIB_EXPORT_CLASS が採用されている
//http://wiki.ros.org/nodelet/Tutorials/Porting%20nodes%20to%20nodelets?action=diff&rev1=17&rev2=18
//  PLUGINLIB_DECLARE_CLASS ( akira_recog_obj, recogObjMainClass, akira_recog_obj::recogObjMainClass, nodelet::Nodelet );
//
//  PLUGINLIB_EXPORT_CLASS ( PLUGIN_PACKAGE_NAME::PLUGIN_CLASS_NAME, BASE_PACKAGE_NAME::BASE_CLASS_NAME )
//多分，PLUGIN_PACKAGE_NAME は PLUGIN_CLASS_NAME の存在する名前空間

//pcl::PointIndices::indicesはstd::vector<int>だから
//sizeでベクトルのサイズを参照できる
//多分inliersに入ってるのは，inliersであると判断された点の順番

//pcl::toROSMsg ( *cloud, *output );は，
//pcl::PCLPointCloud2::Ptr pcl_output ( new pcl::PCLPointCloud2 );
//pcl::toPCLPointCLoud2 ( *cloud, *pcl_output );
//pcl_conversions::fromPCL ( *pcl_output, *output );
//と同じ．
