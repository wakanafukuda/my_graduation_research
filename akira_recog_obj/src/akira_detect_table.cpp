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

    pcl::PointCloud<pcl::PointXYZ>::Ptr through_x;
    pcl::PointCloud<pcl::PointXYZ>::Ptr through_y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr through_z;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud ( voxeled_cloud );
    pass.setFilterFieldName ( "x" );
    pass.setFilterLimits ( 0.0, 2.0 );
    pass.filter ( *through_x );

    pass.setInputCloud ( through_x );
    pass.setFilterFieldName ( "y" );
    pass.setFilterLimits ( -1.0, 1.0 );
    pass.filter ( *through_y );

    pass.setInputCloud ( through_y );
    pass.setFilterFieldName ( "z" );
    pass.setFilterLimits ( -1.0, 1.0 );
    pass.filter ( *through_z );
    
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    //pcl_msgs::ModelCoefficients::Ptr ros_coefficients ( new pcl_msgs::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::SACSegmentation <pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_PERPENDICULAR_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( 0.05 );
    seg.setAxis ( Eigen::Vector3f ( 0.0, 1.0, 0.0 ) );// frame_id: camera_depth_optical_frame
    seg.setEpsAngle ( 30.0f * ( M_PI / 180.0f ) );
    seg.setMaxIterations ( 500 );
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

// SACsegmentation に関して
// 1 の答えの人
// https://answers.ros.org/question/61811/pcl-sacsegmentation-setaxis-and-setmodeltype-has-no-effect-in-output/
// eps_parameter が必要．axis は isModelValid () 関数でしか使われていない．
// さらに eps_angle_ > 0 の時だけ使われる．( sac_model_parallel_plane.hpp より )
// eps_angle_ は 0 に初期化されるから，このパラメータを明示的にセットしなければ
// axis は無視される．( sac_model_parallel_plane.h より )
//
// setEpsAngle () は「モデルの法線ベクトルと与えられた軸とでの最大許容範囲」とあるから
// axis 変数は平面の法線ベクトルを期待している．
//
// 0 の答えの人
// XZ 平面がほしいので，
// pcl::SACMODEL_PERPENDICULAR_PLANE ( 与えられた軸に対して垂直 )
// seg.setMaxIterations ( 500 ) ( デフォルトが 50 回と少ない <- キーポイント )
// seg.setDistanceThreshold ( 0.05 ) ( 平面上の 0.05m 以内のものを採用 )
// Eigen::Vector3f ( 0.0, 1.0, 0.0 )
// seg.setEpsAngle ( 30.0f * ( M_PI / 180.0f ) ) ( XZ 平面に対して 30 度以内は平面 )
