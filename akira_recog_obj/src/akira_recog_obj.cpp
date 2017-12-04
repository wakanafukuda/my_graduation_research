#include "akira_recog_obj/akira_recog_obj.h"

namespace akira_recog_obj
{
  
  recogObjMainClass::recogObjMainClass ()
  {
    ROS_INFO ( "akira recognize objects nodelet start" );
  }

  recogObjMainClass::~recogObjMainClass ()
  {
    ROS_INFO ( "akira recognize objects nodelet stop" );
  }

  ros::Publisher pub_grab_position;
  ros::Subscriber sub_raw_clouds;

  bool makeFilter;
  bool tableIsDetected;

  //  table_filter table_filter;
  
  void recogObjMainClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_grab_position = nh.advertise <sensor_msgs::PointCloud2> ( "grab_position", 1 );
    sub_raw_clouds = nh.subscribe ( "/camera/depth_registered/points", 10, &recogObjMainClass::callback, this );

    makeFilter = false;
    tableIsDetected = false;
  }

  void recogObjMainClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds )
  {
    sensor_msgs::PointCloud2::Ptr output ( new sensor_msgs::PointCloud2 );
    if ( makeFilter )
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	filtering_clouds2 ( input_clouds, filtered_clouds );
      }
    else
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_table_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	filtering_clouds1 ( input_clouds, filtered_clouds );
	detecting_table ( filtered_clouds, extracted_table_clouds );
	if ( tableIsDetected )
	  {
	    //making_filter ( extracted_table_clouds );
	    pcl::toROSMsg ( *extracted_table_clouds, *output );
	    pub_grab_position.publish ( *output );
	    ROS_INFO ( "result: %d", output->is_dense );
 	  }
      }
  }
  
  void recogObjMainClass::filtering_clouds1 ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr uncut_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_voxeled_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_clouds, *uncut_clouds );
    passthrough_filter ( uncut_clouds, "z", 2.5, 0, noisy_clouds );
    noise_filter ( noisy_clouds, no_voxeled_clouds );
    voxel_grid ( no_voxeled_clouds, filtered_clouds );    
  }

  void recogObjMainClass::filtering_clouds2 ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
  }

  void recogObjMainClass::detecting_table ( pcl::PointCloud<pcl::PointXYZ>::Ptr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& extracted_table_clouds )
  {
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_PERPENDICULAR_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( 1.5 );
    seg.setAxis ( Eigen::Vector3f ( 0.0, 1.0, 0.0 ) );
    seg.setEpsAngle ( 90.0f * ( M_PI / 180.0f ) );
    seg.setMaxIterations ( 300 );
    seg.setInputCloud ( input_clouds->makeShared () );
    seg.segment ( *inliers, *coefficients );

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    if ( inliers->indices.size () == 0 )
      {
	std::cerr << "No inliers." << std::endl;
	tableIsDetected = false;
      }
    else
      {
	tableIsDetected = true;
	extract.setInputCloud ( input_clouds );
	extract.setIndices ( inliers );
	extract.setNegative ( false );
	extract.filter ( *extracted_table_clouds );
     }
  }

  void recogObjMainClass::making_filter ( pcl::PointCloud<pcl::PointXYZ>::Ptr& input_clouds )
  {
    //    table_filter temp_data;
    //    temp_data.max.setAll ( 10 );
    //    temp_data.min.setAll ( -10 );
    
  }

  void recogObjMainClass::noise_filter ( pcl::PointCloud<pcl::PointXYZ>::Ptr& noisy_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& no_noisy_clouds )
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud ( noisy_clouds );
    sor.setMeanK ( 50 );
    sor.setStddevMulThresh ( 1.0 );
    sor.filter ( *no_noisy_clouds );
  }

  void recogObjMainClass::voxel_grid ( pcl::PointCloud<pcl::PointXYZ>::Ptr& no_voxeled_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& voxeled_clouds )
  {
    sensor_msgs::PointCloud2::Ptr converted1_no_voxeled_clouds ( new sensor_msgs::PointCloud2 );
    pcl::toROSMsg ( *no_voxeled_clouds, *converted1_no_voxeled_clouds );
    pcl::PCLPointCloud2::Ptr converted2_no_voxeled_clouds ( new pcl::PCLPointCloud2 );
    pcl::PCLPointCloud2ConstPtr converted2_no_voxeled_cloudsPtr ( converted2_no_voxeled_clouds );
    pcl::PCLPointCloud2 raw_voxeled_clouds;
    pcl_conversions::toPCL( *converted1_no_voxeled_clouds, *converted2_no_voxeled_clouds );
    pcl::VoxelGrid<pcl::PCLPointCloud2> vgf;
    vgf.setInputCloud ( converted2_no_voxeled_cloudsPtr );
    vgf.setLeafSize ( 0.02, 0.02, 0.02 );
    vgf.filter ( raw_voxeled_clouds );
    pcl::fromPCLPointCloud2 ( raw_voxeled_clouds, *voxeled_clouds );
  }

  void recogObjMainClass::passthrough_filter ( pcl::PointCloud<pcl::PointXYZ>::Ptr& uncut_clouds, std::string axis, double max, double min, pcl::PointCloud<pcl::PointXYZ>::Ptr& cut_clouds )
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud ( uncut_clouds );
    pass.setFilterFieldName ( axis );
    pass.setFilterLimits ( min, max );
    pass.filter ( *cut_clouds );
  }
}

PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::recogObjMainClass, nodelet::Nodelet )
