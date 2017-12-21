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

  tf::TransformListener tl_camera_to_base_link;
  
  bool makeFilter;
  bool tableIsDetected;

  table_filter* t_filter;
  table_filter* t_filter_data; 
  int* makeFilterCounter;
  
  void recogObjMainClass::onInit ()
  {
    ros::NodeHandle& nh = getNodeHandle ();
    pub_grab_position = nh.advertise <sensor_msgs::PointCloud2> ( "grab_position", 1 );
    sub_raw_clouds = nh.subscribe ( "/camera/depth_registered/points", 10, &recogObjMainClass::callback, this );

    makeFilter = false;
    tableIsDetected = false;

    t_filter = new table_filter;
    t_filter_data = new table_filter[ 10 ];
    makeFilterCounter = new int;
    *makeFilterCounter = 0;
  }

  void recogObjMainClass::callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds )
  {
    sensor_msgs::PointCloud2::Ptr output ( new sensor_msgs::PointCloud2 );
    sensor_msgs::PointCloud2::Ptr transformed_clouds ( new sensor_msgs::PointCloud2 );
    transform_pointclouds ( "camera_link", input_clouds, "base_link", transformed_clouds );

    if ( makeFilter )
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	filtering_clouds2 ( transformed_clouds, filtered_clouds );
	ROS_INFO ( "success making filter" );
	pcl::toROSMsg ( *filtered_clouds, *output );
	pub_grab_position.publish ( *output );
      }
    else
      {
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_table_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
	filtering_clouds1 ( transformed_clouds, filtered_clouds );
	detecting_table ( filtered_clouds, extracted_table_clouds );
	if ( tableIsDetected )
	  {
	    pcl::toROSMsg ( *extracted_table_clouds, *output );
	    pub_grab_position.publish ( *output );
	    making_filter ( extracted_table_clouds );
 	  }
      }
  }
  
  void recogObjMainClass::filtering_clouds1 ( sensor_msgs::PointCloud2::Ptr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr uncut_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_voxeled_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxeled_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_clouds, *uncut_clouds );
    passthrough_filter ( uncut_clouds, "z", 2.0, 0, noisy_clouds );
    noise_filter ( noisy_clouds, no_voxeled_clouds );
    voxel_grid ( no_voxeled_clouds, filtered_clouds );
    
  }

  void recogObjMainClass::filtering_clouds2 ( sensor_msgs::PointCloud2::Ptr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_transformed_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr uncut_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_clouds_x ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cut_clouds_y ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cut_clouds ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::fromROSMsg ( *input_clouds, *uncut_clouds );
    
    passthrough_filter ( uncut_clouds, "x", t_filter->max.getX (), t_filter->min.getX (), cut_clouds_x );
    passthrough_filter ( cut_clouds_x, "y", t_filter->min.getY (), ( t_filter->min.getY () - 1.5 ), cut_clouds_y );
    passthrough_filter ( cut_clouds_y, "z", t_filter->max.getZ (), t_filter->min.getZ (), noisy_clouds );
    noise_filter ( noisy_clouds, filtered_clouds );
  }

  void recogObjMainClass::detecting_table ( pcl::PointCloud<pcl::PointXYZ>::Ptr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& extracted_table_clouds )
  {
    pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients ( true );
    seg.setModelType ( pcl::SACMODEL_PERPENDICULAR_PLANE );
    seg.setMethodType ( pcl::SAC_RANSAC );
    seg.setDistanceThreshold ( 0.02 );//1.5
    seg.setAxis ( Eigen::Vector3f ( 0.0, 1.0, 0.0 ) );
    seg.setEpsAngle ( 30.0f * ( M_PI / 180.0f ) );
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
    if ( *makeFilterCounter < 10 )
      {
	t_filter_data[ *makeFilterCounter ].max.setAll ( -100 );
	t_filter_data[ *makeFilterCounter ].min.setAll ( 100 );
	
	for ( pcl::PointCloud<pcl::PointXYZ>::iterator pit = input_clouds->points.begin () ; pit != input_clouds->points.end () ; ++pit )
	  {
	    if ( pit->x > t_filter_data[ *makeFilterCounter ].max.getX () )
	      t_filter_data[ *makeFilterCounter ].max.setX ( pit->x );
	    if ( pit->x < t_filter_data[ *makeFilterCounter ].min.getX () )
	      t_filter_data[ *makeFilterCounter ].min.setX ( pit->x );
	    if ( pit->y > t_filter_data[ *makeFilterCounter ].max.getY () )
	      t_filter_data[ *makeFilterCounter ].max.setY ( pit->y );
	    if ( pit->y < t_filter_data[ *makeFilterCounter ].min.getY () )
	      t_filter_data[ *makeFilterCounter ].min.setY ( pit->y );
	    if ( pit->z > t_filter_data[ *makeFilterCounter ].max.getZ () )
	      t_filter_data[ *makeFilterCounter ].max.setZ ( pit->z );
	    if ( pit->z < t_filter_data[ *makeFilterCounter ].min.getZ () )
	      t_filter_data[ *makeFilterCounter ].min.setZ ( pit->z );
	  }
	*makeFilterCounter += 1;
	ROS_INFO ( "Counter: %d", *makeFilterCounter );
      }
    else if ( *makeFilterCounter == 10 )
      {
	double temp_data;
	for ( int i = 0 ; i < 9 ; i++ )
	  {
	    for ( int j = 9 ; j > i + 1 ; j-- )
	      {
		if ( t_filter_data[ j ].max.getX () > t_filter_data[ j - 1 ].max.getX () )
		  {
		    temp_data = t_filter_data[ j ].max.getX ();
		    t_filter_data[ j ].max.setX ( t_filter_data[ j - 1 ].max.getX () );
		    t_filter_data[ j - 1 ].max.setX ( temp_data );
		  }
		if ( t_filter_data[ j ].min.getX () > t_filter_data[ j - 1 ].min.getX () )
		  {
		    temp_data = t_filter_data[ j ].min.getX ();
		    t_filter_data[ j ].min.setX ( t_filter_data[ j - 1 ].min.getX () );
		    t_filter_data[ j - 1 ].min.setX ( temp_data );
		  }
		if ( t_filter_data[ j ].max.getY () > t_filter_data[ j - 1 ].max.getY () )
		  {
		    temp_data = t_filter_data[ j ].max.getY ();
		    t_filter_data[ j ].max.setY ( t_filter_data[ j - 1 ].max.getY () );
		    t_filter_data[ j - 1 ].max.setY ( temp_data );
		  }
		if ( t_filter_data[ j ].min.getY () > t_filter_data[ j - 1 ].min.getY () )
		  {
		    temp_data = t_filter_data[ j ].min.getY ();
		    t_filter_data[ j ].min.setY ( t_filter_data[ j - 1 ].min.getY () );
		    t_filter_data[ j - 1 ].max.setY ( temp_data );
		  }
		if ( t_filter_data[ j ].max.getZ () > t_filter_data[ j - 1 ].max.getZ () )
		  {
		    temp_data = t_filter_data[ j ].max.getZ ();
		    t_filter_data[ j ].max.setZ ( t_filter_data[ j - 1 ].max.getZ () );
		    t_filter_data[ j - 1 ].max.setZ ( temp_data );
		  }
		if ( t_filter_data[ j ].min.getZ () > t_filter_data[ j - 1 ].min.getZ () )
		  {
		    temp_data = t_filter_data[ j ].min.getZ ();
		    t_filter_data[ j ].min.setZ ( t_filter_data[ j - 1 ].min.getZ () );
		    t_filter_data[ j - 1 ].min.setZ ( temp_data );
		  }
	      }
	  }
	t_filter->max.setX ( t_filter_data[ 4 ].max.getX () );
	t_filter->min.setX ( t_filter_data[ 4 ].min.getX () );
	t_filter->max.setY ( t_filter_data[ 4 ].max.getY () );
	t_filter->min.setY ( t_filter_data[ 4 ].min.getY () );
	t_filter->max.setZ ( t_filter_data[ 4 ].max.getZ () );
	t_filter->min.setZ ( t_filter_data[ 4 ].min.getZ () );
	delete[] t_filter_data;
	ROS_INFO ( "pushed table filter value" );
	makeFilter = true;
      }
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

  void transform_pointclouds ( std::string target_frame, const sensor_msgs::PointCloud2::ConstPtr& input_clouds, std::string fixed_frame, sensor_msgs::PointCloud2::Ptr& transformed_clouds )
  {
    sensor_msgs::PointCloud temp_input_clouds;
    sensor_msgs::PointCloud::Ptr temp_transformed_clouds ( new sensor_msgs::PointCloud );
    try
      {
	sensor_msgs::convertPointCloud2ToPointCloud ( *input_clouds, temp_input_clouds );
	tl_camera_to_base_link.transformPointCloud ( target_frame, ros::Time ( 0 ), temp_input_clouds, fixed_frame, *temp_transformed_clouds );
      }
    catch ( tf::TransformException ex )
      {
	ROS_ERROR ( "%s", ex.what () );
	ros::Duration ( 1.0 ).sleep ();
      }
    sensor_msgs::convertPointCloudToPointCloud2 ( *temp_transformed_clouds, *transformed_clouds );
  }
}

PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::recogObjMainClass, nodelet::Nodelet )
