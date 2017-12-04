#ifndef AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS
#define AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS

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

#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/filters/filter.h>

namespace akira_recog_obj
{
  class recogObjMainClass : public nodelet::Nodelet
  {
  public:
    recogObjMainClass ();
    ~recogObjMainClass ();

    ros::Publisher pub_grab_position;
    ros::Subscriber sub_raw_clouds;

    bool makeFilter;
    
    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds );

    void filtering_clouds1 ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_clouds );
  };
}

#endif
