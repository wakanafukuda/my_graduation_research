#ifndef AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS
#define AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS

#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace akira_recog_obj
{
  class detectTableClass
  {
  public:
    detectTableClass();
    ~detectTableClass();

    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers_index;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers_data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outliers_data;
    
    int presentNumber;
    int thisIsTable;


  };
}

#endif
