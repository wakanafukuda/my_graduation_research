#ifndef AKIRA_RECOG_OBJ_H_CLUSTERING_CLOUDS_CLASS
#define AKIRA_RECOG_OBJ_H_CLUSTERING_CLOUDS_CLASS

#include <stdio.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

namespace akira_recog_obj
{
  class clusteringCloudsClass : public nodelet::Nodelet
  {
  public:
    clusteringCloudsClass ();
    ~clusteringCloudsClass ();

    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& raw_input_object );
  };
}
#endif
