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

    ros::Subscriber sub_raw_obj;
    ros::Publisher pub_random_obj;
    
    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input_cloud );
  };
}
#endif
