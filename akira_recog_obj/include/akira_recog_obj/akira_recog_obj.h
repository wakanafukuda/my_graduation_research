#ifndef AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS
#define AKIRA_RECOG_OBJ_H_RECOG_OBJ_MAIN_CLASS

#include <stdio.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

namespace akira_recog_obj
{
  class recogObjMainClass : public nodelet::Nodelet
  {
  public:
    recogObjMainClass ();
    ~recogObjMainClass ();

    ros::Publisher pub_grab_position;
    ros::Subscriber sub_raw_clouds;

    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds );

  };
}

#endif
