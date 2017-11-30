#ifndef AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS
#define AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS

#include <stdio.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace akira_recog_obj
{
  class detectTableClass : public nodelet::Nodelet
  {
  public:
    detectTableClass ();
    ~detectTableClass ();

    ros::Publisher pub_raw_obj;
    //ros::Publisher pub_raw_table;
    //ros::Publisher pub_table_coefficients;
    ros::Subscriber sub_raw_cloud;

    double begin;
    double now;
    double before;
    double sum_time;

    int32_t counter;

    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input_clouds );

  };
}

#endif

/*****
ワンタイムインクルード(インクルードガード)
ヘッダファイルが複数回インクルードされるのを防ぐ

#ifndef, #define の後の文字列は他のヘッダファイルと重複しないものを指定する．
2度目以降インクルードされるときは，最初の #ifndef ではじかれる．
*****/
