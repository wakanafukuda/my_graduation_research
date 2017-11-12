#ifndef AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS
#define AKIRA_RECOG_OBJ_H_DETECT_TABLE_CLASS

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
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

    ros::Publisher pub_obj;
    ros::Publisher pub_table;
    ros::Publisher pub_coefficients;
    ros::Subscriber sub;

    int counter;
    std_msgs::Float32** temp_data;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl_msgs::ModelCoefficients::Ptr ros_coefficients;
    
    virtual void onInit ();
    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input );

  };
}

#endif

/*****
ワンタイムインクルード(インクルードガード)
ヘッダファイルが複数回インクルードされるのを防ぐ

#ifndef, #define の後の文字列は他のヘッダファイルと重複しないものを指定する．
2度目以降インクルードされるときは，最初の #ifndef ではじかれる．
*****/
