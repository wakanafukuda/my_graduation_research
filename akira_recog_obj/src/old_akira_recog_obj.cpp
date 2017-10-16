//akira_recog_obj/akira_recog_obj.h無しで動くプログラム

#include <stdio.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

namespace akira_recog_obj
{
  class recogObjMainClass : public nodelet:Nodelet
  {
  public:
    recogObjMainClass ()
    {}

  private:
    virtual void onInit ()
    {
      ros::NodeHandle& private_nh = getPrivateNodeHandle ();
      pub = private_nh.advertise <sensor_msgs::PointCloud2> ( "out", 1 );
      sub = private_nh.subscribe ( "/camera/depth/points", 10, &recogObjMainClass::callback, this );
    }

    void callback ( const sensor_msgs::PointCloud2::ConstPtr& input )
    {
      sensor_msgs::PointCloud2 output;
      output = *input;
      pub.publish ( output );//pub.publish ( *input );
    }

    ros::Publisher pub;
    ros::Subscriber sub;
  };

  PLUGINLIB_EXPORT_CLASS ( akira_recog_obj::recogObjMainClass, nodelet::Nodelet );
}

//"nodelet/Tutorials/Porting nodes to nodelets"の差分
//2015年以降は PLUGINLIB_DECLARE_CLASS の代わりに PLUGINLIB_EXPORT_CLASS が採用されている
//http://wiki.ros.org/nodelet/Tutorials/Porting%20nodes%20to%20nodelets?action=diff&rev1=17&rev2=18
//  PLUGINLIB_DECLARE_CLASS ( akira_recog_obj, recogObjMainClass, akira_recog_obj::recogObjMainClass, nodelet::Nodelet );
//
//  PLUGINLIB_EXPORT_CLASS ( PLUGIN_PACKAGE_NAME::PLUGIN_CLASS_NAME, BASE_PACKAGE_NAME::BASE_CLASS_NAME )
//多分，PLUGIN_PACKAGE_NAME は PLUGIN_CLASS_NAME の存在する名前空間
