#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace akira_recog_obj
{
  ros::NodeHandle nh;
  ros::Subscriber sub_obj_ary;
  ros::Publisher pub_obj_shape;
  ros::Publisher pub_obj_line;
  
  class estObjClass
  {
  public:
    estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node start." );
      sub_obj_ary = nh.subscribe ( "/tabletop/clusters", 1, &estObjClass::estObjCb, this );
      pub_obj_shape = nh.advertise <std_msgs::String> ( "object_shape", 1 );
      pub_obj_line = nh.advertise <visualization_msgs::Marker> ( "object_line", 1 );
    }
    ~estObjClass ()
    {
      ROS_INFO ( "akira estimating objects node stop." );
    }

    void estObjCb ( const visualization_msgs::MarkerArray& msg )
    {
      ROS_INFO ( "get object" );
    }
  };
}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "akira_estimating_objects" );
  akira_recog_obj::estObjClass obj;

  ros::spin ();
  return 0;
}
