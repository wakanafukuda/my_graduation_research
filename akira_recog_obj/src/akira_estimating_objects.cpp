#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
#include <array>

namespace akira_recog_obj
{
  class estObjClass
  {
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub_obj_ary;
    ros::Publisher pub_obj_shape;
    ros::Publisher pub_obj_line;
  
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

    void estObjCb ( const visualization_msgs::MarkerArray::Ptr& dt )
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << dt->markers[ 0 ].header.frame_id;
      msg.data = ss.str ();
      ROS_INFO ( "%s", msg.data.c_str () );

      int length = 0;
      for ( auto it = std::begin ( dt->markers ) ; it != std::end ( dt->markers ) ; ++it )
	{
	  ROS_INFO ( "%d", ++length );
	}
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
