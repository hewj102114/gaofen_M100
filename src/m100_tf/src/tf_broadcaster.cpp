#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<std_msgs/Bool.h>

static bool start_searching = false;
void start_searching_callback(const std_msgs::Bool & start_)
{
  start_searching = start_.data;

}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "m100_publisher" ); // give a name
  ros::NodeHandle n;

  ros::Rate r ( 70 );

  tf::TransformBroadcaster broadcaster;

  const double pi = 3.1415926535;

  ros::Subscriber start_searching= n.subscribe ( "/dji_sdk_demo/start_searching",10,start_searching_callback );
  while ( n.ok() )
    {
      ros::spinOnce();
      if(start_searching)
      {
      broadcaster.sendTransform (
        tf::StampedTransform (
          tf::Transform ( tf::Quaternion ( pi / 2, 0, -pi / 2 ), tf::Vector3 ( 0.0, 0.0, 0.0 ) ),
          ros::Time::now(),"base_link", "m100_guidance" ) );
      }

      r.sleep();
    }
}
