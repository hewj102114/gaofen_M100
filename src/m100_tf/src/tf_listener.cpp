#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener) {
    geometry_msgs::PointStamped guidance_point;
    guidance_point.header.frame_id = "m100_guidance";

    // we'll just use the most recent transform available for our simple example
    guidance_point.header.stamp = ros::Time();

    // just an arbitrary point in space
    guidance_point.point.x = 1.0;
    guidance_point.point.y = 0.2;
    guidance_point.point.z = 0.0;

    try {
        geometry_msgs::PointStamped base_point;
        listener.transformPoint("m100_base", guidance_point, base_point);

        ROS_INFO("m100_guidance: (%.2f, %.2f. %.2f) -----> m100_base: (%.2f, %.2f, %.2f) at time %.2f",
                 guidance_point.point.x, guidance_point.point.y, guidance_point.point.z,
                 base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"m100_guidance\" to \"m100_base\": %s", ex.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "m100_listener");
    ros::NodeHandle n;

    tf::TransformListener listener(ros::Duration(10));

    // we'll transform a point once every second
    ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

    ros::spin();

}
