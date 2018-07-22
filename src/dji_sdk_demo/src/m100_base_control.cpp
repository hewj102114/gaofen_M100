/** @file m100_base_contorol.cpp

 *
 */

#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>
#include <dji_sdk/dji_drone.h>
#include <pthread.h>
#include<sensor_msgs/LaserScan.h>

DJIDrone* drone;

float cmd_vel_x = 0.0, cmd_vel_y = 0.0 ,cmd_vel_z = 0.0;
float fly_height_control = 1.5;
float ob_guidance[5] = {1.5,10,10,10,10};
void obstacle_distance_callback(const sensor_msgs::LaserScanConstPtr& g_oa) {
     for ( int i=0; i<5; i++ )
     {
       printf( " %f ", g_oa->ranges[i]);
       ob_guidance[i]=g_oa->ranges[i];
     }
}

// void base_controller_callback(const geometry_msgs::TwistConstPtr& msg) {
//     cmd_vel_x = msg->linear.x;
//     cmd_vel_y = msg->linear.y;
//     cmd_vel_z = msg->angular.z;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;

    drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
    ros::Subscriber ob_guidance_sub =  n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

    drone->takeoff();
    sleep(1);

    ros::Rate r(50);
 
 //   ros::Subscriber sub = n.subscribe("cmd_vel", 10, base_controller_callback);

    int pitch_rate = 50;
    int yaw_rate = 50;

    drone->gimbal_angle_control(0,0,0,10,0);
    while (n.ok())
    {
      ros::spinOnce();
      // usleep(10);
      if(ob_guidance[0]<1.5)
      {
	fly_height_control += 0.005;        
	
      }
      else if(ob_guidance[0]>1.8)
      {
	fly_height_control -= 0.005;        
      }
	
//	drone->attitude_control(0x5b, cmd_vel_x, cmd_vel_y, fly_height_control, cmd_vel_z * 57.3);
      drone->attitude_control(0x5b, 0, 0, fly_height_control, 0);//HORIZ_VEL+YAW_RATE+HORIZ_BODY+STABLE_ON=0x5b

      pitch_rate = drone->gimbal.pitch>15? -50:(drone->gimbal.pitch>-89? pitch_rate:50);
      yaw_rate = drone->gimbal.yaw>90? -50:(drone->gimbal.yaw>-90?yaw_rate:50);

      drone->gimbal_speed_control(0,pitch_rate,yaw_rate);//gimbal_angle_control ( 0, ( int ) drone->flight_gimbal_angle_pitch_inc*10,0,10,0 );

      r.sleep();
    }
        
    ros::spinOnce();
    return 0;
}

