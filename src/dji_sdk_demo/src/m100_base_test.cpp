#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

#include <stdio.h>

int mode = 0;

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller_test");
    ros::NodeHandle n;
   

    DJIDrone* drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);
//    drone->takeoff();
     while ( ros::ok() )
     {
      ros::spinOnce();
      switch (mode)
      {
	case 0:
	{
	  if (drone->flight_status==1)//standby
	  {  
	    drone->takeoff();
	    printf("takeoff...\n");
	  }
	  else if(drone->flight_status==3)//in air
	  {
	    printf("inair...\n");
	    drone->attitude_control ( 0x9B,0,0,3,0 );
	    if(drone->local_position.z>2.6)	
	    {
	      printf("get top...\n");
	      mode = 1;
	    }
	  }
	  else
	  {
	    printf("arming...\n");
	  }
	  break;
	}
	case 1:
	{
	  printf("landing...\n");
	  drone->attitude_control ( 0x9B,0,0,0,0 );
// 	  if(drone->flight_status==3)
// 	  {
// 	    printf("inair...\n");
// 	  }
	  if(drone->local_position.z<0.02)	
	  {
	    printf("get bottom...\n");
	    mode = 2;
	  }
	  break;
	}
	case 2:
	{
	  printf("takeoff...\n");
	  drone->attitude_control ( 0x9B,0,0,1.2,0 );
	  break;
	}
	default : break;
      }
      
     }
//     sleep(8);
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x5c, 0, 0, 2.5, 20);//NOT 0X42
// 	else
// 	    drone->attitude_control(0x4c, 0, 0, 0, 0);
// 	usleep(30000);
//     }
//    sleep(1);
    //VERTICAL_VELOCITY+HORIZONTAL_VELOCITY+YAW_RATE+HORIZONTAL_BODY+YAW_BODY+SMOOTH_ENABLE
    //= 0x00 + 0x40 + 0x08 + 0x02 + 0X01 + 0x01 = 0x4c
    
    //VERTICAL_POSITION+HORIZONTAL_VELOCITY+YAW_RATE+HORIZONTAL_BODY+YAW_BODY+SMOOTH_ENABLE
    //= 0x10 + 0x40 + 0x08 + 0x02 + 0X01 + 0x01 = 0x5c
    
    //VERTICAL_POSITION+HORIZONTAL_POSITION+YAW_RATE+HORIZONTAL_BODY+YAW_BODY+SMOOTH_ENABLE
    //= 0x10 + 0x80 + 0x08 + 0x02 + 0X01 + 0x01 = 0x9c
    
    //compare HORIZONTAL YAW and default N
    //v contorl
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x4c, 0, 1, 0, 0);//NOT 0X42
// 	else
// 	    drone->attitude_control(0x4c, 0, 0, 0, 0);
// 	usleep(20000);
//     }
//     
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x4b, 0, 1, 0, 0);//NOT 0X42
// 	else
// 	    drone->attitude_control(0x4b, 0, 0, 0, 0);
// 	usleep(20000);
//     }
//     
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x40, 0, 1, 0, 0);
// 	else
// 	    drone->attitude_control(0x40, 0, 0, 0, 0);
// 	usleep(20000);
//     }

    //pos control
//      for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x9c, 0, 1, 1.5, 0);//NOT 0X42
// 	else
// 	    drone->attitude_control(0x4c, 0, 0, 0, 0);
// 	usleep(20000);
//     }
//     
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x9b, 0, 1, 1.5, 0);//NOT 0X42
// 	else
// 	    drone->attitude_control(0x4b, 0, 0, 0, 0);
// 	usleep(20000);
//     }
    
//     for(int i = 0; i < 100; i ++)
//     {
// 	if(i < 90)
// 	    drone->attitude_control(0x90, 0, 1, 1.5, 0);
// 	else
// 	    drone->attitude_control(0x40, 0, 0, 0, 0);
// 	usleep(20000);
//     }

//     int i = 0;
// `
//     while(n.ok()) {
//        ros::spinOnce();
//         i ^= 1;
//         if (i == 0) {
//             drone->attitude_control(0x5b, 0, 1, 0, 0);
// 	    sleep(5);
//         }
//         else {
//             drone->attitude_control(0x5b, 0, -1, 0, 0);
// 	    sleep(5);
//         }
//     }
    return 0;
}