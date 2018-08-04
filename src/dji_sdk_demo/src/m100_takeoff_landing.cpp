#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include<fstream>
#include <stdio.h>

#include <sensor_msgs/LaserScan.h>

ofstream writeF ( "/home/ubuntu/GaofenChallenge/log_lh.txt");

static float ob_distance[5]= {10.0};

/*obstacle_distance from guidance*/
/*0-down, 1-forward,2-right,3-backward,4-left*/
void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
    for ( int i=0; i<5; i++ )
        ob_distance[i]=g_oa.ranges[i];
    ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
}

int flag = 0;
int mode = 0;
float set_flight_height = 2.0;
float flying_height_control_tracking = 2.0;

float set_height = 1.4;
float flying_height_control = 1.4;

int count_to_landing = 200;


int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controller_test");
    ros::NodeHandle n;

    DJIDrone* drone = new DJIDrone(n);
    drone->request_sdk_permission_control();
    sleep(1);

    ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

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
				    if ( ob_distance[0]<set_flight_height-0.1)
				    {
					flying_height_control_tracking += 0.003;
				    }
				    if ( (ob_distance[0]>set_flight_height+0.1)&&ob_distance[0]<5)
				    { 
					flying_height_control_tracking -= 0.003;
				    }
				    drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);
				    
				    writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log
				    
				    if((ob_distance[0]>set_flight_height - 0.3)&&ob_distance[0]<4)	
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
				// descend to 1.2m
				printf("landing...\n");
// 				if ( ob_distance[0]< 0.9)
// 				{
// 				   flying_height_control_tracking += 0.01;
// 				}
// 				if((ob_distance[0]> 1.2)&&(ob_distance[0]< 5))
// 				{ 
// 				   flying_height_control_tracking -= 0.01;
// 				}
// 				drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);
				
				sleep(5);
				
				writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log

				if(abs(ob_distance[0] - 1.7) < 0.1)				  
				{	
				  
				    writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log
				    
				    for(int i = 0; i < count_to_landing; i ++)
				    {
					if(i < (count_to_landing-5))
					   drone->attitude_control(0x4B, 0, 0, -0.5, 0);//NOT 0X42
					else
					   drone->attitude_control(0x4B, 0, 0, 0, 0);
					
					usleep(10000);										
				    }
					printf("land...\n");
					mode = 2;
				}
				break;
			}
			case 2:
			{
				printf("takeoff...\n");
				writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log
				
// 				for(int i = 0; i < count_to_landing; i ++)
// 				{
// 				    if(i < (count_to_landing-5))
// 					drone->attitude_control(0x4B, 0, 0, 0.6, 0);//NOT 0X42
// 				    else
// 					drone->attitude_control(0x4B, 0, 0, 0, 0);
// 				    
// 				    usleep(10000);										
// 				}
                                
                                if(flag == 0)
				{
                                    drone->attitude_control(0x5B,0,0,2,0);
				}
                            				
				if ( (ob_distance[0] > set_height)&& (ob_distance[0] < 4))
				{
				    drone->attitude_control(0x4B,0,0,0,0);
				    flag = 1;
				}
				
				writeF<<"flying_height_control="<<flying_height_control<<endl;   // write in log
				    
				break;
			}
			default : break;
		}
      
     }
    return 0;
}