#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>

#include <stdio.h>
#include<math.h>
#include <cstdlib>
#include<std_msgs/Int8.h>
#include<std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h> // motion
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include<fstream> //log

#define GIMBAL_USED
#define filter_N 4   
#define EPS 0.0000000001
#define USE_OBDIST  
   
#define PI 3.1415926536 
#define VEL_MODE

static float ob_distance[5]= {10.0};
/*obstacle_distance from guidance*/
/*0-down, 1-forward,2-right,3-backward,4-left*/
void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
    for ( int i=0; i<5; i++ )
        ob_distance[i]=g_oa.ranges[i];
    ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
}

static bool Detection_fg = false;
void tag_detection_resultCallback(const std_msgs::Bool & num_of_detection )
{
    Detection_fg = num_of_detection.data;
    //ROS_INFO ( "%d tag(s) are detected",numOfDetections );
}

static float g_pos_x = 0.0;
static float g_pos_y = 0.0;
static float g_pos_z = 0.0;
/* motion */
/* postion relative to original points*/
float start_yaw = 0.0;
void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
    // g_pos_x=cos(start_yaw)*g_pos.vector.x-sin(start_yaw)*g_pos.vector.y;
    // g_pos_y=sin(start_yaw)*g_pos.vector.x+cos(start_yaw)*g_pos.vector.y;
    // g_pos_z=g_pos.vector.z;

    g_pos_x=g_pos.vector.x;
    g_pos_y=g_pos.vector.y;
    g_pos_z=g_pos.vector.z;
//    printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
//    printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}

int mode = 0;
float serach_flight_height_1 = 3.0;// for numbers
float serach_flight_height_2 = 1.2;// for tags
float flying_height_control_tracking = 3.0;
int count_to_landing = 280;      // if landoff height is 3.0, count_to_landing = 280;
int samll_adjust_count = 0;//for a small scall adjust flight xy
int count_forward=0;// go forward count

//just now
int count_to_forward = 100;
bool take_off_flag = 0;


int delay_count = 300;
static float last_flight_x = 0.0;
static float last_flight_y = 0.0;  
static float last_flight_yaw = 0.0;
static uint8_t land_mode = 0;   
static int cbring_mode = 0; 

ofstream writeF ( "/home/ubuntu/GaofenChallenge/log_lh.txt");

float sum(float a[],size_t len);
float find_min(float a[], size_t len);
float find_max(float a[], size_t len);
bool parking(float &cmd_fligh_x,
	     float &cmd_flight_y,
	     float height,
	     uint8_t detection_flag,
	     DJIDrone* drone);
bool cbarrier(float &cmd_fligh_x,
	      float &cmd_flight_y, 
	      float &cmd_yaw, 
	      float height,
	      float set_height, 
	      uint8_t detection_flag,
	      DJIDrone* drone,
	      int &delayCount);

int main(int argc, char **argv) 
{
    ros::init ( argc, argv, "main_base" );
    ROS_INFO ( "gaofen_main_base_test" );
    ros::NodeHandle n;

    DJIDrone* drone = new DJIDrone(n);
    //virtual RC test data
    writeF<<ros::Time::now() <<endl;
    
    ros::Publisher  mission_type_pub = n.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );  
    ros::Publisher start_searching_pub = n.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );
    ros::Publisher state_in_mission_pub = n.advertise<std_msgs::Int8>("/dji_sdk_demo/state_in_mission",10);
    ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );
    ros::Subscriber position_sub = n.subscribe("/guidance/position", 10, position_callback);
    ros::Subscriber tag_detections_sub = n.subscribe ( "tag_detection/detections",100,tag_detection_resultCallback );
    
    //for easy to adjust height
    flying_height_control_tracking = serach_flight_height_1;
    //For filtering;
    float filtered_x=0.0,filtered_y=0.0, filtered_yaw=0.0;
    float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};

    int time_count=0;
    int count=0;// go forward count

//    float start_yaw = 0.0;
    bool flip_once = false;//mission success flag

    //public datas
    int state_in_mission = 0;
    std_msgs::Int8 state_msg;
    state_msg.data = state_in_mission;//0~9 state
    std_msgs::Bool start_searching;//numbers/tags
    start_searching.data= false;
    std_msgs::Bool mission_type;//numbers/tags
    mission_type.data=false; //false for round 3, true for round 4

    int searching_state = 0;
    float bound_len = 1.0;//adjust bound lenght avoid getting out

    ros::Rate loop_rate ( 50 );
    //Head down at the beginning.row,picth,yaw,duration
    drone->gimbal_angle_control ( 0,-900,0,20 ); 

    while ( ros::ok() )
    {
	ros::spinOnce();
		
        // for numbers setting
        start_searching_pub.publish ( start_searching );
        mission_type_pub.publish ( mission_type );

        // get bias values
        for ( int i = 0; i< filter_N-1; i++ )
        {
            filter_seq_x[i] = filter_seq_x[i+1];
            filter_seq_y[i] = filter_seq_y[i+1];
        }
        filter_seq_x[filter_N-1] = drone->flight_x;
        filter_seq_y[filter_N-1] = drone->flight_y;
        last_flight_yaw = filtered_yaw;
        filtered_x =  (( sum ( filter_seq_x,filter_N )-find_max ( filter_seq_x,filter_N )-find_min ( filter_seq_x,filter_N ) ) / ( filter_N-2 ))*0.8;
        filtered_y =  (( sum ( filter_seq_y,filter_N )-find_max ( filter_seq_y,filter_N )-find_min ( filter_seq_y,filter_N ) ) / ( filter_N-2 ))*0.8;
        filtered_yaw= drone->flight_yaw;
        if(abs(filtered_yaw-last_flight_yaw)>20.0)   
        filtered_yaw = last_flight_yaw;
        if(filtered_yaw>10) 
        filtered_yaw=10;
        if(filtered_yaw<-10) 
        filtered_yaw=-10;
      
        if (drone->rc_channels.mode != 8000)
            continue;
        else
        {
            drone->request_sdk_permission_control();
        }
        if (drone->rc_channels.gear==-10000)
        {
            mode=1;
            state_in_mission=-1;
            count=0;
            time_count=0;
            cbring_mode=0;
            land_mode=0;
        }
        else
        {
            if(mode)
            {
                ROS_INFO ( "In Mission" );
 //               writeF<<"state_in_mission="<<state_in_mission<<endl;
 //               writeF<<"filtered_x="<<filtered_x<<endl;   // write in log
 //               writeF<<"filtered_y="<<filtered_y<<endl;   // write in log            
 //               writeF<<"Detection_fg="<<Detection_fg<<endl;// write in log              
 //               writeF << g_pos_x  << g_pos_y << g_pos_z << endl;
                state_msg.data = state_in_mission;
                state_in_mission_pub.publish(state_msg);
                ROS_INFO("state_in_mission= %d",state_in_mission);
                switch (state_in_mission) 
                {
                    case -1://take off
                    {
                        if (drone->flight_status==1)//standby
                        {  
                            start_yaw = drone->yaw_from_drone;//Record the yaw of taking off	
                            drone->takeoff();
                            writeF<<"begin take off start yaw = "<<start_yaw<<endl;
                        }
                        else if(drone->flight_status==3)//in air
                        {
                            if ( ob_distance[0]<serach_flight_height_1-0.1) //adjust height
                            {
                                flying_height_control_tracking += 0.003;
                            }
                            else if ( (ob_distance[0]>serach_flight_height_1+0.1)&&ob_distance[0]<5)
                            { 
                                flying_height_control_tracking -= 0.003;
                            }
                            drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                            writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log

                            if(abs(ob_distance[0]-serach_flight_height_1) < 0.2 &&(ob_distance[0]<4))	//forward about 2m
                            {
                            for(int i = 0; i < count_to_forward; i ++)
                                {
                                if(i < count_to_forward)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                usleep(10000);
                                }
                                state_in_mission = 0;
				    
                            }	
                        }
                        else
                        {
                        
                        }
                        break;	
                    }

                    case 0: //for parking pad 1
                    {		       
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);
			    
                            if(flip_once)
                            {
                                state_in_mission=1;
                                flip_once=false;
                            }
                        }
                        else //keep searching
                        {	      
                            switch (searching_state) 
                            {
                                case 0://1.first left
                                {
                                     if(g_pos_y-bound_len > 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 1;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,0.3,0,0);
                                     }
                                     break;
                                }
                                case 1://2.then right
                                {
                                     if(g_pos_y+bound_len < 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 2;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,-0.3,0,0);
                                     }
                                     break;
                                }
                                case 2://3.then keep height
                                {
                                    drone->attitude_control(0x4B,0,0,0,0);
                                     break;
                                }
                                default:
                                    break;
                            }
                        }
                        break;
                    }

                    case 1://take off and forward,need to change height
                    {
                        drone->gimbal_angle_control(0,0,0,20);   //look farward.
                        if ( (ob_distance[0]<serach_flight_height_1-0.1))
                        {
                            flying_height_control_tracking += 0.003;
                        }
                        else if ( (ob_distance[0]>serach_flight_height_1+0.1)&&ob_distance[0]<5)
                        { 
                            flying_height_control_tracking -= 0.003;
                        }
                        drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                        if((ob_distance[0]>serach_flight_height_1 - 1.5)&&ob_distance[0]<4) //change 1.5m
                        {
                            for(int i = 0; i < count_to_forward; i ++)
                            {
                                if(i < count_to_forward)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                usleep(10000); 
                            }
                            state_in_mission=2;
                        }
                        break;	
                    }

                    case 2:  //for crossing circle 3
                    {
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.5,2.3,Detection_fg,drone,time_count);  //2018-07-31 19:50  1.2  1.9
                            if(flip_once)
                            {
                                state_in_mission=3; 
                                flip_once=false;
                            }
                        }
                        else //keep searching
                        {		   
                            switch (searching_state) 
                            {
                                case 0://1.first left
                                {
                                     if(g_pos_y-bound_len > 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 1;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,0.3,0,0);
                                     }
                                     break;
                                }
                                case 1://2.then right
                                {
                                     if(g_pos_y+bound_len < 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 2;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,-0.3,0,0);
                                     }
                                     break;
                                }
                                case 2://3.then keep height
                                {
                                    drone->attitude_control(0x4B,0,0,0,0);
                                     break;
                                }
                                default:
                                    break;
                            }
                        }
                        break;
                    }
           	
                    case 3://take off and forward,need to change forward distance
                    {
                        drone->gimbal_angle_control(0,0,0,20);   //look farward.
                        if ( (ob_distance[0]<serach_flight_height_1-0.1))
                        {
                            flying_height_control_tracking += 0.003;
                        }
                        else if ( (ob_distance[0]>serach_flight_height_1+0.1)&&ob_distance[0]<5)
                        { 
                            flying_height_control_tracking -= 0.003;
                        }
                        drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                        if((ob_distance[0]>serach_flight_height_1 - 1.5)&&ob_distance[0]<4)
                        {
                            for(int i = 0; i < count_to_forward; i ++)
                            {
                                if(i < count_to_forward)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                usleep(10000);
                            }
                            state_in_mission=4;
                        }
                        break;	
                    }
                    
                    case 4: //for parking pad 2
                    {		       
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);
			    
                            if(flip_once)
                            {
                                state_in_mission=5;
                                flip_once=false;
                            }
                        }
                        else //keep searching
                        {	      
                            switch (searching_state) 
                            {
                                case 0://1.first left
                                {
                                     if(g_pos_y-bound_len > 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 1;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,0.3,0,0);
                                     }
                                     break;
                                }
                                case 1://2.then right
                                {
                                     if(g_pos_y+bound_len < 0)
                                     {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 2;
                                     }
                                     else
                                     {
                                        drone->attitude_control(0x4B,0,-0.3,0,0);
                                     }
                                     break;
                                }
                                case 2://3.then keep height
                                {
                                    drone->attitude_control(0x4B,0,0,0,0);
                                     break;
                                }
                                default:
                                    break;
                            }
                        }
                        break;
                    }

                    case 5: //land
                        drone->landing();
                        break;

                    default:
                        break;
                }
            } 
            else
            {
                ROS_INFO("MODE ERROR: Please button up first!");
            }
        }
    }
    
    return 0;
}

float sum ( float a[],size_t len )
{
    float sum = 0;
    for ( int i = 0; i<len; i++ )
    {
        sum += a[i];
    }
    return sum;
}

float find_min ( float a[], size_t len )
{
    float min = a[0];

    for ( int  j = 1; j<len; j++ )
    {
        if ( a[j]<min )
            min = a[j];

    }

    return min;
}

float find_max ( float a[], size_t len )
{
    float max = a[0];

    for ( int  j = 1; j<len; j++ )
    {
        if ( a[j]>max )
            max = a[j];
    }
    return max;
}


bool parking(float &cmd_fligh_x,float &cmd_flight_y,float height, uint8_t detection_flag,DJIDrone* drone)
{
    drone->gimbal_angle_control ( 0,-900,0,15 );
    writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log
        
    if(cmd_fligh_x>0.2) cmd_fligh_x=0.2;//init 0.8,limit bias for smoothing
    else if(cmd_fligh_x<-0.2) cmd_fligh_x=-0.2;
    if(cmd_flight_y>0.2) cmd_flight_y=0.2;
    else if(cmd_flight_y<-0.2) cmd_flight_y=-0.2;
  
    if(land_mode==0) //again for the same landing height
    {
        if ( ob_distance[0]<height-0.1)
        {
        flying_height_control_tracking += 0.003;
        }
        else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
        { 
        flying_height_control_tracking -= 0.003;
        }
        drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);
        
        if(ob_distance[0]>height-0.5)
        land_mode=1;
    }

    if(land_mode==1&&drone->gimbal.pitch<-85)   //closing to landing part and landing
    {
        if(detection_flag)  //parking pad detected
        {     
            if(abs(cmd_fligh_x)>0.12||abs(cmd_flight_y)>0.12)  //closing and keep height
            {
                if ( ob_distance[0]<height-0.1)
                {
                    flying_height_control_tracking += 0.003;
                }
                else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
                { 
                    flying_height_control_tracking -= 0.003;
                }
                drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);
            }
            else if(abs(cmd_fligh_x)<0.12&&abs(cmd_flight_y)<0.12) //samll adjust
            {
//                 samll_adjust_count ++;
//                 if(samll_adjust_count < 0)
//                 {
//                     drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0); 
//                 }
//                 else
//                 {
//                     samll_adjust_count = 0;
//                     //drone->landing();
//                     //sleep(1);
//                     for(int i = 0; i < count_to_landing; i ++)
//                     {
//                         if(i < count_to_landing)
//                             drone->attitude_control(0x4B, 0, 0, -0.5, 0);//NOT 0X42
//                         else
//                             drone->attitude_control(0x4B, 0, 0, 0, 0);
//                         usleep(10000);
//                     }
//                     land_mode=2;
//                 }
	      
		  if ( ob_distance[0]<1.8)
		  {
		    flying_height_control_tracking += 0.008;   //init 0.008
		  }
		  else if ( (ob_distance[0]>2.0)&&ob_distance[0]<6) 
		  { 
		    flying_height_control_tracking -= 0.008;    //init 0.008
		  }
		  drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);   //TODO: height adjusting

		  if(ob_distance[0]<2.0)
		  {
		      for(int i = 0; i < count_to_landing; i ++)
		      {
			  if(i < count_to_landing)
			      drone->attitude_control(0x4B, 0, 0, -0.5, 0);//NOT 0X42
			  else
			      drone->attitude_control(0x4B, 0, 0, 0, 0);
			  usleep(10000);
		      }
		      writeF << "land_mode = 2" << endl;
		      	land_mode=0;
			return true;
		    }
            }
        }
        else    //parkinng pad undetected
        {

        }
    } 
  return false;
}

bool cbarrier(float &cmd_fligh_x,float &cmd_flight_y, float &cmd_yaw,float height, float set_height, uint8_t detection_flag,DJIDrone* drone,int &delayCount)
{
  if(cbring_mode==0)  //turn the gimbal direction
  {
    drone->gimbal_angle_control(0,0,0,20);   //look farward.
    if ( ob_distance[0]<height-0.1)
    {
      flying_height_control_tracking += 0.005;
    }
    else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.005;
    }
    drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
    if(drone->gimbal.pitch>-5&&abs(ob_distance[0]-height)<0.5)
      cbring_mode=1; 
    writeF<<"decending for tag detection, pitch="<<drone->gimbal.pitch<<",height="<<ob_distance[0]<<endl;
  }
  else if(cbring_mode==1)  //closing to the hanging pad. first x,y then z
  {
    writeF<<"mode: "<<cbring_mode<<endl;
    //cmd_fligh_x-0.5 is the safe distance of drone to the pad, adjusting
    if ( ob_distance[0]<height-0.1)
    {
      flying_height_control_tracking += 0.002;
    }   
    else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.002;
    }
    if(detection_flag)
    {
      if(cmd_flight_y>0.2)  cmd_flight_y = 0.2;    //initial 0.25
      if(cmd_flight_y<-0.2) cmd_flight_y =-0.2;    //initial 0.25
      if(abs(cmd_yaw)<5&&cmd_flight_y<0.2)
	drone->attitude_control ( 0x5B,0.2,cmd_flight_y,flying_height_control_tracking,cmd_yaw);
      else
	drone->attitude_control ( 0x5B,0.0,cmd_flight_y,flying_height_control_tracking,cmd_yaw);
      writeF<<"tag detected, cmd_yaw="<<cmd_yaw<<endl;
    }
    else   
    {
      cmd_fligh_x=0.0; cmd_flight_y=0.0;
      drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
      writeF<<"tag NOT detected, cmd_yaw="<<cmd_yaw<<endl;   //yaw is not zero for no tag, need to be fixed.
    }
    if(detection_flag&&abs(cmd_fligh_x-1.4)<0.2&&abs(cmd_flight_y)<0.2&&abs(cmd_yaw)<5)// maybe should change
      cbring_mode=2;
  }
  else if(cbring_mode==2)   //go up
  {
    writeF<<"mode: "<<cbring_mode<<endl;
    if ( ob_distance[0]<set_height-0.1)
    {
      flying_height_control_tracking += 0.003;
    }
    else if ( (ob_distance[0]>set_height+0.1)&&ob_distance[0]<10)
    {
      flying_height_control_tracking -= 0.003;
    }
    drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
    
    if(abs(set_height-ob_distance[0])<0.2)
      cbring_mode=3;
  }
  else if(cbring_mode==3)  //cross
  { 
    writeF<<"mode: "<<cbring_mode<<", dtime="<<delayCount<<endl;
    if ( ob_distance[0]<set_height-0.1)
    {
      flying_height_control_tracking += 0.001;
    }
    else if ( (ob_distance[0]>set_height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.001;
    }
    drone->attitude_control ( 0x4B,0.4,0,0,0);  //go farward
    delayCount++;
    //sleep(10);
    //if(detection_flag==0) 
    //   drone->attitude_control ( 0x9B,1,0,flying_height_control_tracking,0);  //go farward
    //else
    if(delayCount>delay_count)
    {
      cbring_mode=0;
      delayCount=0;
      return true; 
    }
  }
  
  return false;
}