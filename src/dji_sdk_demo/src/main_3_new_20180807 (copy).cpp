#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include<std_msgs/Int8.h>
#include<std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include<math.h>
#include<fstream>

#define GIMBAL_USED
#define filter_N 4   
#define EPS 0.0000000001
#define USE_OBDIST  
#define PI 3.1415926536 
#define VEL_MODE

using namespace DJI::onboardSDK;
using namespace actionlib;

ofstream writeF ( "/home/ubuntu/GaofenChallenge/log.txt");

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
void tag_detection_resultCallback (const std_msgs::Bool & num_of_detection);
void obstacle_distance_callback(const sensor_msgs::LaserScan & g_oa);
float sum(float a[],size_t len);
float find_min(float a[], size_t len);
float find_max(float a[], size_t len);

static bool Detection_fg = false;
int delay_count = 300;
static uint8_t land_mode = 0;   
static int cbring_mode = 0; 
static float last_flight_x = 0.0;
static float last_flight_y = 0.0;  
static float last_flight_yaw = 0.0;//not used yet
static float ob_distance[5]= {10.0};

bool take_off_flag = 0;   //2018-08-05
float set_height = 2.5;   //2018-08-05

float flying_height_control_tracking=0.0;
float searching_height = 1.0;

// addition
int samll_adjust_count = 0;//for a small scall adjust xy
int count_to_landing = 220;
int count_forward=0;

/*obstacle_distance from guidance*/
/*0-down, 1-forward,2-right,3-backward,4-left*/

void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
    for ( int i=0; i<5; i++ )
        ob_distance[i]=g_oa.ranges[i];
    //ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
}

void tag_detection_resultCallback(const std_msgs::Bool & num_of_detection )
{
    Detection_fg = num_of_detection.data;
    //ROS_INFO ( "%d tag(s) are detected",numOfDetections );
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

int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "sdk_client" );
  ROS_INFO ( "sdk_service_client_test" );
  
  ros::NodeHandle node_priv ( "~" );
  //Some params
  int max_takeoff_waitcount = 700;
  double tracking_flight_height = 2.0;
  double descending_height_delta = 0.005;
  double initial_descending_height_at_search = 10.0;
  double initial_searching_height=1.5;
  node_priv.param<int> ( "delayCount",delay_count,100 );
  node_priv.param<double> ( "initTrackingHeight",tracking_flight_height,2.0 );
  node_priv.param<double> ( "descendSpeed",descending_height_delta,0.005 );
  node_priv.param<double> ( "initDescedingHeightSearch",initial_descending_height_at_search,10.0 );
  node_priv.param<int> ( "maxTakeoffWaitCount",max_takeoff_waitcount,700 );
  node_priv.param<double>("initial_searching_height",initial_searching_height,1.7);
  
  ros::NodeHandle nh; 
  DJIDrone* drone = new DJIDrone ( nh );   //
  
  writeF<<ros::Time::now() <<endl;//virtual RC test data
  //USED_APRILTAG_TYPE
  ros::Publisher  mission_type_pub = nh.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );  
  ros::Publisher start_searching_pub = nh.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );
  ros::Publisher state_in_mission_pub = nh.advertise<std_msgs::Int8>("/dji_sdk_demo/state_in_mission",10);
  ros::Subscriber tag_detections_sub = nh.subscribe ( "tag_detection/detections",100,tag_detection_resultCallback );
  ros::Subscriber obstacle_distance_sub = nh.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );
  
  flying_height_control_tracking = tracking_flight_height;
  
  float filtered_x=0.0,filtered_y=0.0, filtered_yaw=0.0;
  float yaw=0;
  //For filtering;
  float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};
  float initial_DeltaX, initial_DeltaY,DeltaX,DeltaY,initial_speedingup_height;
  float last_x_tag, last_y_tag;
  
  int time_count=0;
  int count=0;
  
  bool main_operate_code=false;
  
  int state_in_mission = 0;
  std_msgs::Int8 state_msg;
  state_msg.data = state_in_mission;
  
  float start_yaw = 0.0;
  
  std_msgs::Bool start_searching;
  start_searching.data= false;
  
  bool flip_once = false;
  
  std_msgs::Bool mission_type;
  mission_type.data=false; //false for round 3, true for round 4

  int mode = 0;
  


  ros::Rate loop_rate ( 50 );
  //Head down at the beginning.row,picth,yaw,duration
  drone->gimbal_angle_control ( 0,-900,0,20 ); 
  
  while ( ros::ok() )
  {
    ros::spinOnce();
    
    // if start_searching=1, follow line
    start_searching_pub.publish ( start_searching );
    mission_type_pub.publish ( mission_type );
    
    for ( int i = 0; i< filter_N-1; i++ )
    {
      filter_seq_x[i] = filter_seq_x[i+1];
      filter_seq_y[i] = filter_seq_y[i+1];
    }
    filter_seq_x[filter_N-1] = drone->flight_x;
    filter_seq_y[filter_N-1] = drone->flight_y;
    last_flight_yaw = filtered_yaw;

    //filtered_x =  drone->flight_x*0.9;//
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
      main_operate_code=false;
      cbring_mode=0;
      land_mode=0;
    }
    else
    {
      if(mode)
      {
	ROS_INFO ( "In Mission" );
	writeF<<"state_in_mission="<<state_in_mission<<endl;
	//
	writeF<<"filtered_x="<<filtered_x<<endl;   // write in log
	writeF<<"filtered_y="<<filtered_y<<endl;   // write in log

	//
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
  //	      flying_height_control_tracking = tracking_flight_height;
		drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0 );
		if(drone->local_position.z>2.6)	 
		    state_in_mission=0;		
	      }
	      else
	      {
		
	      }
	    break;	
	  }
/********************************************************************************************************************************/	  
	  case 0: //for parking pad 2
	  {
	      if(main_operate_code==false)
	      {
		  count++;
		  drone->attitude_control(0x4B,0.7,0,0,0); //first go forward
		  if(count>200) //for middle searching
		  { 
		    count=0;
		    main_operate_code=true;
		    drone->attitude_control(0x4B,0,0,0,0);
		  }
	      }
	      else
	      {	      
		  flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		  if(flip_once)
		  {
		    state_in_mission=1;
		    flip_once=false;
		    main_operate_code=false;
		  }
	      }
	      break;
	  }
/*******************************************************************************************************************************/	
	  case 1:  //for crossing circle 3
	  {
	    flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.5,2.3,Detection_fg,drone,time_count);  //2018-07-31 19:50  1.2  1.9
	    if(flip_once)
	    {
	      state_in_mission=2; 
	      flip_once=false;
	    }
	  break;
	  }
/*******************************************************************************************************************************/		  
	  case 2:  //for parking pad 4
	  {
	    writeF<<"drone yaw"<<drone->yaw_from_drone<<" start_yaw"<<start_yaw<<endl;
	    if(abs(drone->yaw_from_drone-start_yaw-1.57)*57.3>5)
	      drone->attitude_control(0x4B,0,0,0,10); //back
	    else
	    {
		writeF<<"after turned, go ahead"<<endl;
		if(main_operate_code==false)
		{
		    count++;
		    drone->attitude_control(0x4B,0.4,0.2,0,0); //back
		    if(count>280) //for middle searching
		    { 
		      count=0;
		      main_operate_code=true;
		      drone->attitude_control(0x4B,0,0,0,0);
		    }
		}
		else
		{		
		    flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
				    
		    if(flip_once)
		    {
		      main_operate_code=false;
		      state_in_mission=3;
		      flip_once=false;
		    }
		}
	    }
	  break;
	  }
/***************************************************************/	
	  case 3: //land
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
                    
                    land_mode=2;
                }
            }
        }
        else    //parkinng pad undetected
        {

        }
    } 
    else if(land_mode==2)   //automatic takeoff
    {
        //drone->takeoff();
        //sleep(1);
        if(take_off_flag == 0)
	{
	    drone->attitude_control(0x5B,0,0,3.8,0);
	}				
	if ( (ob_distance[0] > set_height)&& (ob_distance[0] < 4))
	{
	    drone->attitude_control(0x4B,0,0,0,0);
	    take_off_flag  = 1;
	    land_mode = 3;
	}
    }
    else if(land_mode==3)   //automatic forward
    {
        take_off_flag = 0;
	
        count_forward++;
        if(count_forward<100)
        {
            drone->attitude_control(0x4B,0.7,0,0,0);
        }
        else
        {
            count_forward=0;
            land_mode=0;
            return true;
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
	if(cmd_flight_y>0.16)  cmd_flight_y = 0.16;    //initial 0.25
	if(cmd_flight_y<-0.16) cmd_flight_y =-0.16;    //initial 0.25
	if(abs(cmd_yaw)<5&&cmd_flight_y<0.1)           //initial 0.1
	  drone->attitude_control ( 0x5B,0.25,cmd_flight_y,flying_height_control_tracking,cmd_yaw);   //initial 0.2
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
      
      if(detection_flag&&abs(cmd_fligh_x-1.4)<0.2&&abs(cmd_flight_y)<0.2&&abs(cmd_yaw)<5)
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
