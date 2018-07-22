#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//#include<geometry_msgs/PoseWithCovarianceStamped.h>

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

ofstream writeF ( "/root/log.txt");
using namespace DJI::onboardSDK;
using namespace actionlib;
static uint8_t numOfDetections = 0;
static float last_flight_x = 0.0;
static float last_flight_y = 0.0;
static float last_flight_yaw = 0.0;//not used yet
static bool using_smallTags = false;  
static float flight_yaw_relative = 0.0;//not used yet
static float ob_distance[5]= {10.0};


float cmd_vel_x = 0.0, cmd_vel_y = 0.0 ,cmd_vel_z = 0.0;
float searching_height = 1.0;

void apriltag_detection_resultCallback ( const std_msgs::Int8 & num_of_detection )
{
	numOfDetections = num_of_detection.data;
	//ROS_INFO ( "%d tag(s) are detected",numOfDetections );
}
void apriltag_using_smallTags_callback ( const std_msgs::Bool & using_smallTags_ )
{
	using_smallTags = using_smallTags_.data;
}
void base_controller_callback ( const geometry_msgs::TwistConstPtr& msg )
{
	cmd_vel_x = msg->linear.x;
	cmd_vel_y = msg->linear.y;
	cmd_vel_z = msg->angular.z;
}

/*obstacle_distance from guidance*/
/*0-down, 1-forward,2-right,3-backward,4-left*/

void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
	for ( int i=0; i<5; i++ )
		ob_distance[i]=g_oa.ranges[i];
	
	//ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
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


int get_direction_mode()
{
	float safe_distance=1.0;
	int direction_mode;
	if (ob_distance[1]<safe_distance)//forward
	{
		if (ob_distance[2]>safe_distance)//right
			direction_mode=2;
		else
		{
			if (ob_distance[4]>safe_distance)//left
				direction_mode=3;
			else
			{
				if (ob_distance[3]>safe_distance)//back
					direction_mode=4;
				else
					direction_mode=0;
			}
		}
	}
	else
		direction_mode=1;
	return direction_mode;
}


int main ( int argc, char **argv )
{
	bool main_operate_code = false;
	int temp32;
	bool valid_flag = false;
	bool err_flag = false;
	//for test
	double v_state_x=0.0, v_state_y=0.0, vxx=0.0, vyy=0.0;
	
	//Some params
	int delay_count = 100;
	int max_takeoff_waitcount = 700;
	double tracking_flight_height = 2.0;
	double descending_height_delta = 0.005;
	double initial_descending_height_at_search = 10.0;
	double initial_searching_height=1.5;
	
	ros::init ( argc, argv, "sdk_client" );
	ROS_INFO ( "sdk_service_client_test" );
	
	
	ros::NodeHandle node_priv ( "~" );
	node_priv.param<int> ( "delayCount",delay_count,100 );
	node_priv.param<double> ( "initTrackingHeight",tracking_flight_height,2.0 );
	node_priv.param<double> ( "descendSpeed",descending_height_delta,0.005 );
	node_priv.param<double> ( "initDescedingHeightSearch",initial_descending_height_at_search,10.0 );
	node_priv.param<int> ( "maxTakeoffWaitCount",max_takeoff_waitcount,700 );
	node_priv.param<double>("initial_searching_height",initial_searching_height,1.7);
	ros::NodeHandle nh; 
	DJIDrone* drone = new DJIDrone ( nh );
	
	writeF<<ros::Time::now() <<endl;
	//virtual RC test data
	
	ros::Publisher  mission_type_pub = nh.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );
	
	
	ros::Subscriber apriltag_num_of_detections_sub = nh.subscribe ( "apriltag_detection/numofdetections",100,apriltag_detection_resultCallback );
	ros::Subscriber apriltag_using_smallTags = nh.subscribe ( "apriltag_detection/usingSmallTags",10,apriltag_using_smallTags_callback );
	ros::Subscriber obstacle_distance_sub = nh.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );
	
	//SEARCHING FOR SURVIVORS sub
	ros::Subscriber sub = nh.subscribe ( "cmd_vel", 10, base_controller_callback );
	
	//USED_APRILTAG_TYPE
	ros::Publisher start_searching_pub = nh.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );
	
	
	float flying_height_control_tracking = tracking_flight_height;
	float safe_distance=1.0;
	
	int count = 0;
	int count_tracking = 0;
	int count_losing_tracking =0;
	ros::Rate loop_rate ( 50 );
	static bool flag = false;
	// enum used_apriltag_type{tag25H9=1, tag36H11, tag16H5};
	//uint8_t used_apriltag_type = 1;
	
	//float flight_x_filtered = 0.0;
	// std_msgs::Float64 filtered_x_msg,not_filtered_x_msg;
	float filtered_x=0.0,filtered_y=0.0, filtered_yaw=0.0;
	//filtered_x_msg.data = 0.0;
	float yaw=0;
	// not_filtered_x_msg.data = 0.0;
	
	//For filtering;
	float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};
	
	//For vel_MODE
	float vel_x_drone = 0.0, vel_y_drone = 0.0;
	
	float initial_DeltaX, initial_DeltaY,DeltaX,DeltaY,initial_speedingup_height;
	float last_x_tag, last_y_tag;
	//drone->gimbal_angle_control ( 0,-900,0,20 ); //Head down at the beginning.
	int start_state=0;
	int landing_state = 0;
	int state_in_mission = 0;
	int tik_count = 0;
	
	
	
	
	int takeoff_wait_count = 0;
	
	float start_yaw = 0.0;
	int pitch_rate = 50;
	int yaw_rate = 50;
	
	std_msgs::Bool start_searching;
	start_searching.data= true;
	bool flip_once = true;
	
	/******************************************IMPORTANT*************************************************/
	std_msgs::Bool mission_type;
	mission_type.data=true; //false for round 3, true for round 4
	drone->gimbal_angle_control( 0,0,0,15);
	
	int direction_mode=1;//1-forward  2-right  3-left  
	
	while ( ros::ok() )
	{
		ros::spinOnce();
		
		start_searching_pub.publish ( start_searching );
		mission_type_pub.publish ( mission_type );
		
		//TODO:IF obstacle is the horizontial level, how to use obstacle distance to avoid. Maybe it already used.
		switch ( drone->transparentdata.data.at ( 0 ) )
		{
			
			/*****************************************************'Q' : TAKE OFF**********************************************/
			/********************************************************************************************************************/
			case 'q':
			{
				//ROS_INFO ( "take off" );
				writeF<<"take off"<<drone->local_position.z<<endl;
				//cannot use flying_height_control_tracking for second time flight.
				start_state=0;
				count=0;
				tik_count==0;
				//drone->gimbal_angle_control( 0,0,0,20);
				flying_height_control_tracking = tracking_flight_height;
				start_yaw = drone->yaw_from_drone;//Record the yaw of taking off
				main_operate_code=false;
				break;
			}
			/************************************************'d' START LINE FOLLOW MISSION**********************************************************************/
			/*****************************************************************************************************************/
			case 'd':  
			{
			  start_searching.data=true;
			  if(start_state==0)
			  {
			    drone->takeoff();
			    if(drone->flight_status==3)
			      drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0 );
			    if(drone->local_position.z>tracking_flight_height-0.3)
			      start_state=1;
			  }
			  else
			  {
			    if(drone->rc_channels.gear==-10000&&main_operate_code==false)
			    {
			      count++;
			      drone->attitude_control(0x4B,0,-0.6,0,0);
			      if(count>380) //for middle searching
			      { 
				count=0;
				main_operate_code=true;
				drone->attitude_control(0x4B,0,0,0,0);
			      }
			    }
			    else if(drone->rc_channels.gear==-4545&&main_operate_code==false)
			    {
			      count++;
			      drone->attitude_control(0x4B,0,0.0,0,0);
			      if(count>50) //for middle searching
			      { 
				count=0;
				main_operate_code=true;
				drone->attitude_control(0x4B,0,0,0,0);
			      }
			    }
			    else
			    {
			      
			      if (ob_distance[0]<tracking_flight_height-0.1)// ||flying_height_control_tracking<1.8 ) drone->local_position.z<tracking_flight_height-0.1||
			      { 
				flying_height_control_tracking += 0.001;
			      }
			      if ( (ob_distance[0]>tracking_flight_height+0.1)&ob_distance[0]<10)// &&flying_height_control_tracking>2.2 ) //ob_distance[0]>1.8 )drone->local_position.z>tracking_flight_height+0.1||
			      {  
				flying_height_control_tracking -= 0.001;
			      }
			      
			      writeF<<ob_distance[1]<<"  "<<ob_distance[2]<<"  "<<ob_distance[4]<<" mode:"<<direction_mode<<endl;
			      if (direction_mode==1)//forward
			      {
				filtered_x=0.2;
				filtered_y=0;
				if (ob_distance[1]<safe_distance)
				{
				  if (ob_distance[2]>ob_distance[4])//right>left
				    direction_mode=2;
				  else 
				    direction_mode=3;
				}
			      }
			      
			      if (direction_mode==2)//right
			      {
				filtered_x=0;
				filtered_y=0.4;
				if (ob_distance[2]<safe_distance)
				  direction_mode=3;
				if (ob_distance[1]>safe_distance+0.5)
				  direction_mode=1;
			      }
			      
			      if (direction_mode==3)//left
			      {
				filtered_x=0;
				filtered_y=-0.4;
				if (ob_distance[4]<safe_distance)
				  direction_mode=2;
				if (ob_distance[1]>safe_distance+0.5)
				  direction_mode=1;
			      }
			      
			      if(tik_count<delay_count)
			      {
				drone->attitude_control ( 0x5B,filtered_x,filtered_y,flying_height_control_tracking,0 );    
				tik_count++;
			      } 
			      else
			      {
				tik_count=0;
				drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0 );    
				drone->gimbal_angle_control( 0,0,0,25,0);
				sleep(4);
				drone->gimbal_angle_control( 0,0,-900,25,0);
				sleep(4);
				drone->gimbal_angle_control( 0,0,1800,25,0);
				sleep(4);
				drone->gimbal_angle_control( 0,0,900,25,0);
				sleep(4);
				drone->gimbal_angle_control( 0,0,-1800,25,0);
				sleep(4);
			      }
			    }
			  }
			  
			  break;
			} 
			
			/*****************************************************'a' for abort mission***************************************************************************/
			/********************************************************************************************************************************************/
			case 'a':
				start_searching.data=false;
				ROS_INFO ( "abort mision" );
				writeF<<"abort mision"<<drone->local_position.z<<endl;
				// for ( int i = 0; i<200; i++ )
				drone->attitude_control ( 0x9B,0,0,drone->local_position.z,0 );
				// while ( !drone->release_sdk_permission_control() )
				
				drone->release_sdk_permission_control();
				//usleep ( 10000 );
				break;
				
			default:
				//  ROS_INFO ( "Waithing for command from mobile!\n" );
				break;
		}
		ROS_INFO ( "Command code is %c",drone->transparentdata.data.at ( 0 ) );
		loop_rate.sleep();
		
	}
	writeF.close();
	return 0;
}
