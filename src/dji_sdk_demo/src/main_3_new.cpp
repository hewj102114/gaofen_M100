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

static float ul_distance[5]= {10.0};
static int ul_reliability[5]= {1};
/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{
//    printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    for (int i = 0; i < 5; i++)
    {
	ul_distance[i]=g_ul.ranges[i];
//        printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
    }
}

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

static float guidance_pos_x = 0.0;
static float guidance_pos_y = 0.0;
static float g_pos_z = 0.0;
float last_g_pos_x = 0.0;
/* motion */
/* postion relative to original points*/
float start_yaw = 0.0;

void position_callback(const geometry_msgs::Vector3Stamped& g_pos)
{
     guidance_pos_x=cos(start_yaw)*g_pos.vector.x-sin(start_yaw)*g_pos.vector.y;
     guidance_pos_y=sin(start_yaw)*g_pos.vector.x+cos(start_yaw)*g_pos.vector.y;
     g_pos_z=g_pos.vector.z;
//     g_pos_x=g_pos.vector.x;
//     g_pos_y=g_pos.vector.y;
//     g_pos_z=g_pos.vector.z;
//    printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);

//    printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
}

int go_down = 0;
int mode = 0;
//bool take_off_flag = 0;
//int samll_adjust_count = 0;//for a small scall adjust flight xy

float flying_height_control_tracking = 3.0;
int count_to_landing = 230;

//just now 2018-08-20 
int searching_state = 0;
int tags_searching_state = 0;
int cross_state = 0;
float g_pos_x=0.0,g_pos_y=0.0;

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
	      uint8_t detection_flag,
	      DJIDrone* drone);
void searching_right(float &pos_y,float &bound_length, int state,DJIDrone* drone);
void searching_left(float &pos_y,float &bound_length, int state,DJIDrone* drone);
bool mission_function(bool park_or_circle,float target_flight_height,float cross_forward_distance,float serach_flight_height_next,float last_g_posx,DJIDrone* drone);                   

int main(int argc, char **argv) 
{
    //global
    int bound_len = 500;//adjust bound lenght avoid getting out
    float searching_velocity = 0.3;//init 0.3

    bool park_direction_1 = false; //right is true,left is flase
    bool park_direction_2 = false;
    bool park_direction_3 = false;
    bool cross_direction_4 = false;
    bool park_direction_5 = false;
    bool cross_direction_6 = false;
    bool cross_direction_7 = false;
    bool park_direction_8 = false;
    bool park_direction_9 = false;
    
    bool park_or_circle_1 = false; //park is false,circle is true
    bool park_or_circle_2 = false;
    bool park_or_circle_3 = false;
    bool park_or_circle_4 = false;
    bool park_or_circle_5 = false;
    bool park_or_circle_6 = false;
    bool park_or_circle_7 = false;
    bool park_or_circle_8 = false;
    bool park_or_circle_9 = false;

    float serach_flight_height_1 = 3.0;
    float serach_flight_height_2 = 3.0;
    float serach_flight_height_3 = 3.0;
    float serach_flight_height_4 = 3.0;
    float serach_flight_height_5 = 3.0;
    float serach_flight_height_6 = 3.0;
    float serach_flight_height_7 = 3.0;
    float serach_flight_height_8 = 3.0;
    float serach_flight_height_9 = 3.0;

    float target_flight_height_1 = 3.0;
    float target_flight_height_2 = 3.0;
    float target_flight_height_3 = 3.0;
    float target_flight_height_4 = 3.0;
    float target_flight_height_5 = 3.0;
    float target_flight_height_6 = 3.0;
    float target_flight_height_7 = 3.0;
    float target_flight_height_8 = 3.0;
    float target_flight_height_9 = 3.0;

    int count_to_forward01 = 75;
    float count_to_forward12 = 75;
    float cross_forward_distance23 = 2.0;
    float cross_forward_distance34 = 0;
    float cross_forward_distance45 = 3.2; //distance1=3.5
    float cross_forward_distance56 = 0;
    float cross_forward_distance67 = 2.2; //distance2=2.5
    float cross_forward_distance78 = 3.2; //distance1=3.5
    float cross_forward_distance89 = 2.0;
    float cross_forward_distance910 = 5.0;
    
    int set_count_to_landing = 230;      // if landoff height is 3.0, count_to_landing = 280;
					 //if landoff = 270 seems a little big
    
    //for tags
    float serach_flight_height_tags = 1.3;// for tags
    int count_forward_tags = 200;//count_between_tags
 //   int count_to_tags = 200;
    int tags_searching_count=0;// go forward count
    
    ros::init ( argc, argv, "main_base" );
    ROS_INFO ( "gaofen_main_base_test" );

    ros::NodeHandle node_priv ( "~" );
    node_priv.param<int> ( "bound_len",bound_len,500 );
    node_priv.param<float> ( "searching_velocity",searching_velocity,0.3);
    
    node_priv.param<bool> ( "park_direction_1",park_direction_1,false );
    node_priv.param<bool> ( "park_direction_2",park_direction_2,false );
    node_priv.param<bool> ( "park_direction_3",park_direction_3,false );
    node_priv.param<bool> ( "cross_direction_4",cross_direction_4,false );
    node_priv.param<bool> ( "park_direction_5",park_direction_5,false );
    node_priv.param<bool> ( "cross_direction_6",cross_direction_6,false );
    node_priv.param<bool> ( "cross_direction_7",cross_direction_7,false );
    node_priv.param<bool> ( "park_direction_8",park_direction_8,false );
    node_priv.param<bool> ( "park_direction_9",park_direction_9,false );

    node_priv.param<bool> ( "park_or_circle_1",park_or_circle_1,false );
    node_priv.param<bool> ( "park_or_circle_2",park_or_circle_2,false );
    node_priv.param<bool> ( "park_or_circle_3",park_or_circle_3,false );
    node_priv.param<bool> ( "park_or_circle_4",park_or_circle_4,false );
    node_priv.param<bool> ( "park_or_circle_5",park_or_circle_5,false );
    node_priv.param<bool> ( "park_or_circle_6",park_or_circle_6,false );
    node_priv.param<bool> ( "park_or_circle_7",park_or_circle_7,false );
    node_priv.param<bool> ( "park_or_circle_8",park_or_circle_8,false );
    node_priv.param<bool> ( "park_or_circle_9",park_or_circle_9,false );

    node_priv.param<float> ( "serach_flight_height_1",serach_flight_height_1,3.0 );
    node_priv.param<float> ( "serach_flight_height_2",serach_flight_height_2,3.0 );
    node_priv.param<float> ( "serach_flight_height_3",serach_flight_height_3,3.0 );
    node_priv.param<float> ( "serach_flight_height_4",serach_flight_height_4,3.0 );
    node_priv.param<float> ( "serach_flight_height_5",serach_flight_height_5,3.0 );
    node_priv.param<float> ( "serach_flight_height_6",serach_flight_height_6,3.0 );
    node_priv.param<float> ( "serach_flight_height_7",serach_flight_height_7,3.0 );
    node_priv.param<float> ( "serach_flight_height_8",serach_flight_height_8,3.0 );
    node_priv.param<float> ( "serach_flight_height_9",serach_flight_height_9,3.0 );

    node_priv.param<float> ( "target_flight_height_1",target_flight_height_1,3.0 );
    node_priv.param<float> ( "target_flight_height_2",target_flight_height_2,3.0 );
    node_priv.param<float> ( "target_flight_height_3",target_flight_height_3,3.0 );
    node_priv.param<float> ( "target_flight_height_4",target_flight_height_4,3.0 );
    node_priv.param<float> ( "target_flight_height_5",target_flight_height_5,3.0 );
    node_priv.param<float> ( "target_flight_height_6",target_flight_height_6,3.0 );
    node_priv.param<float> ( "target_flight_height_7",target_flight_height_7,3.0 );
    node_priv.param<float> ( "target_flight_height_8",target_flight_height_8,3.0 );
    node_priv.param<float> ( "target_flight_height_9",target_flight_height_9,3.0 );

    node_priv.param<int> ( "count_to_forward01",count_to_forward01,75);
    node_priv.param<float> ( "count_to_forward12",count_to_forward12,2.0);
    node_priv.param<float> ( "cross_forward_distance23",cross_forward_distance23,2.0);
    node_priv.param<float> ( "cross_forward_distance34",cross_forward_distance34,0);
    node_priv.param<float> ( "cross_forward_distance45",cross_forward_distance45,3.2);
    node_priv.param<float> ( "cross_forward_distance56",cross_forward_distance56,0);
    node_priv.param<float> ( "cross_forward_distance67",cross_forward_distance67,2.2);
    node_priv.param<float> ( "cross_forward_distance78",cross_forward_distance78,3.2);
    node_priv.param<float> ( "cross_forward_distance89",cross_forward_distance89,2.0);
    node_priv.param<float> ( "cross_forward_distance910",cross_forward_distance910,5.0);

    node_priv.param<int> ( "set_count_to_landing",set_count_to_landing,230);
    
    node_priv.param<float>("serach_flight_height_tags",serach_flight_height_tags,1.3);
    node_priv.param<int> ( "count_forward_tags",count_forward_tags,200);
    
    ros::NodeHandle n;

    DJIDrone* drone = new DJIDrone(n);
    //virtual RC test data
    writeF<<ros::Time::now() <<endl;
    
    writeF << "bound_len =" << bound_len << endl;
    writeF << "searching_velocity =" << searching_velocity << endl;

    writeF << "park_direction_1 =" << park_direction_1 << endl;
    writeF << "park_direction_2 =" << park_direction_2 << endl;
    writeF << "park_direction_3 =" << park_direction_3 << endl;
    writeF << "cross_direction_4 =" << cross_direction_4 << endl;
    writeF << "park_direction_5 =" << park_direction_5 << endl;
    writeF << "cross_direction_6 =" << cross_direction_6 << endl;
    writeF << "cross_direction_7 =" << cross_direction_7 << endl;
    writeF << "park_direction_8 =" << park_direction_8 << endl;
    writeF << "park_direction_9 =" << park_direction_9 << endl;

    writeF << "park_or_circle_1 =" << park_or_circle_1 << endl;
    writeF << "park_or_circle_2 =" << park_or_circle_2 << endl;
    writeF << "park_or_circle_3 =" << park_or_circle_3 << endl;
    writeF << "park_or_circle_4 =" << park_or_circle_4 << endl;
    writeF << "park_or_circle_5 =" << park_direction_5 << endl;
    writeF << "park_or_circle_6 =" << park_or_circle_6 << endl;
    writeF << "park_or_circle_7 =" << park_or_circle_7 << endl;
    writeF << "park_or_circle_8 =" << park_or_circle_8 << endl;
    writeF << "park_or_circle_9 =" << park_or_circle_9 << endl;

    writeF << "serach_flight_height_1 =" << serach_flight_height_1 << endl;
    writeF << "serach_flight_height_2 =" << serach_flight_height_2 << endl;
    writeF << "serach_flight_height_3 =" << serach_flight_height_3 << endl;
    writeF << "serach_flight_height_4 =" << serach_flight_height_4 << endl;
    writeF << "serach_flight_height_5 =" << serach_flight_height_5 << endl;
    writeF << "serach_flight_height_6 =" << serach_flight_height_6 << endl;
    writeF << "serach_flight_height_7 =" << serach_flight_height_7 << endl;
    writeF << "serach_flight_height_8 =" << serach_flight_height_8 << endl;
    writeF << "serach_flight_height_9 =" << serach_flight_height_9 << endl;

    writeF << "target_flight_height_1 =" << target_flight_height_1 << endl;
    writeF << "target_flight_height_2 =" << target_flight_height_2 << endl;
    writeF << "target_flight_height_3 =" << target_flight_height_3 << endl;
    writeF << "target_flight_height_4 =" << target_flight_height_4 << endl;
    writeF << "target_flight_height_5 =" << target_flight_height_5 << endl;
    writeF << "target_flight_height_6 =" << target_flight_height_6 << endl;
    writeF << "target_flight_height_7 =" << target_flight_height_7 << endl;
    writeF << "target_flight_height_8 =" << target_flight_height_8 << endl;
    writeF << "target_flight_height_9 =" << target_flight_height_9 << endl;

    writeF << "count_to_forward01 =" << count_to_forward01 << endl;
    writeF << "count_to_forward12 =" << count_to_forward12 << endl;
    writeF << "cross_forward_distance23 =" << cross_forward_distance23 << endl;
    writeF << "cross_forward_distance34 =" << cross_forward_distance34 << endl;
    writeF << "cross_forward_distance45 =" << cross_forward_distance45 << endl;
    writeF << "cross_forward_distance56 =" << cross_forward_distance56 << endl;
    writeF << "cross_forward_distance67 =" << cross_forward_distance67 << endl;
    writeF << "cross_forward_distance78 =" << cross_forward_distance78 << endl;
    writeF << "cross_forward_distance89 =" << cross_forward_distance89 << endl;
    writeF << "cross_forward_distance910 =" << cross_forward_distance910 << endl;

    writeF << "set_count_to_landing =" << set_count_to_landing << endl;

    writeF << "serach_flight_height_tags =" << serach_flight_height_tags << endl;
    writeF << "count_forward_tags =" << count_forward_tags << endl;

    ros::Publisher  mission_type_pub = n.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );  
    ros::Publisher start_searching_pub = n.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );
    ros::Publisher state_in_mission_pub = n.advertise<std_msgs::Int8>("/dji_sdk_demo/state_in_mission",10);
    ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );
    ros::Subscriber position_sub = n.subscribe("/guidance/position", 10, position_callback);
    ros::Subscriber tag_detections_sub = n.subscribe ( "tag_detection/detections",10,tag_detection_resultCallback );
    ros::Subscriber ultrasonic_sub = n.subscribe("/guidance/ultrasonic", 10, ultrasonic_callback);
    
    //for easy to adjust height
    flying_height_control_tracking = serach_flight_height_1;
    count_to_landing = set_count_to_landing;
    
    //For filtering;
    float filtered_x=0.0,filtered_y=0.0, filtered_yaw=0.0;
//    float g_pos_x=0.0,g_pos_y=0.0;
    float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};
    float filter_pos_x[filter_N]= {0},filter_pos_y[filter_N]= {0};

    int time_count=0;

    bool flip_once = false;//mission success flag
    bool mission_function_flag = false;//mission success flag
    
    //public datas
    int state_in_mission = 0;
    std_msgs::Int8 state_msg;
    state_msg.data = state_in_mission;//0~9 state

    std_msgs::Bool start_searching;//park/circle
    start_searching.data= false;//park

    std_msgs::Bool mission_type;//numbers/tags
    mission_type.data=false;//numbers
/*
    int searching_state = 0;
    //2018-08-07 evening
    int tags_searching_state = 0;
    //just now 2018-08-07 night
    int cross_state = 0;
*/
    ros::Rate loop_rate ( 50 );
    //Head down at the beginning.row,picth,yaw,duration
    
    drone->gimbal_angle_control ( 0,-900,0,20 ); 
//    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross

    while ( ros::ok() )
    {
	ros::spinOnce();
	//ROS_INFO("IN");
        // for numbers setting
	
        start_searching_pub.publish ( start_searching );
        mission_type_pub.publish ( mission_type );

        // get bias values
        for ( int i = 0; i< filter_N-1; i++ )
        {
            filter_seq_x[i] = filter_seq_x[i+1];
            filter_seq_y[i] = filter_seq_y[i+1];
	    filter_pos_x[i] = filter_pos_x[i+1];
            filter_pos_y[i] = filter_pos_y[i+1];
        }
        filter_seq_x[filter_N-1] = drone->flight_x;
        filter_seq_y[filter_N-1] = drone->flight_y;
        last_flight_yaw = filtered_yaw;
	
	filter_pos_x[filter_N-1] = guidance_pos_x;
        filter_pos_y[filter_N-1] = guidance_pos_y;
	
        filtered_x =  (( sum ( filter_seq_x,filter_N )-find_max ( filter_seq_x,filter_N )-find_min ( filter_seq_x,filter_N ) ) / ( filter_N-2 ))*0.8;
        filtered_y =  (( sum ( filter_seq_y,filter_N )-find_max ( filter_seq_y,filter_N )-find_min ( filter_seq_y,filter_N ) ) / ( filter_N-2 ))*0.8;
	
        g_pos_x = ( sum ( filter_pos_x,filter_N )-find_max ( filter_pos_x,filter_N )-find_min ( filter_pos_x,filter_N ) ) / ( filter_N-2 );
        g_pos_y = ( sum ( filter_pos_y,filter_N )-find_max ( filter_pos_y,filter_N )-find_min ( filter_pos_y,filter_N ) ) / ( filter_N-2 );
        
        filtered_yaw= drone->flight_yaw;
        if(abs(filtered_yaw-last_flight_yaw)>20.0)   
        filtered_yaw = last_flight_yaw;
        if(filtered_yaw>10) 
        filtered_yaw=10;
        if(filtered_yaw<-10) 
        filtered_yaw=-10;
//	ROS_INFO("IN");
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

 //   state_in_mission=6;//for cross
//	    state_in_mission=18;//for tags
  // 	    state_in_mission=3;//2018-08-20
  /// 	    state_in_mission=1;//from 1 to 2
            time_count=0;
            cbring_mode=0;
            land_mode=0;
        }
        else
        {
            if(mode)
            {
  //               ROS_INFO ( "In Mission" );
               writeF<<"state_in_mission="<<state_in_mission<<endl;
                //writeF<<"filtered_x="<<filtered_x<<endl;   // write in log
                //writeF<<"filtered_y="<<filtered_y<<endl;   // write in log            
                writeF<<"Detection_fg="<<Detection_fg<<endl;// write in log              
 //               writeF << g_pos_x  << g_pos_y << g_pos_z << endl;
                state_msg.data = state_in_mission;
                state_in_mission_pub.publish(state_msg);
//              ROS_INFO("state_in_mission= %d",state_in_mission);
                switch (state_in_mission) 
                {
                    case -1://take off, serach_flight_height_1, count_to_forward01 ,to park
                    {		       
                        if (drone->flight_status==1)//standby
                        {  
                            start_yaw = drone->yaw_from_drone;//Record the yaw of taking off	
                            drone->takeoff();
                            writeF<<"begin take off start yaw = "<<start_yaw<<endl;
			               // writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
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
                            drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

                           // writeF<<"ob_distance[0]="<<ob_distance[0]<<endl;   // write in log
                           // writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;

                            if(abs(ob_distance[0]-serach_flight_height_1) < 0.2 &&(ob_distance[0]<4))	//forward about 2m
	
                            {
                                for(int i = 0; i < count_to_forward01; i ++)
                                {
                                if(i < count_to_forward01)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                usleep(10000);
                                }
                                state_in_mission = 0;//cross circle  /////////////////////////////////////////////////////
                              
             //                   writeF<<"start yaw = "<<start_yaw<<endl;
			       // writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
	
                                //2018-08-19 night
                                start_searching.data = park_or_circle_1; //for park
                            }	
                        }
                        else
                        {
                   
                        }
                        break;	
                    }

                    case 0: //for parking pad 1, target_flight_height_1 = 0
                    {		       
                        if(Detection_fg) //if detecting!
                        {
 //                           searching_state = 0;

                            flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);
			    
                            if(flip_once)
                            {
				                        last_g_pos_x = g_pos_x;
                                state_in_mission=1;
                                flip_once=false;
				
			                      	//2018-08-19 night
                                start_searching.data = park_or_circle_2; //for park
                            }
                        }
                        else //keep searching
                        {
                            if(park_direction_1) //right
                            {
                                drone->attitude_control(0x4B,0,0.3,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-0.3,0,0);
                            }                           
                        }
                        break;

                    }


                    case 1://take off and forward, serach_flight_height_2, cross_forward_distance12,to park
                    {
                       switch(cross_state)
                        {
                        case 0:
                        {
                            if (drone->flight_status==1)//standby
                            { 
                                drone->takeoff();  //takeoff
                            }
                            else if(drone->flight_status==3)//in air
                            {
                                if ( (ob_distance[0]<serach_flight_height_2-0.1))
                                {
                                    flying_height_control_tracking += 0.01; //init 0.003,speed up 
                                }
                                else if ( (ob_distance[0]>serach_flight_height_2+0.1)&&ob_distance[0]<5)
                                { 
                                    flying_height_control_tracking -= 0.01; //init 0.003
                                }
                                drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

                                if((ob_distance[0]>serach_flight_height_2-0.2)&&ob_distance[0]<4)
                                {
                                    cross_state = 1;
                                }  
                            }

                        break;
                        }
                        case 1:
                        {
														/*
                            float delta_g_pos_x = g_pos_x - last_g_pos_x;
                            //	    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                            if(abs(delta_g_pos_x)<cross_forward_distance12) //for parking 5
                            {
                                drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0, 0, 0, 0);
                                //		writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//	
                               // writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                               // writeF<<"start yaw = "<<start_yaw<<endl;
                                state_in_mission=2;
                                cross_state = 0;
				                                 //2018-08-19 night
                       //         start_searching.data = park_or_circle_2; //for park
														
                            }	
													*/	
			      for(int i = 0; i < count_to_forward12; i ++)
			      {
			      if(i < count_to_forward12)
				  drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
			      else
				  drone->attitude_control(0x4B, 0, 0, 0, 0);
			      usleep(10000);
			      }
					
			      state_in_mission=2;
                              cross_state = 0;


                        break;
                        }			 
                        default:
                            break;
                        
                        }		       
                        break;	
                    }


                    case 2: //for parking pad 2, park_or_circle_3 set gimbal , park_or_circle_3
                    {		       
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);
			    
                            if(flip_once)
                            {
				                        last_g_pos_x = g_pos_x;

                                writeF <<"X="<<g_pos_x << endl;

                                state_in_mission=3;

                                flip_once=false;

                                // park_or_circle_3 set gimbal
                                if(park_or_circle_3) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                                //2018-08-19 night
                                start_searching.data = true; // park_or_circle_3; //for park
                            }
                        }
                        else //keep searching
                        {
                            if(park_direction_2) //right
                            {
                                drone->attitude_control(0x4B,0,0.3,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-0.3,0,0);
                            }   	
                        }
                        break;
                    }

                    case 3://take off and forward, target_flight_height_2 = 0 ,serach_flight_height_3, cross_forward_distance23
                    {      
                        switch(cross_state)
                        {
                            case 0:
                            {
                                if(park_or_circle_3 == 0)
                                {
				    if (drone->flight_status==1)//standby
				    { 
					drone->takeoff();  //takeoff
				    }
				    else if(drone->flight_status==3)//in air
				    {
					if ( (ob_distance[0]<serach_flight_height_3-0.1))
					{
					flying_height_control_tracking += 0.01; //init 0.003,speed up 
					}
					else if ( (ob_distance[0]>serach_flight_height_3+0.1)&&ob_distance[0]<5)
					{ 
					flying_height_control_tracking -= 0.01; //init 0.003
					}
					drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

					if((ob_distance[0]>serach_flight_height_3-0.2)&&ob_distance[0]<4)
					{
					cross_state = 1;
					}  
				    }
                                }
                                else
                                {
                                    if (drone->flight_status==1)//standby
				    { 
					drone->takeoff();  //takeoff
				    }
				    else if(drone->flight_status==3)//in air
				    {
					if ( (ob_distance[0]<1.2)&& (go_down == 0))
					{
					flying_height_control_tracking += 0.003; //init 0.003,speed up 
					}			                                
					drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

					if((ob_distance[0]>1.0)&&ob_distance[0]<4)
					{		                                    
					    go_down = 1;
					} 

            if(go_down)
            {
					    if ( (ob_distance[0]<serach_flight_height_3-0.1))
					    {
					    flying_height_control_tracking += 0.01; //init 0.003,speed up 
					    }
					    else if ( (ob_distance[0]>serach_flight_height_3+0.1)&&ob_distance[0]<5)
					    { 
					    flying_height_control_tracking -= 0.003; //init 0.003
					    }
					    drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);
					
                                             if((ob_distance[0]<1.2)&&ob_distance[0]<4)
					      {		                                    
						go_down = 0;
                                                cross_state = 1;
					      } 
                                         }
				     }
                                }

                                break;
                            }
                            case 1:
                            {
                                float delta_g_pos_x = g_pos_x - last_g_pos_x;
                             	 // writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                                if(abs(delta_g_pos_x)<cross_forward_distance23) //for parking 5
                                {
                                drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                }
                                else
                                {
                                drone->attitude_control(0x4B,0, 0, 0, 0);
                                //writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//	
                               // writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                                state_in_mission=5;
                                cross_state = 0;

                                }			  
                                break;
                            }			 
                            default:
                            break;

                        }		       
                        break;
              
                    }
                    
                    case 4: //park_or_circle_3 set park_or_cross,serach_flight_height_3,park_or_circle_4 set gimbal , park_or_circle_4
                    {
                        if(Detection_fg) //if detecting!
                        {
 //                           searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_3) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_3,Detection_fg,drone);

                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_3,Detection_fg,drone);
                            }

                            if(flip_once)
                            {
				                        last_g_pos_x = g_pos_x;
                                state_in_mission=5;
                                flip_once=false;
				          //writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
				
                 //				flying_height_control_tracking = serach_flight_height_3;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_4) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                                //2018-08-19 night
                                start_searching.data = park_or_circle_4; //for park
                            }
                       }
//                        else //keep searching
//                         {	      
//                             if(park_direction_3) //right
//                             {
//                                 drone->attitude_control(0x4B,0,searching_velocity,0,0);
//                             }
//                             else
//                             {
//                                 drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
//                             }           
//                         }
                        break;
                    }

                     case 5://take off and forward,park_or_circle_3 set park_or_cross,target_flight_height_3 ,serach_flight_height_4, cross_forward_distance34 
                    {
                        mission_function_flag = mission_function(park_or_circle_3,1.2,cross_forward_distance34,serach_flight_height_4,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 6;
                            mission_function_flag = false;
                        } 
                        break;  
                        /*
                         // park_or_circle_3 set park_or_cross
                        if(park_or_circle_3) //cross
                        {
                            switch(cross_state)
                            {
                                case 0:
                                {
                                    // go up/down
                                    if ( ob_distance[0]<target_flight_height_3-0.1)
                                    {
                                        flying_height_control_tracking += 0.003;   //initial 0.003
                                    }
                                    else if ( (ob_distance[0]>target_flight_height_3+0.1)&&ob_distance[0]<10)
                                    {
                                        flying_height_control_tracking -= 0.003;   //initial 0.003
                                    }
                                    drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
                                    if(abs(target_flight_height_3-ob_distance[0])<0.2)
                                    {
                                        cross_state = 1;
                                    }
                                    break;
                                }
                                case 1:
                                {
                                //cross you can also use pos_x to limit the x distance
                                    //for example
                                    float delta_g_pos_x = g_pos_x - last_g_pos_x;
                                    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                                    if(abs(delta_g_pos_x)<cross_forward_distance34) //for parking 5
                                    {
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    //usleep(10000);
                                    }
                                    else
                                    {
                                        cross_state = 2;
                                    writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//
                                    }				    
                                break;  
                                }
                                case 2:
                                {			     
                                    if ( (ob_distance[0]<serach_flight_height_4-0.1))
                                    {
                                        flying_height_control_tracking += 0.01;
                                    }
                                    else if ( (ob_distance[0]>serach_flight_height_4+0.1)&&ob_distance[0]<5)
                                    { 
                                        flying_height_control_tracking -= 0.01;
                                    }
                                    drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                                    if((ob_distance[0]>serach_flight_height_4 - 0.2)&&ob_distance[0]<4)
                                    {
                                        cross_state = 0;
                                        //writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                                        state_in_mission=6;
                                    }
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                        else //park
                        {
                            switch(cross_state)
                            {
                                case 0:
                                {
                                    if (drone->flight_status==1)//standby
                                    { 
                                        drone->takeoff();  //takeoff
                                    }
                                    else if(drone->flight_status==3)//in air
                                    {
                                    
                                        if ( (ob_distance[0]<serach_flight_height_4-0.1))
                                        {
                                        flying_height_control_tracking += 0.006; //init 0.003,speed up 
                                        }
                                        else if ( (ob_distance[0]>serach_flight_height_4+0.1)&&ob_distance[0]<5)
                                        { 
                                        flying_height_control_tracking -= 0.006; //init 0.003
                                        }
                                        drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

                                        if((ob_distance[0]>serach_flight_height_4)&&ob_distance[0]<4)
                                        {
                                            cross_state = 1;
                                        }  
                                    }
                                    break;
                                }
                                case 1:
                                {			    
                                    float delta_g_pos_x = g_pos_x - last_g_pos_x;
                                        //	    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                                    if(abs(delta_g_pos_x)<cross_forward_distance34) //for parking 5
                                    {
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    }
                                    else
                                    {
                                    drone->attitude_control(0x4B,0, 0, 0, 0);
                                        //		writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//	
                                    writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                                    
                                    state_in_mission=6;
                                    cross_state = 0;
                                    }			  
                                break;
                                }			 
                                default:
                                    break;			 
                            }		       
                        }
                                break;
                                */
                    }

                    case 6: //park_or_circle_4 set park_or_cross,serach_flight_height_4,park_or_circle_5 set gimbal , park_or_circle_5
                    {
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_4) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_4,Detection_fg,drone);
                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_4,Detection_fg,drone);
                            }

                            if(flip_once)
                            {
                                state_in_mission=7; 
				                        last_g_pos_x = g_pos_x;
                                flip_once=false;

                                //2018-08-19 night
                             //   flying_height_control_tracking = serach_flight_height_4;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_5) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                            
                                start_searching.data = park_or_circle_5; 
                            }
                        }
                        else //keep searching
                        {		   
                          if(cross_direction_4) //right
                            {
                                drone->attitude_control(0x4B,-0.1,searching_velocity,0,0); /////??????????????????
                            }
                            else
                            {
                                drone->attitude_control(0x4B,-0.1,-1*searching_velocity,0,0);  ///??????????????????????
                            }          
                        }
                        break;
                    }
           	
                    case 7://take off and forward,park_or_circle_4 set park_or_cross,target_flight_height_4 ,serach_flight_height_5, cross_forward_distance45 
                    {
                        mission_function_flag = mission_function(park_or_circle_4,target_flight_height_4,cross_forward_distance45,serach_flight_height_5,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 8;
                            mission_function_flag = false;
                        } 
                        break; 
                        /*
                        if(park_or_circle_4) //cross
                        {
                            switch(cross_state)
                            {
                                case 0:
                                {
                                    // go up/down
                                    if ( ob_distance[0]<target_flight_height_4-0.1)
                                    {
                                        flying_height_control_tracking += 0.003;   //initial 0.003
                                    }
                                    else if ( (ob_distance[0]>target_flight_height_4+0.1)&&ob_distance[0]<10)
                                    {
                                        flying_height_control_tracking -= 0.003;   //initial 0.003
                                    }
                                    drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
                                    if(abs(target_flight_height_4-ob_distance[0])<0.2)
                                    {
                                        cross_state = 1;
                                    }
                                    break;
                                }
                                case 1:
                                {
                                //cross you can also use pos_x to limit the x distance
                                    //for example
                                    float delta_g_pos_x = g_pos_x - last_g_pos_x;
                                    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                                    if(abs(delta_g_pos_x)<cross_forward_distance45) //for parking 5
                                    {
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    //usleep(10000);
                                    }
                                    else
                                    {
                                        cross_state = 2;
                                    writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//
                                    }				    
                                break;  
                                }
                                case 2:
                                {			        
                                    if ( (ob_distance[0]<serach_flight_height_5-0.1))
                                    {
                                        flying_height_control_tracking += 0.01;
                                    }
                                    else if ( (ob_distance[0]>serach_flight_height_5+0.1)&&ob_distance[0]<5)
                                    { 
                                        flying_height_control_tracking -= 0.01;
                                    }
                                    drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                                    if((ob_distance[0]>serach_flight_height_5 - 0.2)&&ob_distance[0]<4)
                                    {
                                        cross_state = 0;
                                        writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                                        state_in_mission=8;
                                    }
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                        else //park
                        {
                               switch(cross_state)
                            {
                                case 0:
                                {
                                    if (drone->flight_status==1)//standby
                                    { 
                                        drone->takeoff();  //takeoff
                                    }
                                    else if(drone->flight_status==3)//in air
                                    {
                                    
                                        if ( (ob_distance[0]<serach_flight_height_5-0.1))
                                        {
                                        flying_height_control_tracking += 0.006; //init 0.003,speed up 
                                        }
                                        else if ( (ob_distance[0]>serach_flight_height_5+0.1)&&ob_distance[0]<5)
                                        { 
                                        flying_height_control_tracking -= 0.006; //init 0.003
                                        }
                                        drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

                                        if((ob_distance[0]>serach_flight_height_5)&&ob_distance[0]<4)
                                        {
                                            cross_state = 1;
                                        }  
                                    }
                                    break;
                                }
                                case 1:
                                {			    
                                    float delta_g_pos_x = g_pos_x - last_g_pos_x;
                                        //	    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                                    if(abs(delta_g_pos_x)<cross_forward_distance45) //for parking 5
                                    {
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    }
                                    else
                                    {
                                    drone->attitude_control(0x4B,0, 0, 0, 0);
                                        //		writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//	
                                    writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
                                    
                                    state_in_mission=8;
                                    cross_state = 0;
                                    }			  
                                break;
                                }			 
                                default:
                                    break;			 
                            }	 
                        }
                         break;
                         */
                    }
                    
                    case 8: //park_or_circle_5 set park_or_cross,serach_flight_height_5,park_or_circle_6 set gimbal , park_or_circle_6
                    {		       
                        if(Detection_fg) //if detecting!
                        {
     //                       searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_5) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_5,Detection_fg,drone);
                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_5,Detection_fg,drone);
                            }
			    
                            if(flip_once)
                            {
				last_g_pos_x = g_pos_x;
                                state_in_mission=9;
                                flip_once=false;

                                   //2018-08-19 night
                             //   flying_height_control_tracking = serach_flight_height_4;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_6) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                            
                                start_searching.data = park_or_circle_6; 
                            }
                        }
                        else //keep searching
                        {	      
                          if(park_direction_5) //right
                            {
                                drone->attitude_control(0x4B,0,searching_velocity,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
                            }         
                        }
                        break;
                    }

                    case 9://take off and forward ,park_or_circle_5 set park_or_cross,target_flight_height_5 , cross_forward_distance56 ,serach_flight_height_6
                    {
                        mission_function_flag = mission_function(park_or_circle_5,target_flight_height_5,cross_forward_distance56,serach_flight_height_6,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 10;
                            mission_function_flag = false;
                        } 
                        break;                  
                    }

                    case 10:  //park_or_circle_6 set park_or_cross,serach_flight_height_6,park_or_circle_7 set gimbal , park_or_circle_7
                    {
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_6) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_6,Detection_fg,drone);
                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_6,Detection_fg,drone);
                            }

                            if(flip_once)
                            {
				last_g_pos_x = g_pos_x;
                                state_in_mission=11; 
                                flip_once=false;

                                       //2018-08-19 night
                             //   flying_height_control_tracking = serach_flight_height_4;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_7) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                            
                                start_searching.data = park_or_circle_7; 
                            }
                        }
                        else //keep searching
                        {		   
                          if(cross_direction_6) //right
                            {
                                drone->attitude_control(0x4B,0,searching_velocity,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
                            }        
                        }
                        break;
                    }

                    case 11://take off and forward ,park_or_circle_6 set park_or_cross,target_flight_height_6 , cross_forward_distance67 ,serach_flight_height_7
                    {
                        mission_function_flag = mission_function(park_or_circle_6,target_flight_height_6,cross_forward_distance67,serach_flight_height_7,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 12;
                            mission_function_flag = false;
                        }
                        break;   
                    }
                    
                    case 12:  //park_or_circle_7 set park_or_cross,serach_flight_height_7,park_or_circle_8 set gimbal , park_or_circle_8
                    {
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_7) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_6,Detection_fg,drone);
                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_6,Detection_fg,drone);
                            }

                            if(flip_once)
                            {
                                state_in_mission=13;
				last_g_pos_x = g_pos_x;
                                flip_once=false;

                                //2018-08-19 night
                             //   flying_height_control_tracking = serach_flight_height_4;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_8) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                                
                                start_searching.data = park_or_circle_8; 
                            }
                        }
                        else //keep searching
                        {		   
                          if(cross_direction_7) //right
                            {
                                drone->attitude_control(0x4B,0,searching_velocity,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
                            }   
                        }
                        break;
                    }

                    case 13://take off and forward ,park_or_circle_7 set park_or_cross,target_flight_height_7 , cross_forward_distance78 ,serach_flight_height_8
                    {
                        mission_function_flag = mission_function(park_or_circle_7,target_flight_height_7,cross_forward_distance78,serach_flight_height_8,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 14;
                            mission_function_flag = false;
                        } 
                        break;
                    }

                    case 14: //park_or_circle_8 set park_or_cross,serach_flight_height_8,park_or_circle_9 set gimbal , park_9
                    {		       
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            // park_or_circle_3 set park_or_cross
                            if(park_or_circle_8) //cross
                            {
                                flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,serach_flight_height_8,Detection_fg,drone);
                            }
                            else //park
                            {
                                flip_once = parking(filtered_x,filtered_y,serach_flight_height_8,Detection_fg,drone);
                            }

                            if(flip_once)
                            {
				last_g_pos_x = g_pos_x;
                                state_in_mission=15;
                                flip_once=false;

                                     //2018-08-19 night
                             //   flying_height_control_tracking = serach_flight_height_4;
                                // park_or_circle_3 set gimbal
                                if(park_or_circle_9) //cross
                                {
                                    drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                                }
                                else //park
                                {
                                    drone->gimbal_angle_control ( 0,-900,0,20 ); 
                                }
                            
                                start_searching.data = park_or_circle_9; 
                            }
                        }
                        else //keep searching
                        {	      
                          if(park_direction_8) //right
                            {
                                drone->attitude_control(0x4B,0,searching_velocity,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
                            }   
                        }
                        break;
                    }
                    
                    case 15://take off and forward ,park_or_circle_8 set park_or_cross,target_flight_height_8 , cross_forward_distance89 ,serach_flight_height_9
                    {
                        mission_function_flag = mission_function(park_or_circle_8,target_flight_height_8,cross_forward_distance89,serach_flight_height_9,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 16;
                            mission_function_flag = false;
                        } 
                        break;
                    }

                    case 16: //for parking pad 9
                    {		       
                        if(Detection_fg) //if detecting!
                        {
                            searching_state = 0;

                            flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);
			    
                            if(flip_once)
                            {
				last_g_pos_x = g_pos_x;
                                state_in_mission=17;
                                flip_once=false;

                                drone->gimbal_angle_control ( 0,0,0,20 ); //for cross
                            }
                        }
                        else //keep searching
                        {	      
                            if(park_direction_9) //right
                            {
                                drone->attitude_control(0x4B,0,searching_velocity,0,0);
                            }
                            else
                            {
                                drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
                            }   
                        }
                        break;
                    }

                    case 17://take off and forward ,park_or_circle_9 set park_or_cross,target_flight_height_9 , cross_forward_distance910 ,serach_flight_height_tags
                    {
                        mission_function_flag = mission_function(park_or_circle_9,target_flight_height_9,cross_forward_distance910,serach_flight_height_tags,last_g_pos_x,drone);
                        if(mission_function_flag)
                        {
                            state_in_mission = 18;
                            mission_function_flag = false;

                            mission_type.data=true;//tags
                        } 
                        break;
                    }
/*
                    case 18:// find tags
                    {
                        if(abs(drone->yaw_from_drone-start_yaw)*57.3>6)
                        {
                        if((drone->yaw_from_drone-start_yaw)*57.3>6) //2018-08-11
                            drone->attitude_control(0x4c,0,0,0,-1);  //9c is height control
                        if((drone->yaw_from_drone-start_yaw)*57.3<-6)
                            drone->attitude_control(0x4c,0,0,0,1); 
                        }
                        else
                        {
                        switch (tags_searching_state) 
                        {
                            case 0://1.first right
                            {
                                    if(tags_searching_count-bound_len > 0)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
					tags_searching_count = 0;
                                        tags_searching_state = 1;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.25,0,0);
					tags_searching_count++;
                                    }
                                    break;
                            }
                            case 1://2.then left
                            {
                                    if(tags_searching_count-bound_len > 0)
                                    {
					last_g_pos_x = g_pos_x;
                                        drone->attitude_control(0x4B,0,0,0,0);
					tags_searching_count = 0;
                                        searching_state = 2;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,-0.25,0,0);
					tags_searching_count++;
                                    }
                                    break;
                            }
                            case 2://3.then keep height and go forward
                            {
                                drone->attitude_control(0x4B,0,0,0,0);

                                for(int i = 0; i < count_forward_tags; i ++)
                                {
                                    if(i < count_forward_tags)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                    usleep(10000);
                                }


//                                tags_searching_state=3;
				  tags_searching_state=2;//for tags
                                break;
                            }
                            case 3://4.then right
                            {
                                    if(tags_searching_count-bound_len > 0)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
					tags_searching_count = 0;
                                        tags_searching_state = 4;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.25,0,0);
					tags_searching_count++;
                                    }
                                    break;
                            }
                            case 4://5.then left
                            {
                                    if(tags_searching_count-bound_len > 0)
                                    {
					last_g_pos_x = g_pos_x;
                                        drone->attitude_control(0x4B,0,0,0,0);
					tags_searching_count = 0;
                                        tags_searching_state = 5;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,-0.25,0,0);
					tags_searching_count++;
                                    }
                                    break;
                            }
                            case 5://6.then keep height and go forward
                            {
                                drone->attitude_control(0x4B,0,0,0,0);
                                for(int i = 0; i < count_forward_tags; i ++)
                                {
                                    if(i < count_forward_tags)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                    usleep(10000);
                                }
                                tags_searching_state=6;
                                break;
                            }
                            case 6://7.then right
                            {
                                    
                                    if(tags_searching_count-bound_len > 0)
                                    {
					last_g_pos_x = g_pos_x;
                                        drone->attitude_control(0x4B,0,0,0,0);
					tags_searching_count = 0;
                                        tags_searching_state = 7;
					 
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.25,0,0);
					tags_searching_count++;
                                    }
                                    break;
                            }
                            case 7:// go up 2.8m and then forward 
                            {
                                //Head down at the beginning.row,picth,yaw,duration
                                drone->gimbal_angle_control ( 0,-900,0,20 ); 

                                if ( (ob_distance[0]<serach_flight_height_1-0.1))
                                {
                                    flying_height_control_tracking += 0.003;
                                }
                                else if ( (ob_distance[0]>serach_flight_height_1+0.1)&&ob_distance[0]<5)
                                { 
                                    flying_height_control_tracking -= 0.003;
                                }
                                drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                                if((ob_distance[0]>serach_flight_height_1 - 0.2)&&ob_distance[0]<4) 
                                {
                                    for(int i = 0; i < count_forward_tags; i ++)
                                    {
                                        if(i < count_forward_tags)
                                        drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                        else
                                        drone->attitude_control(0x4B, 0, 0, 0, 0);
                                        usleep(10000);
                                    }
                                 
                                        if(i < count_forward_tags)
                                        drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                        else
                                        drone->attitude_control(0x4B, 0, 0, 0, 0);
                                        usleep(10000);
                                    }
                                    tags_searching_state=0;
				    start_searching.data=false; //for nums
				    mission_type.data=false; 
				    state_in_mission=19;
                                }
                                break;	
                            }
                            default:
                                break;
                        }
                        break;   tags_searching_state=0;
				    start_searching.data=false; //for nums
				    mission_type.data=false; 
				    state_in_mission=19;
                                }
                                break;	
                            }
                            default:
                                break;
                        }
                        break;
		      }
                    }
*/

		    case 18:// find tags
                    {
		      if(abs(drone->yaw_from_drone-start_yaw)*57.3>6)
                        {
                        if((drone->yaw_from_drone-start_yaw)*57.3>6) //2018-08-11
                            drone->attitude_control(0x4c,0,0,0,-1);  //9c is height control
                        if((drone->yaw_from_drone-start_yaw)*57.3<-6)
                            drone->attitude_control(0x4c,0,0,0,1); 
                        }
                      else
                      {
                        switch (tags_searching_state) 
                        {
                            case 0://1.first right
                            {
                                    if(ul_distance[2] <1.5 && ul_reliability[2]==1)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        tags_searching_state = 1;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.25,0,0);
                                    }
                                    break;
                            }
                            case 1://2.then left
                            {
                                    if(ul_distance[4] <1.5 && ul_reliability[4]==1)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 2;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,-0.25,0,0);
                                    }
                                    break;
                            }
                            case 2://3.then keep height and go forward
                            {
                                drone->attitude_control(0x4B,0,0,0,0);

                                for(int i = 0; i < count_forward_tags; i ++)
                                {
                                    if(i < count_forward_tags)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                    usleep(10000);
                                }

//                                tags_searching_state=3;
				tags_searching_state=2;//for tags
                                break;
                            }
                            case 3://4.then right
                            {
                                    if(ul_distance[2] <1.5 && ul_reliability[2]==1)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        tags_searching_state = 4;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.3,0,0);
                                    }
                                    break;
                            }
                            case 4://5.then left
                            {
                                    if(ul_distance[4] <1.5 && ul_reliability[4]==1)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        searching_state = 5;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,-0.3,0,0);
                                    }
                                    break;
                            }
                            case 5://6.then keep height and go forward
                            {
                                drone->attitude_control(0x4B,0,0,0,0);
                                for(int i = 0; i < count_forward_tags; i ++)
                                {
                                    if(i < count_forward_tags)
                                    drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                    else
                                    drone->attitude_control(0x4B, 0, 0, 0, 0);
                                    usleep(10000);
                                }
                                tags_searching_state=6;
                                break;
                            }
                            case 6://7.then right
                            {
                                    if(ul_distance[2] <1.5 && ul_reliability[2]==1)
                                    {
                                        drone->attitude_control(0x4B,0,0,0,0);
                                        tags_searching_state = 7;
                                    }
                                    else
                                    {
                                        drone->attitude_control(0x4B,0,0.3,0,0);
                                    }
                                    break;
                            }
                            case 7:// go up 2.8m and then forward 
                            {
                                //Head down at the beginning.row,picth,yaw,duration
                                drone->gimbal_angle_control ( 0,-900,0,20 ); 

                                if ( (ob_distance[0]<serach_flight_height_1-0.1))
                                {
                                    flying_height_control_tracking += 0.003;
                                }
                                else if ( (ob_distance[0]>serach_flight_height_1+0.1)&&ob_distance[0]<5)
                                { 
                                    flying_height_control_tracking -= 0.003;
                                }
                                drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                                if((ob_distance[0]>serach_flight_height_1 - 0.2)&&ob_distance[0]<4) 
                                {
                                    for(int i = 0; i < count_forward_tags; i ++)
                                    {
                                        if(i < count_forward_tags)
                                        drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                                        else
                                        drone->attitude_control(0x4B, 0, 0, 0, 0);
                                        usleep(10000);
                                    }
                                    tags_searching_state=0;
				    
                                }
                                break;	
                            }
                            default:
                                break;
                        }
                        break;
		      }
                    }
                    
		    case 19:
		    {
		      if(Detection_fg) //if detecting!
		      {
			  searching_state = 0;

			  flip_once = parking(filtered_x,filtered_y,serach_flight_height_1,Detection_fg,drone);

			  if(flip_once)
			  {
			      state_in_mission=20;
			      flip_once=false;
			      
			  }
		      }
		      else //keep searching
		      {	      
			  if(park_direction_1) //right
			  {
                              drone->attitude_control(0x4B,0,searching_velocity,0,0);
			  }
			  else
			  {
			      drone->attitude_control(0x4B,0,-1*searching_velocity,0,0);
			  }    
		      }
		      break;
                            
		    }  
                    case 20: //land
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
                    flying_height_control_tracking += 0.003;// initial 0.003
                }
                else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
                { 
                    flying_height_control_tracking -= 0.003;// initial 0.003
                }
                drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);
            }
            else if(abs(cmd_fligh_x)<0.12&&abs(cmd_flight_y)<0.12) //samll adjust
            {
	      
		  if ( ob_distance[0]<2.3)
		  {
		    flying_height_control_tracking += 0.015;   //init 0.008,speed up landing
		  }
		  else if ( (ob_distance[0]>2.6)&&ob_distance[0]<6) 
		  { 
		    flying_height_control_tracking -= 0.015;    //init 0.008
		  }
		  drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);   //TODO: height adjusting

		  if(ob_distance[0]<2.5)
		  {
		      drone->landing();
		      sleep(1);   //wait for airscrew still
		      
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

bool cbarrier(float &cmd_fligh_x,float &cmd_flight_y, float &cmd_yaw,float height, uint8_t detection_flag,DJIDrone* drone)
{
  //for a test 
  cmd_yaw = 0;

   writeF<<"0ob_distance[0]="<<ob_distance[0]<<endl;

  if(abs(drone->yaw_from_drone-start_yaw)*57.3>6)
  {
      if((drone->yaw_from_drone-start_yaw)*57.3>6) //2018-08-11
	drone->attitude_control(0x4c,0,0,0,-1);  //9c is height control
      if((drone->yaw_from_drone-start_yaw)*57.3<-6)
	drone->attitude_control(0x4c,0,0,0,1); 
  }
  else
  {
 
      if(cbring_mode==0)  //turn the gimbal direction
      {
//	drone->gimbal_angle_control(0,0,0,20);   //look farward.
	
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
	  flying_height_control_tracking += 0.003;
	}   
	else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
	{ 
	  flying_height_control_tracking -= 0.003;
	}
/*	
	if(detection_flag)
	{
	  if(cmd_flight_y>0.2)  cmd_flight_y = 0.2;    //initial 0.25
	  if(cmd_flight_y<-0.2) cmd_flight_y =-0.2;    //initial 0.25
    //      if(abs(cmd_yaw)<5&&cmd_flight_y<0.2)       //2018-08-11 night
	  if(abs(cmd_yaw)<5&&abs(cmd_flight_y)<0.2) 
	    drone->attitude_control ( 0x4B,0.0,cmd_flight_y,0,cmd_yaw);   //initail x = 0.2
	  else
	    drone->attitude_control ( 0x4B,0.0,cmd_flight_y,0,cmd_yaw);
	  writeF<<"tag detected, cmd_yaw="<<cmd_yaw<<endl;
	}
	else   
	{
	  cmd_fligh_x=0.0; cmd_flight_y=0.0;
	  drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
	  writeF<<"tag NOT detected, cmd_yaw="<<cmd_yaw<<endl;   //yaw is not zero for no tag, need to be fixed.
	}
	
	  //    writeF<<"cross circle:cmd_fligh_x="<<cmd_fligh_x<<endl;
        if(detection_flag&&cmd_fligh_x>0.5&&cmd_fligh_x<2.5 &&abs(cmd_flight_y)<0.1&&abs(cmd_yaw)<5)// use cmd_filgh_x to filter false
  //    if(detection_flag&&abs(cmd_flight_y)<0.02&&abs(cmd_yaw)<5)// maybe should change //init 0.2 unstable
	{
	  writeF<<"tag detected, cmd_fligh_x="<<cmd_fligh_x<<endl;
	  writeF<<"tag detected, cmd_fligh_y="<<cmd_flight_y<<endl;
	  writeF<<"alined..."<<endl;
	  cbring_mode=0;
		cmd_fligh_x = 0; //
	  return true;
	} 
	*/

/*
	if(detection_flag)
	{
	  if(cmd_flight_y>0.2)  cmd_flight_y = 0.2;
	  if(cmd_flight_y<-0.2) cmd_flight_y =-0.2;
	  if(abs(cmd_yaw)<5&&cmd_flight_y<0.2)
	    drone->attitude_control ( 0x5B,-0.1,cmd_flight_y,flying_height_control_tracking,cmd_yaw);
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
	{
	   writeF<<"tag detected, cmd_x="<<cmd_fligh_x<<endl;
	  writeF<<"tag detected, cmd_y="<<cmd_flight_y<<endl;
	  writeF<<"alined..."<<endl;
	  cbring_mode=0;
	  return true;
	}
*/
      } 
 }

  return false;
}

bool mission_function(bool park_or_circle,float target_flight_height,float cross_forward_distance,float serach_flight_height_next, float last_g_posx,DJIDrone* drone)                   
{
    if(park_or_circle) //cross
    {

        switch(cross_state)
        {
            case 0:
            {
                // go up/down
                if ( ob_distance[0]<target_flight_height-0.1)
                {
                    flying_height_control_tracking += 0.015;   //initial 0.003
                }
                else if ( (ob_distance[0]>target_flight_height+0.1)&&ob_distance[0]<10)
                {
                    flying_height_control_tracking -= 0.015;   //initial 0.003
                }
                drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
                if(abs(target_flight_height-ob_distance[0])<0.2)
                {
                    cross_state = 1;
                }
                break;
            }
            case 1:
            {
            //cross you can also use pos_x to limit the x distance
                //for example
                float delta_g_pos_x = g_pos_x - last_g_posx;
                writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                if(abs(delta_g_pos_x)<cross_forward_distance) //for parking 5
                {
                drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                //usleep(10000);
                }
                else
                {
                    cross_state = 2;
		    writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//
		   
		    //2018-08-20 for high to low,flying_height_control_tracking is too big
		    flying_height_control_tracking = target_flight_height;
                }				    
            break;  
            }
            case 2:
            {			        

                if ( (ob_distance[0]<serach_flight_height_next-0.1))
                {
                    flying_height_control_tracking += 0.01;
                }
                else if ( (ob_distance[0]>serach_flight_height_next+0.1)&&ob_distance[0]<5)
                { 
                    flying_height_control_tracking -= 0.01;
                }
                drone->attitude_control(0x5B,0,0,flying_height_control_tracking,0);

                if(abs(ob_distance[0]-serach_flight_height_next)<0.1&&ob_distance[0]<4)
                {
                    cross_state = 0;
                    writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;

                    return true;
                }
                break;
            }
            default:
                break;
        }
    }
    else //park
    {
        switch(cross_state)
        {
            case 0:
            {
	        if(serach_flight_height_next >1.5)
		{
		   if (drone->flight_status==1)//standby
		    { 
			drone->takeoff();  //takeoff
		    }
		    else if(drone->flight_status==3)//in air
		    {
		    
		      
			    if ( (ob_distance[0]<serach_flight_height_next-0.1))
			    {
			    flying_height_control_tracking += 0.012; //init 0.003,speed up 
			    }
			    else if ( (ob_distance[0]>serach_flight_height_next+0.1)&&ob_distance[0]<5)
			    { 
			    flying_height_control_tracking -= 0.012; //init 0.003
			    }
			    drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);
			    if((ob_distance[0]>serach_flight_height_next)&&ob_distance[0]<4)
			    {
				cross_state = 1;
			    }  
		    }
		}
                else
		{
		  if (drone->flight_status==1)//standby
		  { 
		      drone->takeoff();  //takeoff
		  }
		  else if(drone->flight_status==3)//in air
		  {
		    // if ( (ob_distance[0]<1.2)&& (go_down == 0))
		   //  {
		   //   flying_height_control_tracking += 0.003; //init 0.003,speed up 
		    //  }			                                
		   //   drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);

		      if((ob_distance[0]>1.0)&&ob_distance[0]<4)
		      {		                                    
			  go_down = 1;
		      } 

		      if(go_down)
		      {
			  if ( (ob_distance[0]<serach_flight_height_next-0.1))
			  {
			  flying_height_control_tracking += 0.01; //init 0.003,speed up 
			  }
			  else if ( (ob_distance[0]>serach_flight_height_next+0.1)&&ob_distance[0]<5)
			  { 
			  flying_height_control_tracking -= 0.003; //init 0.003
			  }
			  drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);
		      
			  if((ob_distance[0]<serach_flight_height_next)&&ob_distance[0]<4)
			  {		                                    
			    go_down = 0;
			    cross_state = 1;
			  } 
		      }
		    }
		  
		}
                break;
            }
            case 1:
            {			    
                float delta_g_pos_x = g_pos_x - last_g_posx;
                    //	    writeF << "delta_g_pos_x="<<delta_g_pos_x << endl;//
                if(abs(delta_g_pos_x)<cross_forward_distance) //for parking 5
                {
                drone->attitude_control(0x4B,0.7,0,0,0);//NOT 0X42
                }
                else
                {
		  drone->attitude_control(0x4B,0, 0, 0, 0);
		      //		writeF << "delta_g_pos_x1="<<delta_g_pos_x << endl;//	
		  writeF <<"X="<<g_pos_x<<"  Y="<< g_pos_y <<"  Z="<<g_pos_z << endl;
		  
		  cross_state = 0;

		  return true;
                }			  
            break;
            }			 
            default:
                break;			 
        }	 
    }
    return false;
}

// void searching_right(float &pos_y,float &bound_length, int state,DJIDrone* drone)
// {
// 
// }
// void searching_left(float &pos_y,float &bound_length, int state,DJIDrone* drone)
// {
//   
// }
