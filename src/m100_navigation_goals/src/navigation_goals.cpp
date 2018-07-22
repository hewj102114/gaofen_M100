#include <ros/ros.h>
#include <stdlib.h>
#include<iostream>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<std_msgs/Bool.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static float ob_distance[5]= {10.0};
static bool start_searching = false;
static bool flip_once = true;
void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
  for ( int i=0; i<5; i++ )
    ob_distance[i]=g_oa.ranges[i];
}

void start_searching_callback ( const std_msgs::Bool &searching_flag )
{

  //ROS_INFO ( "start searching is %d",searching_flag.data );
  start_searching = searching_flag.data;

}

int main ( int argc, char** argv )
{
  ros::init ( argc, argv, "navigation_goals" );
  ros::NodeHandle n;
  ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

  ros::Subscriber start_searching_sub = n.subscribe ( "/dji_sdk_demo/start_searching",10,start_searching_callback );
  //Test filtering

  //tell the action client that we want to spin a thread by default

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";


  int x = 0;
  while ( start_searching==false)
    {
      usleep ( 1000 );

      ros::spinOnce();
    }
  MoveBaseClient ac ( "move_base", true );

  //wait for the action server to come up
  while ( !ac.waitForServer ( ros::Duration ( 5.0 ) ) )
    {
      ROS_INFO ( "Waiting for the move_base action server to come up" );

      ros::spinOnce();
    }


    int count = 0;


  while ( n.ok() )
    {
      // srand(time(NULL));
      // int x = rand() % 8;



      ros::spinOnce();

      switch ( x )
        {
        case 0:
          goal.target_pose.pose.position.x = 4.0;
          goal.target_pose.pose.position.y = 0;
          goal.target_pose.pose.orientation.w = 1.0;
          //   x = 1;
          break;
        case 1:
          goal.target_pose.pose.position.x = 0;
          goal.target_pose.pose.position.y = 4.0;
          goal.target_pose.pose.orientation.w = 1.0;
          //   x = 2;
          break;
        case 2:
          goal.target_pose.pose.position.x = -4.0;
          goal.target_pose.pose.position.y = 0;
          goal.target_pose.pose.orientation.w = 1.0;
          //  x = 3;
          break;
        case 3:
          goal.target_pose.pose.position.x = 0;
          goal.target_pose.pose.position.y = -4.0;
          goal.target_pose.pose.orientation.w = 1.0;
          //   x = 0;
          break;

        }

      goal.target_pose.header.stamp = ros::Time::now();


      ROS_INFO ( "Sending goal..." );
      //  sleep(45);
      ac.sendGoal ( goal );
      if ( ac.waitForResult ( ros::Duration ( 20) ) );
      if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED )
        ROS_INFO ( "Hooray, the base moved 1 meter forward" );

      else if ( ac.getState() == actionlib::SimpleClientGoalState::ABORTED )
        {
          ROS_INFO ( "ABORTED" );
          x ++ ;
        }

      else if ( ac.getState() == actionlib::SimpleClientGoalState::PENDING )
        {
          ROS_INFO ( "PENDING" );
          // x +=3 ;
        }
        count ++;
      if(count%4==0)
	x++;
        
      if(count%6 == 0)
	x--;
      
        

    }
  return 0;


}
