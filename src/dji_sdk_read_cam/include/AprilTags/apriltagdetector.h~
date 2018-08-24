#ifndef APRILTAGDETECTOR_H
#define APRILTAGDETECTOR_H

// AprilTag family 
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

#define USING_OPENCV
#define USING_ROS

#include<std_msgs/Int8.h>
#include<std_msgs/Bool.h>

#include <dji_sdk/Reldist.h>

#include<dji_sdk/DetectionPoints.h>



#include<dji_sdk/TransparentTransmissionData.h>


//#ifdef USING_ROS
#include "ros/ros.h"
#include <ros/publisher.h>
//#endif
//#define _SHOW_PHOTO

#ifndef GIMBAL_USED
#define GIMBAL_USED
#include<dji_sdk/Gimbal.h>
#endif

/*   for gaofen competition, small tags are not needed. only 36h11 used.
#ifndef SMALL_TAG_USED
#define SMALL_TAG_USED
#endif
*/

#ifndef PI
const double PI = 3.14159265358979323846;
#endif

const double TWOPI = 2.0*PI;
static int last_received_apriltag_type = 1;
class ApriltagDetector
{
public:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  int m_mode; // 0: AprilTag detection directly
  // 1: w	ith ROI window
  // 2: with Optical flow tracking

  bool m_draw; // draw image and April tag detections?
  bool m_timing; // print timing information for each tag extraction call
  bool m_isShowResult;

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_frames;
  vector<int> m_win; // [ x_min, x_max, y_min, y_max ]
  bool m_isTracking;
  
 // bool m_usi


#ifdef USING_OPENCV
  cv::VideoCapture m_cap;
#endif

  ros::Publisher m_result_pub;
  ros::Publisher  tag_detections_pub;
  dji_sdk::Reldist rel_dist;
  std_msgs::Int8 m_numOfDetections;
  dji_sdk::DetectionPoints m_detectionPoints;
  ros::Publisher m_detectionPoints_pub;
//Flag: start using small TagSize
  ros::Publisher m_using_smallTags_pub;
  
  ros::Subscriber m_CMD_FROM_DJI_sub;
  ros::Subscriber m_mission_type_sub;
  ros::Subscriber m_state_in_mission_sub;
  ros::Subscriber m_start_searching_sub;
 // ros::Subscriber m_used_apriltag_type_sub;
#ifdef GIMBAL_USED
  ros::Subscriber m_gimbal_sub;
  dji_sdk::Gimbal m_gimbal;
#endif


  //float last_flight_height
  
  uint8_t m_CMD_from_remote; 
  bool m_mission_type;
  bool m_start_searching;
  uint8_t m_state_in_mission;
  std::vector<cv::Point2f> points[2];
  vector<AprilTags::TagDetection> detections;
  bool usingSmallTags;
  int m_used_apriltag_type;
public:
  ApriltagDetector ( ros::NodeHandle& node ) :
    m_tagCodes ( AprilTags::tagCodes36h11 ),
    m_tagDetector ( NULL ),


    m_mode ( 1 ),

    m_draw ( false ),
    m_timing ( true ),
    m_isShowResult ( true ),

    m_width ( 640 ),
    m_height ( 360 ),
    m_tagSize ( 0.163 ),
    m_fx ( 263.5 ), //Specified by x3 Camera;
    m_fy ( 253.6 ),
    m_px ( 318.44 ),
    m_py ( 170 ),
    m_frames ( 0 ),
    m_isTracking ( false ),
    m_CMD_from_remote('W'),
    m_mission_type ( true ),
    m_start_searching(false),
    m_state_in_mission(0),
//    m_used_apriltag_type(1),

    usingSmallTags ( false )
  {

    m_tagDetector = new AprilTags::TagDetector ( m_tagCodes );
    // cv::namedWindow("AprilTag", 1);
    m_result_pub  = node.advertise<dji_sdk::Reldist> ( "apriltag_detection/reldist",10 );
    tag_detections_pub = node.advertise<std_msgs::Bool> ( "tag_detection/detections",10 );
    m_detectionPoints_pub = node.advertise<dji_sdk::DetectionPoints> ( "apriltag_detection/detectionPoints",10 );

    m_using_smallTags_pub = node.advertise<std_msgs::Bool> ( "apriltag_detection/usingSmallTags",10 );
#ifdef GIMBAL_USED
    m_gimbal_sub = node.subscribe ( "dji_sdk/gimbal",10,&ApriltagDetector::gimbal_read_callback,this );
#endif

    m_CMD_FROM_DJI_sub = node.subscribe ( "dji_sdk/data_received_from_remote_device",10,&ApriltagDetector::cmd_from_mobile_callback,this );
    m_mission_type_sub = node.subscribe ( "dji_sdk_demo/mission_type",10,&ApriltagDetector::mission_type_callback,this );
    m_state_in_mission_sub = node.subscribe ("/dji_sdk_demo/state_in_mission",10,&ApriltagDetector::state_in_mission_callback,this);
    m_start_searching_sub = node.subscribe ("/dji_sdk_demo/start_searching",10,&ApriltagDetector::start_searching_callback,this);
//   m_used_apriltag_type_sub = node.subscribe("used_apriltag_type",10,&ApriltagDetector::apriltag_type_change_callback,this);
  } 

  ~ApriltagDetector() {
    
    if(!m_tagDetector)
      delete(m_tagDetector);
  }

  // changing the tag family
  void setTagCodes ( string s );

  // changing the tag size
  void setTagSize ( double size_ )
  {
    m_tagSize = size_;
  }

  // reboot for new video stream
  void reboot();

  void processImage ( cv::Mat& image );
  // get window with (1+2*delta)*TagSize
  std::vector<int> point2win ( cv::Mat image, float delta = 0 );
  
  void print_detections ();
#ifdef GIMBAL_USED
  void gimbal_read_callback ( const dji_sdk::Gimbal & gimbal )
  {
    m_gimbal = gimbal;
  }

#endif
 
  void Line_detection(cv::Mat& image, dji_sdk::Reldist & result);
  void calculate(cv::Mat &img, double & intercept, double & slope, int &flagSituation);

  void Calcu_attitude(cv::Point2f pnt_tl_src, cv::Point2f pnt_tr_src, cv::Point2f pnt_br_src, cv::Point2f pnt_bl_src);
  int Num_detection(cv::Mat &img,cv::Mat mimg,bool flag, dji_sdk::Reldist & pos_result);


void cmd_from_mobile_callback(const dji_sdk::TransparentTransmissionData & transData)
{
  m_CMD_from_remote = transData.data.at(0);
  if('d' == m_CMD_from_remote )
          setTagCodes("36h11");
}
void mission_type_callback(const std_msgs::Bool &mission_data)
{
  m_mission_type=mission_data.data;
}
void start_searching_callback(const std_msgs::Bool &start_searching_data)
{
  m_start_searching=start_searching_data.data;
}

void state_in_mission_callback(const std_msgs::Int8 &state_in_mission)
{
  m_state_in_mission=state_in_mission.data;
}
/*
void apriltag_type_change_callback(const std_msgs::Int8 & used_type)
{
  m_used_apriltag_type = used_type.data;
  if((int)used_type.data != last_received_apriltag_type)
  {
    if(1 == used_type.data)
      setTagCodes("25h9");
    else if(2== used_type.data)
      setTagCodes("36h11");
    else
      setTagCodes("16h5");
  }
}*/

};

#endif // APRILTAGDETECTOR_H
