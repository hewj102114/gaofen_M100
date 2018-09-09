#include "apriltagdetector.h"
#include <sys/time.h>
#include "DenseGraph.h"
#include "SparseGraph.h"
#include "ReadGraph.h"
#include "LazyPrimMST.h"
#include "CircleDetection.h"
#include <string.h>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
ofstream fcout( "/root/circleDetection.txt",ios::app );
ofstream writeF ( "/home/ubuntu/GaofenChallenge/log1.txt");
//ofstream writeF11 ( "/home/ubuntu/GaofenChallenge/logf.txt");

float flight_height = 0.0;
bool change_once_flag = true;  
const float EPS = 0.00000001;      
const int tag25h9 = 1;
//uint8_t CMD = 'W';
/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad ( double t )
{
  if ( t >= 0. )
    {  
      t = fmod ( t+PI, TWOPI ) - PI;     
    }
  else
    {
      t = fmod ( t-PI, -TWOPI ) + PI;
    }
  return t;
}

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic()
{
  struct timeval t;
  gettimeofday ( &t, NULL );
  return ( ( double ) t.tv_sec + ( ( double ) t.tv_usec ) /1000000. );
}
/**
 * Convert rotation matrix to Euler angles
 */

void wRo_to_euler ( const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll )
{
  yaw = standardRad ( atan2 ( wRo ( 1,0 ), wRo ( 0,0 ) ) );
  double c = cos ( yaw );
  double s = sin ( yaw );
  pitch = standardRad ( atan2 ( -wRo ( 2,0 ), wRo ( 0,0 ) *c + wRo ( 1,0 ) *s ) );
  roll  = standardRad ( atan2 ( wRo ( 0,2 ) *s - wRo ( 1,2 ) *c, -wRo ( 0,1 ) *s + wRo ( 1,1 ) *c ) );
}

double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void ApriltagDetector::findSquares(Mat src, vector<vector<Point> > &squares)
{
    Mat pyr, timg, gray0(src.size(), CV_8U), Src_HSV(src.size(), CV_8U), gray;

    pyrDown(src, pyr, Size(src.cols / 2, src.rows / 2));
    pyrUp(pyr, timg, src.size());
    vector<vector<Point> > contours;

    int counts = 0, counts_2 = 0, counts_3 = 0;
    vector<Point> cand_1, cand_2, cand_3;
    vector<vector<Point> > rect_2;
    vector<vector<Point> > rect_3;
    vector<vector<Point> > out;

    //cout<<"find squares src.chan="<<src.channels()<<endl;
    cvtColor(src, gray0, CV_BGR2GRAY);

    Canny(gray0, gray, 50, 200, 5);

    //imshow("Canny", gray);

    //dilate(gray, gray, Mat(), Point(-1,-1));
    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    vector<Point> approx;

    for (size_t i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() < 200 || contours[i].size() > 1000)
        {
            continue;
        }
        //cout<<"rectangle detected 1111"<<endl;

        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, true);
        //cout<<contours[i].size() <<"  "<<approx.size()<<" "<<fabs(contourArea(Mat(approx)))<<endl;
       

        if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 2000 && isContourConvex(Mat(approx)))
        {
            //cout<<contours[i].size() <<"  "<<approx.size()<<" "<< fabs(contourArea(Mat(approx))) << " " <<  isContourConvex(Mat(approx)) <<endl;
            //cout<<"rectangle detected  2222"<<endl;
            double maxCosine = 0;
            for (int j = 2; j < 5; j++)
            {
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            if (maxCosine < 0.3)
            {
                //cout<<"squares detected!"<<endl;
                squares.push_back(approx);
               
            }
        }
    }
}

void ApriltagDetector::drawSquares( Mat image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();

        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
    }
    //imshow("Square Detection Demo", image);
}

void ApriltagDetector::setTagCodes ( string s )
{
  if ( s=="16h5" )
    {
      m_tagCodes = AprilTags::tagCodes16h5;
    }
  else if ( s=="25h7" )
    {
      m_tagCodes = AprilTags::tagCodes25h7;
    }
  else if ( s=="25h9" )
    {
      m_tagCodes = AprilTags::tagCodes25h9;
    }
  else if ( s=="36h9" )
    {
      m_tagCodes = AprilTags::tagCodes36h9;
    }
  else if ( s=="36h11" )
    {
      m_tagCodes = AprilTags::tagCodes36h11;
    }
  else
    {
      cout << "Invalid tag family specified" << endl;
      exit ( 1 );
    }
  if ( NULL == m_tagDetector )
    m_tagDetector = new AprilTags::TagDetector ( m_tagCodes );
  else
    {
      delete m_tagDetector;
      m_tagDetector = new AprilTags::TagDetector ( m_tagCodes );
    }
}

void ApriltagDetector::reboot()
{
  m_frames = 0;
  m_win.clear();
  m_isTracking = false;
}

void ApriltagDetector::processImage ( cv::Mat& image )
{
  ++m_frames;
  Mat image_gray;
  if ( image.dims!=2 )
    cv::cvtColor ( image, image_gray, CV_BGR2GRAY );
  else
    image_gray = image.clone();

  double t0=0;
  if ( m_timing )
    {
      t0 = tic(); 
    }

  // no prev window, do detection on the whole image; if searching for apriltags, detect the whole image
  if ( m_win.size() !=4||m_mode==0 )//||m_mission_type==true)
    {
      //ROS_INFO("No prev window");
      detections = m_tagDetector->extractTags ( image_gray );
    }
  // prev window exists, only process in it
  else
    {
      cv::Mat imgWin = image_gray ( cv::Range ( m_win[2],m_win[3] ),cv::Range ( m_win[0],m_win[1] ) ).clone();
      detections = m_tagDetector->extractTags ( imgWin );

      // reproject the Tag result to the whole image
      for ( int i=0; i<detections.size(); i++ )
        {
          for ( int ii=0; ii<4; ii++ )
            {
              detections[i].p[ii].first+=m_win[0];
              detections[i].p[ii].second+=m_win[2];
            }
          detections[i].cxy.first+=m_win[0];
          detections[i].cxy.second+=m_win[2];
        }
    }

  if ( detections.empty() )
    m_win.clear();
  else
    {
      m_win = point2win ( image_gray, 1 );
//        cout<<m_win[0]<<" "<<m_win[1]<<" "<<m_win[2]<<" "<<m_win[3]<<endl;
    }

  if ( m_timing )
    {
      double dt = tic()-t0;

//     ROS_INFO ( "Extracting tags took%.6f seconds",dt );
    }

  print_detections();
  // show the current image including any detections
  if ( m_draw )
    {
      for ( unsigned int i=0; i<detections.size(); i++ )
        {
          detections[i].draw ( image );
        }
      //imshow ( "AprilTag", image );
      waitKey ( 1 );

    }
}

void ApriltagDetector::print_detections ( )
{
//   if ( m_isShowResult )
//     cout << "  Id: " << detection.id
//          << " (Hamming: " << detection.hammingDistance << ")";

  // recovering the relative pose of a tag:

  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)



  dji_sdk::Reldist rel_dist;
  rel_dist.header.frame_id = "x3_reldist";

  ROS_INFO ( "%d tags detected.",detections.size() );
  //ROS_INFO ( "%f tag family.", m_tagCodes);
// ROS_INFO ( "Publish detection routine is working..." );

// float last_flight_height = 0.0;
  //m_numOfDetections.data = detections.size();
  //m_numOfDetection_pub.publish ( m_numOfDetections );
  if ( detections.empty() ) // no Tag found
    {
      rel_dist.header.stamp = ros::Time::now();
      rel_dist.x = 1.3;  //for safe distance, 1 meters
      rel_dist.y = 0;
      rel_dist.z = 0;
      rel_dist.yaw = 0;
      rel_dist.pitch = 0;
      rel_dist.roll = 0;
   
      rel_dist.norm = 0.0;
      rel_dist.gimbal_pitch_inc = 0.0;
      rel_dist.istracked = false;

      m_detectionPoints.x0 = m_detectionPoints.y0 =
                               m_detectionPoints.x1 = m_detectionPoints.y1 =
                                     m_detectionPoints.x2 = m_detectionPoints.y2 =
                                           m_detectionPoints.x3 = m_detectionPoints.y3 = 0;
      m_detectionPoints.id = -1;//For no tag;
      m_detectionPoints_pub.publish ( m_detectionPoints );
      // ros::Rate dist_pub_rate(20);
      m_result_pub.publish ( rel_dist );
    }

  for ( unsigned int i=0; i<detections.size(); i++ )
    {
      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      detections[i].getRelativeTranslationRotation ( m_tagSize, m_fx, m_fy, m_px, m_py,
          translation, rotation );
      Eigen::Matrix3d F;
      F <<
        1, 0,  0,
           0,  -1,  0,
           0,  0,  1;
      Eigen::Matrix3d fixed_rot = F*rotation;
      double yaw, pitch, roll;
      wRo_to_euler ( fixed_rot, yaw, pitch, roll );


      if ( m_isShowResult )
        {

          ROS_INFO ( "Tag ID: %d",detections[i].id );
          ROS_INFO ( "distance:%.3f, m,x:%.3f, y:.%3f, z:%.3f, yaw:%.3f, pitch:%.3f, roll:%.3f",
                     translation.norm(),translation ( 0 ),translation ( 1 ),translation ( 2 ),yaw,pitch,roll );
          // Also note that for SLAM/multi-view application it is better to
          // use reprojection error of corner points, because the noise in
          // this relative pose is very non-Gaussian; see iSAM source code
          // for suitable factors.
        }



      rel_dist.header.stamp = ros::Time::now();
      rel_dist.x = translation ( 0 );
      rel_dist.y = translation ( 1 );

      rel_dist.yaw = yaw;
      rel_dist.pitch = pitch;
      rel_dist.roll = roll;
      rel_dist.norm = translation.norm();
      rel_dist.gimbal_pitch_inc = atan ( translation ( 2 ) /translation ( 0 ) ) *57.2958;
      rel_dist.istracked = true;

#ifndef GIMBAL_USED
      rel_dist.height_with_gimbal = translation ( 0 );
      rel_dist.z = translation ( 2 );//IMPORTANT NOTE: Uisng GIMBAL makes it a little bit different
#else //NOTE: The following only makes sense when GIMBAL CONTROLLING is USED.
      if ( m_gimbal.pitch+90.0<0.1 )
        {
          rel_dist.z = translation ( 2 );

          rel_dist.height_with_gimbal = translation ( 0 );
          // flight_height  = rel_dist.x;
          //ROS_INFO ( "Gimbal is straight down <0.1" );
        }
      else
        {

          //ROS_INFO ( "Gimbal >0.1" );
          float temp = sqrt ( pow ( translation ( 0 ),2 ) +pow ( translation ( 2 ),2 ) );
          rel_dist.z = temp *sin ( 0.017453* ( rel_dist.gimbal_pitch_inc+m_gimbal.pitch+90.0 ) ); // /57.2958 ); //IMPORTANT NOTE: Uisng GIMBAL makes it a little bit different
          rel_dist.height_with_gimbal = temp*cos ( ( rel_dist.gimbal_pitch_inc+m_gimbal.pitch+90.0 ) /57.2958 );

          //  flight_height = translation(0)*cos((rel_dist.gimbal_pitch_inc+m_gimbal.pitch+90)/57.2958);
        }
#endif

      // ros::Rate dist_pub_rate(20);


      //However, we just publish the detection result of the first detected tag;
      if(m_CMD_from_remote != 'd')
      {

          m_detectionPoints.x0 = detections[0].p[0].first;
          m_detectionPoints.y0 = detections[0].p[0].second;
          m_detectionPoints.x1 = detections[0].p[1].first;
          m_detectionPoints.y1 = detections[0].p[1].second;
          m_detectionPoints.x2 = detections[0].p[2].first;
          m_detectionPoints.y2 = detections[0].p[2].second;

          m_detectionPoints.x3 = detections[0].p[3].first;
          m_detectionPoints.y3 = detections[0].p[3].second;
          m_detectionPoints.id = detections[0].id;
          m_detectionPoints_pub.publish ( m_detectionPoints );

      }


      //  if(()


//       if ( this->usingSmallTags )
//         {
//           rel_dist.z -= 0.60 ;
//         }

#ifdef SMALL_TAG_USED
      if ( change_once_flag && translation ( 0 ) < 1.35 && m_CMD_from_remote == 'd' ) //(last_flight_height-flight_height)>0.01 )//If is descending
        {
          setTagCodes ( "16h5" );
          // m_tagCodes = AprilTags::tagCodes25h9;
        //  ROS_INFO ( "Tag is 16h5" );
          m_tagSize  = 0.057;
          //  m_CMD_from_remote = 'W';
          this->usingSmallTags = true;
          change_once_flag = false;
        }
#endif
     rel_dist.gimbal_yaw_inc = atan(-rel_dist.y/(EPS+rel_dist.z))* 57.2958;

      m_result_pub.publish ( rel_dist );


    }


  if ( this->usingSmallTags )
    {
      std_msgs::Bool using_smallTags;
      using_smallTags.data = true;
      m_using_smallTags_pub.publish ( using_smallTags );
    }


}


std::vector<int> ApriltagDetector::point2win ( cv::Mat image, float delta )
{
  m_win.clear();
  if ( detections.empty() )
    {
      return m_win;
    }

  int x_min = detections[0].p[0].first,
      x_max = ceil ( detections[0].p[0].first ) +1,
      y_min = detections[0].p[0].second,
      y_max = ceil ( detections[0].p[0].second ) +1;

  int x,y;
  for ( int i=0; i<detections.size(); i++ )
    {
      for ( int ii=0; ii<4; ii++ )
        {
          x = detections[i].p[ii].first;
          y = detections[i].p[ii].second;

          x_min = ( x < x_min ) ? x : x_min;
          x_max = ( ceil ( x ) +1 > x_max ) ? ceil ( x ) +1 : x_max;
          y_min = ( y < y_min ) ? y : y_min;
          y_max = ( ceil ( y ) +1 > y_max ) ? ceil ( y ) +1 : y_max;
        }
    }

  // zoom to 1+delta and check out_of_image
  int dX = x_max-x_min;
  int dY = y_max-y_min;

  x_min = ( x_min-dX*delta > 0 ) ? x_min-dX*delta : 0;
  y_min = ( y_min-dY*delta > 0 ) ? y_min-dY*delta : 0;
  x_max = ( x_max+dX*delta < image.cols ) ? x_max+dX*delta : image.cols;
  y_max = ( y_max+dY*delta < image.rows ) ? y_max+dY*delta : image.rows;

  m_win.push_back ( x_min );
  m_win.push_back ( x_max );
  m_win.push_back ( y_min );
  m_win.push_back ( y_max );
  return m_win;
}

void ApriltagDetector::Line_detection(cv::Mat& image, dji_sdk::Reldist & result)
{
  //Mat image_gray;
  //medianBlur(image, image, 5);
  int flagSituation=0;
  if ( image.channels()==3 )
  { 
    ROS_INFO("color image");
    //cv::cvtColor ( image, image_gray, CV_BGR2GRAY );
  }
  else
  {
    ROS_INFO("gray image");
    return ;
  }
  
  
  double error_y=0, yaww=0;
  calculate(image,error_y,yaww,flagSituation);
  
  //output the result
  result.header.frame_id = "x3_reldist";
  result.header.stamp = ros::Time::now();
  //if camera faces down, x is the vertical distance, y is horizontal y, z is horizontal x
  result.x = 0;
  if(error_y>70) error_y=70;
  if(error_y<-70) error_y=-70;
  result.y = error_y*0.03/10;   //radius=10 pixles=3cm
  result.yaw =  yaww-90;
  
  if(abs(result.yaw)<20) result.z=0.4;
  else if(abs(result.yaw)<50) result.z=0.2;
  else result.z=0.1;
  fcout<<"yaw="<<result.yaw<<endl;
  result.pitch = 0;
  result.roll = 0;
  result.norm = 0;
  result.gimbal_pitch_inc = 0;
  result.istracked = true;
  if(flagSituation==3)  //endpoint
    result.z = 0.0;   //the forward speed
  if(flagSituation==100)
    result.z = 0.1;
  m_result_pub.publish ( result );
}

void ApriltagDetector::calculate(cv::Mat &img, double & intercept, double & slope, int &flagSituation)
{
	//圆检测
		pair<vector<vector<double>>, vector<Vec3f>> results = circleDetection(img);
		vector<vector<double>> disMat = results.first;
		vector<Vec3f> circles = results.second;
		if (circles.size() < 2)
		{
			fcout << "detected circles number: " << circles.size() << endl;
			fcout << "can't detect circle" << endl;
			intercept=0;
			slope=90;
			flagSituation=100;
			return ;
		}

		//图节点数量
		int V = circles.size();

		//构造图
		SparseGraph<double> g = SparseGraph<double>(V, false);
		//ReadGraph<SparseGraph<double>, double> readGraph(g, filename);

		for (int i = 0; i < V; i++)
		{
			for (int j = 0; j < V; j++)
			{
			  g.addEdge(i, j, disMat[i][j]);
			}
		}

		// Test Lazy Prim MST
		//求解最小生成树

		LazyPrimMST<SparseGraph<double>, double> lazyPrimMST(g);

		//MST结果
		vector<Edge<double>> mst = lazyPrimMST.mstEdges();

		// [1] 十字路口
		vector<vector<int>> crossroadMatrix(V);

		//找到每个节点相连的节点
		for (int i = 0; i < V - 1; i++)
		{
		  crossroadMatrix[mst[i].v()].push_back(mst[i].w());  // mst is the edges vector 
		  crossroadMatrix[mst[i].w()].push_back(mst[i].v());
		}
		//十字路口节点距离矩阵
		vector<vector<double>> distanceCrossroad(6, vector<double>(3));
		int indexMaxBelow = 0, indexMaxUp = 0;
		int CrossTpLine=img.cols, CrossBwLine=0;
		for (int i = 0; i < V; i++)
		{
			// cout << i << " " << crossroadMatrix[i].size() << endl;
			if (crossroadMatrix[i].size() == 4)
			{
			    
			    flagSituation = 1;
			    int k = 0;
			    double maxBelow = 0, maxUp = 0;

			    for (int m = 0; m < 3; m++)
			    {
				    for (int n = m + 1; n < 4; n++)
				    {
					distanceCrossroad[k][0] = crossroadMatrix[i][m];
					if(circles[crossroadMatrix[i][m]][1]<=CrossTpLine) CrossTpLine=circles[crossroadMatrix[i][m]][1];
					if(circles[crossroadMatrix[i][m]][1]>CrossBwLine) CrossBwLine=circles[crossroadMatrix[i][m]][1];
					distanceCrossroad[k][1] = crossroadMatrix[i][n];
					if(circles[crossroadMatrix[i][n]][1]<=CrossTpLine) CrossTpLine=circles[crossroadMatrix[i][n]][1];
					if(circles[crossroadMatrix[i][n]][1]>CrossBwLine) CrossBwLine=circles[crossroadMatrix[i][n]][1];
	
					distanceCrossroad[k][2] = circleDistance(circles[crossroadMatrix[i][m]], circles[crossroadMatrix[i][n]]);
					if (distanceCrossroad[k][2] > maxUp)
					{
					  maxUp = distanceCrossroad[k][2];
					  indexMaxUp = k;
					}
					else if (distanceCrossroad[k][2] > maxBelow)
					{
					  maxBelow = distanceCrossroad[k][2];
					  indexMaxBelow = k;
					}
					k++;
				    }
			    }
				//cout << distanceCrossroad[indexMaxUp][0] << " " <<distanceCrossroad[indexMaxUp][1] << " " << distanceCrossroad[indexMaxBelow][0] << " " << distanceCrossroad[indexMaxBelow][1] << endl;
			}
		}
		//十字路口进入
		if (flagSituation == 1&&(MAX_LINE+MIN_LINE)/2>CrossTpLine&&(MAX_LINE+MIN_LINE)/2<CrossBwLine)
		{
			double crossRoad_Line1_Slope = slopeAndIntercept(circles, distanceCrossroad[indexMaxUp][0], distanceCrossroad[indexMaxUp][1], MAX_LINE, MIN_LINE).first;
			double crossRoad_Line2_Slope = slopeAndIntercept(circles, distanceCrossroad[indexMaxBelow][0], distanceCrossroad[indexMaxBelow][1], MAX_LINE, MIN_LINE).first;
			int crossRoadPoint1 = 0, crossRoadPoint2 = 0;
			if (crossRoad_Line1_Slope > 75 && crossRoad_Line1_Slope < 105)
			{
				crossRoadPoint1 = distanceCrossroad[indexMaxUp][0];
				crossRoadPoint2 = distanceCrossroad[indexMaxUp][1];
			}
			else
			{
				crossRoadPoint1 = distanceCrossroad[indexMaxBelow][0];
				crossRoadPoint2 = distanceCrossroad[indexMaxBelow][1];
			}
			pair<double, double> slopeAndInterceptResult = slopeAndIntercept(circles, crossRoadPoint1, crossRoadPoint2, (circles[crossRoadPoint1][1] + circles[crossRoadPoint2][1]) / 2, (circles[crossRoadPoint1][1] + circles[crossRoadPoint2][1]) / 2);
			slope = slopeAndInterceptResult.first;
			intercept = slopeAndInterceptResult.second;
			//cout << "十字路口: " << crossRoadPoint1 << " ------crossRoad------ " << crossRoadPoint2 << endl;
			fcout<< "crossroad detected"<<endl;
			fcout << "intercept: " << intercept << endl;
			fcout << "slope: " << slope << endl;
		}
		else  //control line is not in the cross road
		{
		//[2] 点--线--点
		vector<vector<double>> thetaMatrix;
		vector<double> theta(3);
		thetaMatrix.clear();
		for (int i = 0; i < mst.size(); i++)
		{
			// cout << "start: " << mst[i].v() << " end: " << mst[i].w() << " weight: " << mst[i].wt() << " theta : " << theta[2] << endl;
			if ((circles[mst[i].v()][1] < MIN_LINE && circles[mst[i].w()][1] > MAX_LINE) || (circles[mst[i].w()][1] < MIN_LINE && circles[mst[i].v()][1] > MAX_LINE))
			{
				theta[0] = mst[i].v();
				theta[1] = mst[i].w();
				pair<double, double> slopeAndInterceptResult = slopeAndIntercept(circles, mst[i].v(), mst[i].w(), MAX_LINE, MIN_LINE);
				theta[2] = slopeAndInterceptResult.first;
				slope = slopeAndInterceptResult.first;
				intercept = slopeAndInterceptResult.second;
				thetaMatrix.push_back(theta);

				//cout << "普通: " << mst[i].v() << "---------------" << mst[i].w() << endl;
				fcout << "nomal two point"<<endl;
				fcout << "intercept: " << intercept << endl;
				fcout << "slope: " << slope << endl;
			}
		}
		//[3-4] 起点和终点
		double thetaSinglePoint = 0;
		double distanceSinglePoint = 0;
		pair<int, int> result_Max_Min = Y_Point_Max_Min(circles);
		//Up(起点)
		if ((MAX_LINE + MIN_LINE) / 2.0 > circles[result_Max_Min.first][1])
		{
			flagSituation=2;  //the start point
			slope = slopeAndIntercept(circles, result_Max_Min.first, crossroadMatrix[result_Max_Min.first][0], MAX_LINE, MIN_LINE).first;
			intercept = 320 - circles[result_Max_Min.first][0];
			//cout << "起点: " << result_Max_Min.first << "--------------- " << endl;
			fcout<< "start point"<<endl;
			fcout << "intercept: " << intercept << endl;
			fcout << "slope: " << slope << endl;
		}
		//Down(终点)
		if ((MAX_LINE + MIN_LINE) / 2.0 < circles[result_Max_Min.second][1])
		{
			flagSituation=3;  //endpoint
			slope = slopeAndIntercept(circles, result_Max_Min.second, crossroadMatrix[result_Max_Min.second][0], MAX_LINE, MIN_LINE).first;
			intercept = 320 - circles[result_Max_Min.second][0];
			//cout << "终点: " << "--------------- " << result_Max_Min.second << endl;
			fcout << "end point"<<endl;
			fcout << "intercept: " << intercept << endl;
			fcout << "slope: " << slope << endl;
		}
		}
#ifdef SHOW_DETECTION_RESULT
		for (int i = 0; i < circles.size() - 1; i++)
		{
			Vec3i pointBegin = circles[mst[i].v()];
			Vec3i pointEnd = circles[mst[i].w()];
			line(img, Point(pointBegin[0], pointBegin[1]), Point(pointEnd[0], pointEnd[1]), Scalar(0, 0, 255), 1, CV_AA);
		}
		imshow("Paint Line Result", img);
		waitKey(1);
#endif
}

Mat cameraMatrix = (Mat_<double>(3, 3) << 381.1694, 0, 327.1767, 0, 376.6416, 182.5309, 0, 0, 1);
Mat distCoeffs = (Mat_<double>(1, 4) << -0.1414, 0.1355, 0, 0);
//Point3f world_pnt_tl(-450,-450,0), world_pnt_tr(-450,450,0), world_pnt_br(450,450,0), world_pnt_bl(450,-450,0);
Point3f world_pnt_tl(-275,-275,0), world_pnt_tr(-275,275,0), world_pnt_br(275,275,0), world_pnt_bl(275,-275,0);
//Point3f world_pnt_tl(-83,-83,0), world_pnt_tr(-83,83,0), world_pnt_br(83,83,0), world_pnt_bl(83,-83,0);
cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
double thetaz, thetay, thetax;
Point Senter;
void ApriltagDetector::Calcu_attitude(Point2f pnt_tl_src, Point2f pnt_tr_src, Point2f pnt_br_src, Point2f pnt_bl_src)
{
	vector<Point3f> Points3D;
	vector<Point2f> Points2D;
	
	Points3D.push_back(world_pnt_tl);
	Points3D.push_back(world_pnt_tr);
	Points3D.push_back(world_pnt_br);
	Points3D.push_back(world_pnt_bl);
	Points2D.push_back(pnt_tl_src);
	Points2D.push_back(pnt_tr_src);
	Points2D.push_back(pnt_br_src);
	Points2D.push_back(pnt_bl_src);

	solvePnP(Points3D, Points2D, cameraMatrix, distCoeffs, rvec, tvec);
	
	double rm[9];
	cv::Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);

	double r11 = rotM.ptr<double>(0)[0];
	double r12 = rotM.ptr<double>(0)[1];
	double r13 = rotM.ptr<double>(0)[2];
	double r21 = rotM.ptr<double>(1)[0];
	double r22 = rotM.ptr<double>(1)[1];
	double r23 = rotM.ptr<double>(1)[2];
	double r31 = rotM.ptr<double>(2)[0];
	double r32 = rotM.ptr<double>(2)[1];
	double r33 = rotM.ptr<double>(2)[2];

	thetaz = atan2(r21, r11) / CV_PI * 180;
	thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	thetax = atan2(r32, r33) / CV_PI * 180;
}
/*********************************************************************************************************************************/	
//input: color images

int ApriltagDetector::Num_detection(cv::Mat &img,cv::Mat mimg,bool flag, dji_sdk::Reldist & pos_result)
{	
	std_msgs::Bool detection_flag;

	Mat ppImg; 
        
  ppImg = img;

  int squaresize =  0;
   
	if(!flag) //pad
	{
	    vector<vector<Point> > squares;
	    findSquares(img,squares);

      //img.copyTo(img_cp);
      //drawSquares(img_cp,squares);

			squaresize = squares.size();
      writeF <<"squaresize = "<<squaresize<< endl;

			detection_flag.data=false;
	    if(squares.size() > 0)
	    {
	      vector<Point> approx = squares[0];
	      int center_x=0,center_y=0;
	      for(int n=0;n<4;n++)
	      {
		     center_x+=approx[n].x;
		     center_y+=approx[n].y;
	      }
	      center_x=center_x/4;
	      center_y=center_y/4;

	      pos_result.x = -(center_y-205)*0.04/5;  //5 pixles = 4cm  
	      pos_result.y = (center_x-320)*0.04/5;  //5 pixles = 4cm  (320,180)
	      pos_result.z = 0;
	      pos_result.yaw = 0;
	      detection_flag.data=true;
	  	}

			if(!detection_flag.data)
			{
				pos_result.x = 0;  //5 pixles = 4cm
			  pos_result.y = 0;  //5 pixles = 4cm
			  pos_result.z = 0;
			  pos_result.yaw = 0;
			}
/*
			else
			{
				detection_flag.data=false;
			  pos_result.x = 0;  //5 pixles = 4cm
			  pos_result.y = 0;  //5 pixles = 4cm
			  pos_result.z = 0;
			  pos_result.yaw = 0;
			 
			}
*/
	}
	else
	{
//	  writeF <<"detection_flag ="<< detection_flag.data<< endl;
	   writeF <<"detection_flag successfully1"<< endl;
	  detection_flag.data=false;

	  vector<Mat> channels;
	  Mat  BlueChannel, GreenChannel, RedChannel;
	  Mat canny_mat;
	  vector<vector<Point>> contours_four;
	  
	  split(img, channels);
	  BlueChannel = channels.at(0);
	  GreenChannel = channels.at(1);
	  RedChannel = channels.at(2);
	  for (int i = 0; i < img.cols; i++)
	  {
		  for (int j = 0; j < img.rows; j++)
		  {
			  double Th1 = (double)RedChannel.at<uchar>(j, i) / ((double)BlueChannel.at<uchar>(j, i) + 0.1);
			  double Th2 = (double)GreenChannel.at<uchar>(j, i) / ((double)BlueChannel.at<uchar>(j, i) + 0.1);
			  double Th3 = (double)RedChannel.at<uchar>(j, i) - GreenChannel.at<uchar>(j, i);
			  double Th4 = (double)RedChannel.at<uchar>(j, i);
			  double Th5 = (double)GreenChannel.at<uchar>(j, i);
			  double Th6 = (double)BlueChannel.at<uchar>(j, i);


  //		  if (Th1 > 3.5 && Th2 > 3.5  && Th4 > 100 && Th5 > 90&&abs(Th3)<50)   //afternoon, 4, 4, 120, 120
		    //if (Th1 > 1.7 && Th2 > 1.7  && Th4 > 60 && Th5 > 60&&abs(Th3)<50)    //2018-07-20-11:36  on the sunshine
  //		   if (Th1 > 1.4 && Th2 > 1.4 && Th4 > 50 && Th5 >60&&abs(Th3)<50)       // 2018-07-31-18:55  beside the building
		  //if (Th1 > 1.58 && Th2 > 1.58 && Th4 > 200 && Th5 >100&&abs(Th3)<50)	// 2018-08-19-16:55  bright
		  //if (Th1 > 1.3 && Th2 > 1.3 && Th4 > 50 && Th5 >60&&abs(Th3)<50)	// 2018-08-08-17:14  darker
		  //	  if (Th1 > 1.3 && Th2 > 1.3  && Th4 > 150 && Th5 > 50&&abs(Th3)<50)

	//	      if (Th1 > 3.5 && Th2 > 2.0 && Th4 > 100 && Th5 > 90&&abs(Th3)<100)  //circle	
	//	   if (Th1 > 3.5 && Th2 > 2.0 && Th4 > 100 && Th5 > 25&&abs(Th3)<100)  //circle	
  			if (Th1 > 2.0 && Th2 > 1.8 && Th4 > 100 && Th5 > 25&&abs(Th3)<100)  //circle	
		      {  
			      RedChannel.at<uchar>(j, i) = 255;
			      GreenChannel.at<uchar>(j, i) = 255;
			      BlueChannel.at<uchar>(j, i) = 255;
		      }
		      else
		      {
			      RedChannel.at<uchar>(j, i) = 0;
			      GreenChannel.at<uchar>(j, i) = 0;
			      BlueChannel.at<uchar>(j, i) = 0;					
		      }
			  
					  
		  }
	  }
	  

	  merge(channels, img);
	  cvtColor(img, img, COLOR_BGR2GRAY);
	  cvtColor(mimg, mimg, COLOR_BGR2GRAY);
	  medianBlur(img, img, 3);  /////   
	  threshold(img, img, 150, 255, 0);
	  threshold(mimg, mimg, 150, 255, 0);
	  
//	  Canny(img, canny_mat, 50, 200, 5);

//	  imshow("Canny", canny_mat);
	  Mat ele = getStructuringElement(MORPH_RECT,Size(7,7));
	  erode(img,img,ele,Point(),1);
	  
	  //dilate(gray, gray, Mat(), Point(-1,-1));
	  findContours(img, contours_four, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	  vector<Point> approx;
	  
	  
	  bool FLAG = false;

	  for (size_t i = 0; i < contours_four.size(); i++)
	  {
	      if (contours_four[i].size() < 20 || contours_four[i].size() > 1000)
	      {
		  continue;
	      }
	      //cout<<"rectangle detected 1111"<<endl;

	      approxPolyDP(Mat(contours_four[i]), approx, arcLength(Mat(contours_four[i]), true) * 0.04, true);
	      cout<<contours_four[i].size() <<"  "<<approx.size()<<" "<<fabs(contourArea(Mat(approx)))<<endl;
	   
	      if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 2000 && isContourConvex(Mat(approx)))
	      {
					//cout<<contours[i].size() <<"  "<<approx.size()<<" "<< fabs(contourArea(Mat(approx))) << " " <<  isContourConvex(Mat(approx)) <<endl;
					//cout<<"rectangle detected  2222"<<endl;
					double maxCosine = 0;
					for (int j = 2; j < 5; j++)
					{
							double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
							maxCosine = MAX(maxCosine, cosine);
					}
					if (maxCosine < 0.2)
					{
					//   detection_flag.data=true;
							FLAG = true;

							cout<<"squares detected!"<<endl;
							writeF <<"detection_flag successfully"<< endl;
							const Point* p = &approx[0];
							int n = (int)approx.size();

							//polylines(img, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
					//    imshow("Square Detection Demo", img);
							vector<Point> pts=approx;
							Point2f ptfour[4];
							int addmax=0,addmin=2000,submin=2000,submax=-2000;
							for(int i=0;i<pts.size();i++)
							{
								if(pts[i].x+pts[i].y<addmin)
								{
								addmin=pts[i].x+pts[i].y;
								ptfour[0]=pts[i];
								}
								if(pts[i].x+pts[i].y>addmax)
								{
								addmax=pts[i].x+pts[i].y;
								ptfour[2]=pts[i];
								}
								if(pts[i].x-pts[i].y<submin)
								{
								submin=pts[i].x-pts[i].y;
								ptfour[3]=pts[i];
								}
								if(pts[i].x-pts[i].y>submax)
								{
								submax=pts[i].x-pts[i].y;
								ptfour[1]=pts[i];
								}
							}
							/*
							printf("%f,%f\n",ptfour[0].x,ptfour[0].y);
							printf("%f,%f\n",ptfour[1].x,ptfour[1].y);
							printf("%f,%f\n",ptfour[2].x,ptfour[2].y);
							printf("%f,%f\n",ptfour[3].x,ptfour[3].y);
							*/
						Calcu_attitude(ptfour[0],ptfour[1],ptfour[2],ptfour[3]);
							//	cout << tvec << endl;
						//ROS_INFO("tx=%f, ty=%f, tz=%f, ", tvec.ptr<double>(0)[0],tvec.ptr<double>(1)[0],tvec.ptr<double>(2)[0]);
						//ROS_INFO("thetax=%f, thetay=%f, thetaz=%f ",thetax,thetay,thetaz);
					
						//ROS_INFO("yaw=%f ", pos_result.yaw)
						imwrite("home/ubuntu/GaofenChallenge/cap1/find.jpg",ppImg);
					}
		
	  		}
	  }
	  if(FLAG)
	  {
		  
		  detection_flag.data=true;
		  writeF<<"tx="<<tvec.ptr<double>(0)[0]<<endl;   // write in log
		  writeF<<"ty="<<tvec.ptr<double>(1)[0]<<endl;   // write in log
		  writeF<<"tz="<<tvec.ptr<double>(2)[0]<<endl;   // write in log
		  writeF<<"thetax="<<thetax<<endl;   // write in log
		  writeF<<"thetay="<<thetay<<endl;   // write in log
		  writeF<<"thetaz="<<thetaz<<endl;   // write in log

		  pos_result.x = tvec.ptr<double>(2)[0]/1000;  //m,close to tag, have not subscribe the safe distance 
		  pos_result.y = tvec.ptr<double>(0)[0]/1000;  //m
		  //pos_result.z = tvec.ptr<double>(1)[0]; 
		  pos_result.yaw = thetax-180;  //jiaodu 
			//filter away from drone
			if(pos_result.x > 3.0)
			{
				pos_result.x = 0;  //5 pixles = 4cm
				pos_result.y = 0;  //5 pixles = 4cm
				pos_result.z = 0;
				pos_result.yaw = 0;
				detection_flag.data=false;
			}
	  }
	  else
	  {
		  pos_result.x = 0;  //5 pixles = 4cm
		  pos_result.y = 0;  //5 pixles = 4cm
		  pos_result.z = 0;
		  pos_result.yaw = 0;
		  detection_flag.data=false;
	  }
/*
	  for (size_t i = 0; i < contours_four.size(); i++)
	  {
	      if (contours_four[i].size() < 20 || contours_four[i].size() > 1000)
	      {
		  continue;
	      }
	      //cout<<"rectangle detected 1111"<<endl;

	      approxPolyDP(Mat(contours_four[i]), approx, arcLength(Mat(contours_four[i]), true) * 0.04, true);
	      cout<<contours_four[i].size() <<"  "<<approx.size()<<" "<<fabs(contourArea(Mat(approx)))<<endl;
	    

	      if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 2000 && isContourConvex(Mat(approx)))
	      {
		  //cout<<contours[i].size() <<"  "<<approx.size()<<" "<< fabs(contourArea(Mat(approx))) << " " <<  isContourConvex(Mat(approx)) <<endl;
		  //cout<<"rectangle detected  2222"<<endl;
		  double maxCosine = 0;
		  for (int j = 2; j < 5; j++)
		  {
		      double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
		      maxCosine = MAX(maxCosine, cosine);
		  }
		  if (maxCosine < 0.3)
		  {
		      detection_flag.data=true;
		      cout<<"squares detected!"<<endl;
		       writeF <<"detection_flag successfully"<< endl;
		      const Point* p = &approx[0];
		      int n = (int)approx.size();

		      polylines(img, &p, &n, 1, true, Scalar(0,255,0), 1, CV_AA);
		  //    imshow("Square Detection Demo", img);
		      vector<Point> pts;
		      Point2f ptfour[4];
		      int addmax=0,addmin=2000,submin=2000,submax=-2000;
		      for(int i=0;i<pts.size();i++)
		      {
			      if(pts[i].x+pts[i].y<addmin)
			      {
			      addmin=pts[i].x+pts[i].y;
			      ptfour[0]=pts[i];
			      }
			      if(pts[i].x+pts[i].y>addmax)
			      {
			      addmax=pts[i].x+pts[i].y;
			      ptfour[2]=pts[i];
			      }
			      if(pts[i].x-pts[i].y<submin)
			      {
			      submin=pts[i].x-pts[i].y;
			      ptfour[3]=pts[i];
			      }
			      if(pts[i].x-pts[i].y>submax)
			      {
			      submax=pts[i].x-pts[i].y;
			      ptfour[1]=pts[i];
			      }
		      }
		      
		      printf("%f,%f\n",ptfour[0].x,ptfour[0].y);
		      printf("%f,%f\n",ptfour[1].x,ptfour[1].y);
		      printf("%f,%f\n",ptfour[2].x,ptfour[2].y);
		      printf("%f,%f\n",ptfour[3].x,ptfour[3].y);
		      
			  Calcu_attitude(ptfour[0],ptfour[1],ptfour[2],ptfour[3]);
		      //	cout << tvec << endl;
			  //ROS_INFO("tx=%f, ty=%f, tz=%f, ", tvec.ptr<double>(0)[0],tvec.ptr<double>(1)[0],tvec.ptr<double>(2)[0]);
			  //ROS_INFO("thetax=%f, thetay=%f, thetaz=%f ",thetax,thetay,thetaz);
			  writeF<<"tx="<<tvec.ptr<double>(0)[0]<<endl;   // write in log
			  writeF<<"ty="<<tvec.ptr<double>(1)[0]<<endl;   // write in log
			  writeF<<"tz="<<tvec.ptr<double>(2)[0]<<endl;   // write in log
			  writeF<<"thetax="<<thetax<<endl;   // write in log
			  writeF<<"thetay="<<thetay<<endl;   // write in log
			  writeF<<"thetaz="<<thetaz<<endl;   // write in log

			  pos_result.x = tvec.ptr<double>(2)[0]/1000;  //m,close to tag, have not subscribe the safe distance 
			  pos_result.y = tvec.ptr<double>(0)[0]/1000;  //m
			  //pos_result.z = tvec.ptr<double>(1)[0]; 
			  pos_result.yaw = thetax-180;  //jiaodu 
			  //ROS_INFO("yaw=%f ", pos_result.yaw);

       
       
	      imwrite("home/ubuntu/GaofenChallenge/cap1/find.jpg",ppImg);


		  }
		  else
		  {
		    //flag = 0;
		      pos_result.x = 0;  //5 pixles = 4cm
		      pos_result.y = 0;  //5 pixles = 4cm
		      pos_result.z = 0;
		      pos_result.yaw = 0;
		      detection_flag.data=false;
		  }
	      }
	      else
	      {
					pos_result.x = 0;  //5 pixles = 4cm
					pos_result.y = 0;  //5 pixles = 4cm
					pos_result.z = 0;
					pos_result.yaw = 0;
					detection_flag.data=false;
	      }
	      
	  }*/
   }

    //printf("detection_flag= %d\n",detection_flag.data);
    m_result_pub.publish ( pos_result );
    tag_detections_pub.publish(detection_flag);  //2018-08-11
	
#ifdef _SHOW_PHOTO
	namedWindow("rect_result_out");
	imshow("rect_result_out", result_out);

	namedWindow("contours");
	imshow("contours", result_num);
	waitKey(1);
#endif
	return 0;
}


