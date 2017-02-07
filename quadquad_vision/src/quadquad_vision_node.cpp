#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 70
#define MIN_NUM_FEAT 100


void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status) { 

//this function automatically gets rid of points for which tracking fails

  vector<float> err;          
  Size winSize=Size(21,21);                                               
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
      if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))  {
          if((pt.x<0)||(pt.y<0))  {
            status.at(i) = 0;
          }
          if(points1.size() < 8)
          {
            return;
          }
          points1.erase (points1.begin() + (i - indexCorrection));
          points2.erase (points2.begin() + (i - indexCorrection));
          indexCorrection++;
      }

     }

}


void featureDetection(Mat img_1, vector<Point2f>& points1)  {   //uses FAST as of now, modify parameters as necessary
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 10;
  bool nonmaxSuppression = true;
  //goodFeaturesToTrack(img_1, keypoints_1, 200, .03, 5);
  FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}

int main( int argc, char** argv ) {
  //init ROS and this node
  ros::init(argc, argv, "egomotion_node");
  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<std_msgs::String>("pose", 1000);
  ros::Rate loop_rate(30);

  Mat img_1, img_2;

  double scale = 1.00;

  raspicam::RaspiCam_Cv Camera;
  Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 ); //set Opencv format
  //Open camera 
  cout<<"Opening Camera..."<<endl;
  if ( !Camera.open()) {cerr<<"Error opening camera"<<endl;return -1;}
  //wait a while until camera stabilizes
  cout<<"Sleeping for 3 secs"<<endl;
  ros::Duration(3.0).sleep();


  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;  
  cv::Point textOrg(10, 50);
  

  Camera.grab();
  Camera.retrieve (img_1);


  // feature detection, tracking
  vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
  featureDetection(img_1, points1);        //detect features in img_1
  vector<uchar> status;
  Camera.grab();
  Camera.retrieve(img_2);
  featureTracking(img_1,img_2,points1,points2, status); //track those features to img_2

  double focal = 657.30254;
  cv::Point2d pp(360., 240.);
  //recovering the pose and the essential matrix
  Mat R_f;
  Mat t_f; 
  Mat E, R, t, mask;

  
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R_f, t_f, focal, pp, mask);

  Mat prevImage = img_1;
  Mat currImage;
  vector<Point2f> prevFeatures = points1;
  vector<Point2f> currFeatures;


  //namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  //namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  while(ros::ok()) {
    // get new image
    Camera.grab();
    Camera.retrieve(currImage);

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    if (prevFeatures.size() < MIN_NUM_FEAT) {
      cout << "triggered redetection" << endl;
      featureDetection(prevImage, prevFeatures);
      // EMOTIONAL OUTPUT - apprehensive, slightly confused
      //featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
    }

    //calculate feature changes
    vector<uchar> status;
    featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

    //we don't have enough features to track
    if (currFeatures.size() <= 5)
    {
      // EMOTIONAL OUTPUT - confusion
      cout << "not enough features to track!" << endl;
      continue;
    }
    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
    Mat prevPts(2,prevFeatures.size(), CV_64F), currPts(2,currFeatures.size(), CV_64F);


    for(int i=0;i<prevFeatures.size();i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
      prevPts.at<double>(0,i) = prevFeatures.at(i).x;
      prevPts.at<double>(1,i) = prevFeatures.at(i).y;

      currPts.at<double>(0,i) = currFeatures.at(i).x;
      currPts.at<double>(1,i) = currFeatures.at(i).y;
    }


    t_f = t_f + 10.*(R_f*t);
    R_f = R*R_f;


    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 300;


    cout << "x: " << x << " y: " << y << endl;
    /*circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    imshow( "Trajectory", traj );

    waitKey(1); */
    ros::spinOnce();
    loop_rate.sleep();
  }

  Camera.release();
  cout << "closing camera" << endl;

  return 0;
}