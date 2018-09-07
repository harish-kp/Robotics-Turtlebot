#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <math.h>
#include "matcher.h"

#define PI 3.14159265

namespace enc = sensor_msgs::image_encodings;

bool depth = false;
cv_bridge::CvImagePtr cv_ptr2;
double f_len = 580; // Focal length of Kinect IR camera

static const char WINDOW[] = "Image window";
static const char WINDOW2[] = "Image window 2";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_sub2_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    //image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    image_sub2_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb2, this);

    cv::namedWindow(WINDOW);
    cv::namedWindow(WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (BeginDetect){
    
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
    try
    {
	cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Set up Matcher
    RobustMatcher rmatcher;
    rmatcher.setRatio(0.8f);
    rmatcher.setDist(min(2.5*image.cols*image.rows/2/314928,2.5));
    cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(600); 
    rmatcher.setFeatureDetector(pfd);
    
    cv::Mat image_train;
	
	int seeNo2 = 0; // initialises the count of seen objects
	
    for (int k=0; k<3; k++){
	    // Read test Image
	    if (k==0) {
	    	image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/Fuel.png");
	    	//image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/Fanta.png");
	    } 
	    else if (k==1) {
	    	image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/Fanta.png");
	    }
	    else {
	    	image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/Weet-bix.png");
	    }
	    std::vector<cv::DMatch> matches;
	    std::vector<cv::KeyPoint> keypoints1, keypoints2;
	    
	    cv::Mat homography= rmatcher.match(image_train, image, matches, keypoints1, keypoints2);
	    
	    //cv::drawKeypoints(image,keypoints2,image);
	    cv::Mat image_new;
	    try {
	    	char buffer [50];
	    	int n;
  		n=sprintf (buffer, "Matches %d", k);
		cv::drawMatches(image_train, keypoints1, image, keypoints2, matches, image_new);
		cv::namedWindow(buffer);
		cv::imshow(buffer, image_new);
	    }
	    catch (cv::Exception& e){
	    	// do nothing
	    }
	    
	    //std::cout << "No. of keypoints: " << keypoints1.size() << ", " << keypoints2.size() << std::endl;
	    std::cout << k << ") No. of Matches: " << matches.size() << std::endl;
	    
	    if (matches.size() >= 4) {
		    // Object Center
		    double d1[] = {
			image_train.cols/2, 
			image_train.rows/2, 
			1.0};
		    cv::Mat objCent0 = cv::Mat(3,1,CV_64FC1,d1);
		    cv::Mat objCent1 = homography*objCent0;
		    int centX, centY;
		    centX =  objCent1.at<double>(0,0)/objCent1.at<double>(2,0);
		    centY = objCent1.at<double>(1,0)/objCent1.at<double>(2,0);
		    std::cout << k << ") Center = " << centX << ", " << centY << std::endl;
		    if (depth){
		    	double min_range_ = 0.2;
    			double max_range_ = 2.0;
		    	cv::Mat image2(cv_ptr2->image.rows, cv_ptr2->image.cols, CV_8UC1);
			for(int i = 0; i < cv_ptr2->image.rows; i++)
			{
				float* Di = cv_ptr2->image.ptr<float>(i);
				char* Ii = image2.ptr<char>(i);
				for(int j = 0; j < cv_ptr2->image.cols; j++)
				{   
				    if (Di[j]<=max_range_){
				    	Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
				    }
				    else {
				    	Ii[j] = (char) 0;
				    }
				}   
			}
			if (((centX-5)>0)&((centX+5)<(image2.cols-1))&((centY-5)>0)&((centY+5)<(image2.rows-1))){
			    cv::Rect rect = cv::Rect(centX-5, centY-5, 10, 10);
			    cv::Mat cropimg = image2(rect);
			    cv::Mat disVal;
			    cv::reduce(cropimg,disVal,0,CV_REDUCE_MAX);
			    cv::reduce(disVal,disVal,1,CV_REDUCE_MAX);
			
			    double dist = disVal.at<int>(0,0)/255.0*(max_range_-min_range_)+min_range_;
			    if ((dist < max_range_)&(dist > 0)) {
				    cout << "Distance = " << dist << std::endl;
				    DetectDis[k] = dist;
				
				    double ang = atan((image2.cols/2-centX)/(f_len));
				    std::cout << "Angle = " << ang << std::endl;
				    DetectAng[k] = ang;
				
				    if ((k == 0) | (k==2))
					    detectNo = detectNo + 1;
			    }
            }
			cv::Mat image2_RGB;
			cv::cvtColor(image2, image2_RGB,CV_GRAY2RGB);
			cv::circle(image2_RGB,cv::Point(centX,centY),5,cv::Scalar(0,0,255));
			
			if ((k == 0) | (k==2))
					seeNo2 = seeNo2 + 1;
			
			cv::namedWindow("Distance Measure");
			cv::imshow("Distance Measure", image2_RGB);
		    	
		    	cv::circle(image,cv::Point(centX,centY),5,cv::Scalar(0,0,255));
		    }
	    }
	    else {
	    	    std:: cout << k << ") No detection! " << std::endl;
	    }
    }
    
    cv::imshow(WINDOW, image);
    //cv::drawKeypoints(image_train,keypoints1,image_train);
    //cv::namedWindow("train image");
    //cv::imshow("train image", image_train);
    /*
    cv::Mat image_HSV; 
    cv::Mat image_out;
    cv::cvtColor(image, image_HSV,CV_BGR2HSV);
    
    //cv::vector<cv::Mat> split_HSV;
    //std::cout << image_HSV << std::endl;
    
    cv::inRange(image_HSV, cv::Scalar(0,200,0), cv::Scalar(5, 255, 255), image_out);
    //cv::inRange(image_HSV, cv::Scalar(213,200,0), cv::Scalar(220, 255, 255), image_out);
    
    //cv::inRange(image, cv::Scalar(0,0,150), cv::Scalar(125, 65, 255), image_out);
    cv::namedWindow("Color Diff");
    cv::imshow("Color Diff",image_out);
    */
    if (detectNo == 2) {
    	Detected = true;
    	BeginDetect = false;
    	DetectMove = true;
    	std::cout << "Detected!" << std::endl;
    	//cv::waitKey(50); // wait for 50 ms
    }
    
    std::cout << "Detected Number = " << detectNo << std::endl;
	
	// set the Global seeNo variable to internal seeNo2 value
	seeNo = seeNo2; 
	std::cout << "Number of objects in view: " << seeNo << std::endl;
    
	depth = false;
    detectNo = 0;
    DetectAng[0]=-DetectAng[0];
    DetectAng[1]=-DetectAng[1];
    cv::waitKey(33);
    //cv::waitKey(0);
    
    //image_pub_.publish(cv_ptr->toImageMsg());
    }
  }
  
  void imageCb2(const sensor_msgs::ImageConstPtr& msg2)
  {
    //cv_bridge::CvImagePtr cv_ptr2;
    cv::Mat image2;
    double min_range_ = 0.2;
    double max_range_ = 0.8;
    try
    {
	cv_ptr2 = cv_bridge::toCvCopy(msg2, "32FC1");
	cv::Mat image2(cv_ptr2->image.rows, cv_ptr2->image.cols, CV_8UC1);
	for(int i = 0; i < cv_ptr2->image.rows; i++)
	{
		float* Di = cv_ptr2->image.ptr<float>(i);
		char* Ii = image2.ptr<char>(i);
		for(int j = 0; j < cv_ptr2->image.cols; j++)
		{   
		    if (Di[j]<=max_range_){
		    	Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
		    }
		    else {
		    	Ii[j] = (char) 0;
		    }
		}   
	}
	depth = true;
	if (gestureStart){
	// Set up Matcher
    	RobustMatcher rmatcher;
    	rmatcher.setRatio(0.8f);
    	rmatcher.setDist(0.8);
    	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(600); 
    	rmatcher.setFeatureDetector(pfd);
    	
    	// Detection
    	bool detectD[2];
    	
	for (int k = 0; k < 2; k++){
	    	// Read test Image
	    	cv::Mat image_train, image_train2, image_train3;
	    	if (k==0){
		    	image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth5.png");
		    	image_train2 = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth5_2.png");
		    	image_train3 = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth5_3.png");
	    	}
	    	else {
	    		image_train = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth2.png");
		    	image_train2 = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth2_2.png");
		    	image_train3 = cv::imread("/home/turtlebot/Group09/navibot/src/pictures/depth2_3.png");
	    	}
	    	std::vector<cv::DMatch> matches;
	    	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	    	
	    	cv::Mat homography= rmatcher.match(image_train2, image2, matches, keypoints1, keypoints2);
	    	homography= rmatcher.match(image_train3, image2, matches, keypoints1, keypoints2);

	    	homography= rmatcher.match(image_train, image2, matches, keypoints1, keypoints2);
	    
	    	//std::cout << k << ") No. of keypoints (depth): " << keypoints1.size() << ", " << keypoints2.size() << std::endl;
	    	std::cout << k << ") No. of Matches (depth): " << matches.size() << std::endl;
		/*
		cv::Mat image_new;
		try {
			cv::drawMatches(image_train2, keypoints1, image2, keypoints2, matches, image_new);
			cv::namedWindow("Matches (depth)");
			cv::imshow("Matches (depth)", image_new);
		}
		catch (cv::Exception& e){
			// do nothing
		}*/
		
		if (k==0) {
			if (matches.size() >= 15) {
				detectD[0] = true;
			}
			else {
				detectD[0] = false;
			}
		}
		else {
			if (matches.size() >= 14) {
				detectD[1] = true;
			}
			else {
				detectD[1] = false;
			}
		}
		
	}
	//cv::imshow(WINDOW2, image2);	
	//cv::waitKey(1);
	std::cout << "Detections: " << detectD[0] << ", " << detectD[1] << std::endl;
	
	if (detectD[0] != detectD[1]){
		gestureStart = false;
	}
	}
	cv::imshow(WINDOW2, image2);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
  }
};

