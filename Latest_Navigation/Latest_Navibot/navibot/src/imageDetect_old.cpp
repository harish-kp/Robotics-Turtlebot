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
#include "matcher.h"

namespace enc = sensor_msgs::image_encodings;

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
    image_pub_ = it_.advertise("out", 1);
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

    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Set up Matcher
    RobustMatcher rmatcher;
    rmatcher.setRatio(0.8f);
    rmatcher.setDist(min(2.5*image.cols*image.rows/2/314928,2.5));
    cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(600); 
    rmatcher.setFeatureDetector(pfd);
    
    // Read test Image
    cv::Mat image_train = cv::imread("/home/mits/Documents/ANU/ENGN4627/Projects/ROS_test/test_img/Fuel.png");
    
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    
    cv::Mat homography= rmatcher.match(image_train, image, matches, keypoints1, keypoints2);
    
    std::cout << "No. of keypoints: " << keypoints1.size() << ", " << keypoints2.size() << std::endl;
    std::cout << "No. of Matches: " << matches.size() << std::endl;
    //cv::drawKeypoints(image,keypoints2,image);
    cv::Mat image_new;
    try {
	cv::drawMatches(image_train, keypoints1, image, keypoints2, matches, image_new);
	cv::namedWindow("Matches");
	cv::imshow("Matches", image_new);
    }
    catch (cv::Exception& e){
    	// do nothing
    }
    cv::imshow(WINDOW, image);
    cv::drawKeypoints(image_train,keypoints1,image_train);
    cv::namedWindow("train image");
    cv::imshow("train image", image_train);
    
    cv::Mat image_HSV, image_out;
    cv::cvtColor(image, image_HSV,CV_BGR2HSV);
    
    std::cout << image_HSV << std::endl;
    
    //cv::inRange(image_HSV, cv::Scalar(0,200,0), cv::Scalar(10, 255, 255), image_out);
    cv::inRange(image_HSV, cv::Scalar(213,200,0), cv::Scalar(220, 255, 255), image_out);
    cv::namedWindow("Color Diff");
    cv::imshow("Color Diff",image_out);
    
    //cv::waitKey(3);
    cv::waitKey(0);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void imageCb2(const sensor_msgs::ImageConstPtr& msg2)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image2;
    double min_range_ = 0.2;
    double max_range_ = 0.8;
    try
    {
	cv_ptr = cv_bridge::toCvCopy(msg2, "32FC1");
	cv::Mat image2(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
	for(int i = 0; i < cv_ptr->image.rows; i++)
	{
		float* Di = cv_ptr->image.ptr<float>(i);
		char* Ii = image2.ptr<char>(i);
		for(int j = 0; j < cv_ptr->image.cols; j++)
		{   
		    if (Di[j]<=max_range_){
		    	Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
		    }
		    else {
		    	Ii[j] = (char) 0;
		    }
		}   
	}
	
	// Set up Matcher
    	RobustMatcher rmatcher;
    	rmatcher.setRatio(0.8f);
    	rmatcher.setDist(0.8);
    	cv::Ptr<cv::FeatureDetector> pfd= new cv::SurfFeatureDetector(600); 
    	rmatcher.setFeatureDetector(pfd);
    
    	// Read test Image
    	cv::Mat image_train = cv::imread("/home/mits/Documents/ANU/ENGN4627/Projects/ROS_test/test_img/depth5.png");
    	cv::Mat image_train2 = cv::imread("/home/mits/Documents/ANU/ENGN4627/Projects/ROS_test/test_img/depth5_2.png");
    	cv::Mat image_train3 = cv::imread("/home/mits/Documents/ANU/ENGN4627/Projects/ROS_test/test_img/depth5_3.png");
    
    	std::vector<cv::DMatch> matches;
    	std::vector<cv::KeyPoint> keypoints1, keypoints2;
    	
    	cv::Mat homography= rmatcher.match(image_train2, image2, matches, keypoints1, keypoints2);
    	homography= rmatcher.match(image_train3, image2, matches, keypoints1, keypoints2);

    	homography= rmatcher.match(image_train, image2, matches, keypoints1, keypoints2);
    
    	std::cout << "No. of keypoints (depth): " << keypoints1.size() << ", " << keypoints2.size() << std::endl;
    	std::cout << "No. of Matches (depth): " << matches.size() << std::endl;
	//cv::drawKeypoints(image2,keypoints2,image2);
	//cv::imshow(WINDOW2, image2);
	cv::Mat image_new;
	try {
		cv::drawMatches(image_train2, keypoints1, image2, keypoints2, matches, image_new);
		cv::namedWindow("Matches (depth)");
		cv::imshow("Matches (depth)", image_new);
	}
	catch (cv::Exception& e){
		// do nothing
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

