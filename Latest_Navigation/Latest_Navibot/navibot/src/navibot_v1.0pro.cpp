//*******************************************************
//             navibot_v1.0.cpp
//	modify-day : 24-10-2012
//*******************************************************

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include </opt/ros/electric/stacks/turtlebot/turtlebot_node/msg_gen/cpp/include/turtlebot_node/TurtlebotSensorState.h>
#include "sensor_msgs/LaserScan.h"
#include <ros/rate.h>
#include <iostream>
#include <vector>
using namespace std;




#define KEYCODE_U 0x41		//this means the "A"
#define KEYCODE_S 0x20		// this means the space 
#define KEYCODE_UP 0x26		// Up arrow 

#define Init 0
#define Start 1
#define Move 2
#define Justify 3
#define Circling 4			//Add on version 0.7
#define Detection 5			//Add on version 0.8



//************   Intersection Global Variables **************//
bool BeginDetect = false;
bool DetectMove=false;
bool gestureStart = false;
bool Detected = false;
int detectNo = 0;
bool detectFlag = false;

double DetectAng[3];
double DetectDis[3];

std::string commands="aplay /home/turtlebot/Group09/navibot/src/cow.wav";

//***********************************************************//
#include "imageDetect.cpp"
//Warm Up Project//////////////////////////
double bumperState;
double Dis=0,Ang=0;
double DisOld = 0, DisOld1 = 0;
///////////////////////////////////////

//Open Project Navibot part//////////////////
std::vector<float> RawData;  //Data need to be analyze
ros::Time period_start;		//Timer to control State
bool Record_flag=true;
int State=Init;
double MaxDis,TurnAng;		//Where to Go
int Counter;
bool MoveFlag=false;
double RangeMin;
double AngInc=0;
double DirectionMax[4];
bool BlockTimer = false;		//Add on version 0.7
bool ReverseFlag=true;
struct BaseData
{
	float distance;   //the distance to the object(it is illegal when greater than 10
	double  angle;  //the angle should justify
	bool legal;   // legal or illegal data
};
bool lessmark(const double s1,const double s2)   
{   
	return s1<s2;   
} 
bool greatermark(const BaseData& s1,const BaseData& s2)   
{   
	return s1.distance>s2.distance;   
}
void callback01(const turtlebot_node::TurtlebotSensorState::ConstPtr& msg){
	bumperState = msg->bumps_wheeldrops;
	Dis=Dis+msg->distance;
	Ang=Ang+msg->angle;
	//std::cout << "Dis and Ang = " << Dis << ", " << Ang << std::endl;
}
void callback02(const sensor_msgs::LaserScan::ConstPtr& msg);
class TurtlebotTeleop
{
public:
	TurtlebotTeleop();
	void keyLoop();
	void watchdog();
	void detect();

	ros::NodeHandle nh_,ph_;
	double linear_, angular_;
	ros::Time first_publish_;
	ros::Time last_publish_;
	ros::Time time1_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	void publish(double, double);
	boost::mutex publish_mutex_;
	int count;

};
TurtlebotTeleop::TurtlebotTeleop():
ph_("~"),
	linear_(0),
	angular_(0),
	l_scale_(1.0),
	a_scale_(1.0)
{
	ph_.param("scale_angular", a_scale_, a_scale_);
	ph_.param("scale_linear", l_scale_, l_scale_);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}
int main(int argc, char** argv)
{
	//The driving parameter is the public variable called State
	//State:Detect ---> Rotate the robot to detect the distance
	//State:Move   ---> Follow the Maxium distance to move
	//State:Modify   ---> Modify the current status to correct

	ros::init(argc, argv, "turtlebot_teleop");
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	
	TurtlebotTeleop turtlebot_teleop;
	ros::NodeHandle n;
	signal(SIGINT,quit);
	boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop));
	ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));
	ros::Timer timer2= n.createTimer(ros::Duration(0.01),boost::bind(&TurtlebotTeleop::detect,&turtlebot_teleop));
	ros::Subscriber sens_sub_ = n.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 10, callback01);
	ros::Subscriber sens_sub2_ = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, callback02);
	ros::spin();
	my_thread.interrupt() ;
	my_thread.join() ;
	return(0);
}
void TurtlebotTeleop::watchdog()
{
	int PN;
	int i;
	int NN;	

	std::string commands="aplay /home/turtlebot/Group09/navibot/src/cow.wav";
	std::vector<float> SB;
	if(State==Move)
	{

	SB=RawData;
	
	NN=SB.size();


		PN=(TurnAng<0)?1:-1;

		if(PN*Ang>PN*TurnAng&&PN!=0&&ReverseFlag)
		{
			angular_=-PN*3.0;
			linear_=0;
		}

		else if(ReverseFlag)
		{
			angular_=0;
			linear_=0.5;
			PN=0;
		}
		for(i=0;i<NN&&ReverseFlag;i++)
		{
			if(SB[i]<0.5&&ReverseFlag)
			{
				angular_=0.4;
				linear_=-0.1;
				PN=0;
				ReverseFlag=false;
			}
		}


	}
//	else
//	{
//		angular_=0;
//		linear_=0;
//	}
	
	if(State==Detection)
	{
		detectFlag = true;
		if (!Detected){		
			BeginDetect=true;	
			std::cout << "Not detected yet..." << std::endl;
		}
		else {
			BeginDetect=false;
			if (DetectMove) {	// DetectMove signifies going to the first obj
			    PN=(DetectAng[0]<0)?1:-1;
			    int PN2 = (DetectDis[0]<0)?1:-1;
				
			    if (ReverseFlag){
				if (PN2*Dis>(PN2*DetectDis[0]+0.3)&&PN2!=0) {
					angular_ = 0.0;
					linear_ = -PN2*1.0;
					Ang = 0.0;
					//std::cout << "moving towards object" << std::endl;
					std::cout << "Dis: " << PN2*Dis << ", " << PN2*DetectDis[0]+0.3 << std::endl;
				}
				else if(PN*Ang>PN*DetectAng[0]&&PN!=0){
					angular_=-PN*2.0;
					linear_=0.0;
					//std::cout << "turning towards object" << std::endl;
					std::cout << "Ang: " << PN*Ang << ", " << PN*DetectAng[0] << std::endl;
				}
				else {
					std::cout << "Going into second object" << std::endl;
					DetectMove = false;
					ReverseFlag = false;
					linear_ = 0.0;
					angular_ = 0.0;
					Dis = 0.0;
					Ang = 0.0;
				}
			    }
			    else {
				if(PN*Ang>PN*DetectAng[0]&&PN!=0){
					angular_=-PN*2.0;
					linear_=0.0;
					Dis = 0.0;
					//std::cout << "turning towards object" << std::endl;
					std::cout << "Ang: " << PN*Ang << ", " << PN*DetectAng[0] << std::endl;
				}
				else if (PN2*Dis>(PN2*DetectDis[0]+0.3)&&PN2!=0) {
					angular_ = 0.0;
					linear_ = -PN2*1.0;
					//std::cout << "moving towards object" << std::endl;
					std::cout << "Dis: " << PN2*Dis << ", " << PN2*DetectDis[0]+0.3 << std::endl;
				}
				else {
					std::cout << "Reverse to original position" << std::endl;
					DetectAng[0] = -DetectAng[0];
					DetectDis[0] = -DetectDis[0];
					ReverseFlag = true;
					Ang = 0.0;
					Dis = 0.0;
				}
			    }
			}
			else {
			    PN=(DetectAng[0]<0)?1:-1;
			    int PN2 = (DetectDis[0]<0)?1:-1;
				
			    if (ReverseFlag){
				if (PN2*Dis>(PN2*DetectDis[2]+0.3)&&PN2!=0) {
					angular_ = 0.0;
					linear_ = -PN2*1.0;
					Ang = 0.0;
					//std::cout << "moving towards object" << std::endl;
					std::cout << "Dis: " << PN2*Dis << ", " << PN2*DetectDis[0]+0.3 << std::endl;
				}
				else if(PN*Ang>PN*DetectAng[2]&&PN!=0){
					angular_=-PN*2.0;
					linear_=0.0;
					//std::cout << "turning towards object" << std::endl;
					std::cout << "Ang: " << PN*Ang << ", " << PN*DetectAng[0] << std::endl;
				}
				else {
					std::cout << "Stop!" << std::endl;
					linear_ = 0.0;
					angular_ = 0.0;
					
					publish(angular_, linear_);
					
					if(system(commands.c_str())!=0)
					{
					ROS_ERROR("Could not play sound");
					}
				}
			    }
			    else {
				if(PN*Ang>PN*DetectAng[2]&&PN!=0){
					angular_=-PN*2.0;
					linear_=0.0;
					Dis = 0.0;
					//std::cout << "turning towards object" << std::endl;
					std::cout << "Ang: " << PN*Ang << ", " << PN*DetectAng[0] << std::endl;
				}
				else if (PN2*Dis>(PN2*DetectDis[2]+0.3)&&PN2!=0) {
					angular_ = 0.0;
					linear_ = -PN2*1.0;
					//std::cout << "moving towards object" << std::endl;
					std::cout << "Dis: " << PN2*Dis << ", " << PN2*DetectDis[0]+0.3 << std::endl;
				}
				else {
					std::cout << "Reverse to original position" << std::endl;
					DetectAng[0] = -DetectAng[0];
					DetectDis[0] = -DetectDis[0];
					ReverseFlag = true;
					Ang = 0.0;
					Dis = 0.0;
				}
			    }
			}
			
			
		}
	}
	if(State==Circling)							//Add on version 0.7
	{		cout<<"welcome to detection section1"<<endl;
		
		BlockTimer=true;
		if(Ang<-3.14159*1.5)
		{
			linear_=0;
			angular_=0;
			Ang=0;
			TurnAng=0;
			MaxDis=0;
			State=Detection;
			cout<<"welcome to detection section2"<<endl;
		}
		else
		{
			cout<<"welcome to detection section3"<<endl;			
			linear_=0.5;
			angular_=-0.5;
			cout<<"welcome to detection section4"<<endl;		
		}
//		boost::mutex::scoped_lock lock(publish_mutex_);	//commented by summer
//		publish(angular_, linear_); //commented by summer

	}
	boost::mutex::scoped_lock lock(publish_mutex_);
	publish(angular_, linear_);
 
}
void TurtlebotTeleop::keyLoop()
{
	char c;


	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Waiting to start ---- Waiting to start -----Waiting to start");
	puts("--------------------------------------------------------------");
	puts("		Use up arrow keys to move the turtlebot.  ");
	puts("      Use spacebar to stop the turtlebot.       ");
	puts("--------------------------------------------------------------");


	while (ros::ok())
	{
		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}


		//linear_=angular_=0;			//*****Comment this line on version 0.5
		ROS_DEBUG("value: 0x%02X\n", c);

		switch(c)
		{

		case KEYCODE_U:
			period_start=ros::Time::now(); 
			//State=Start;
			MoveFlag=false;
			State=Detection;
			ReverseFlag = false;
			angular_=0;
			linear_=0;
			break;

		case KEYCODE_S:				//*****this part changed to make space to stop the robot
			period_start=ros::Time::now();
			State=Init;
			MoveFlag=false;
			angular_=0;
			linear_=0;
			break;
		}

		boost::mutex::scoped_lock lock(publish_mutex_);
		publish(angular_, linear_);
	}

	return;
}
void TurtlebotTeleop::publish(double angular, double linear)  
{
	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_*angular;
	vel.linear.x = l_scale_*linear;

	vel_pub_.publish(vel);    


	return;
}
void callback02(const sensor_msgs::LaserScan::ConstPtr& msg){
	RawData = msg->ranges;
	RangeMin= msg->range_min;
	AngInc=msg->angle_increment;
}
void TurtlebotTeleop::detect()
{

	if(ros::Time::now()>period_start+ros::Duration(6))//&&MoveFlag)//***********Removed by Summer
	{
		if(MoveFlag){
			ReverseFlag=true;								
			Record_flag=true;
			if(!BlockTimer)							//******** Add on version 0.7
				State=Start;
			else
				State=Circling;	
				//MoveFlag = false;			//***********Add by Summer
							//*******Add on version 0.7
			cout<<"The Timer Working in Duration&MoveFlag"<<endl;
		}
		else if (detectFlag) {
			// do nothing
		}
		else
		{
			Record_flag=false;
			State=Init;
		}
	}
	if(Record_flag){
		period_start=ros::Time::now();
		Record_flag=false;
	}
	if(State==Init)
	{
		Ang=0;
		Counter=0;
	}
	if(State==Justify)
	{
		publish(0,-0.1);
	}

	if(State==Start)
	{
		int num;
		boost::mutex::scoped_lock lock(publish_mutex_);			
		publish(-3.0,0);
		if(Ang<-3.14159/2)
		{
                        Ang=0;
			std::vector<float> data=RawData;
			num=data.size();
			static std::vector<BaseData>datastruct(num);
			int j=0;int Recording=1;
			for (int i=0;i<num;i++){					//Add on version 0.7 to delect the useless 11.00s
				if(data.at(i)>10&&Recording==1){
					datastruct[i].distance=0;
					j=i;
				}
				else
				{	Recording=0;
					datastruct[i].distance=data.at(i);
				}
				if(i>num-j)
					datastruct[i].distance=0;
				datastruct[i].angle=-3.14159+AngInc*i-3.14159/2*Counter;
				if(data.at(i)>10)
					datastruct[i].legal=true;
				else
					datastruct[i].legal=false;
			}
			Counter++;
			//The following part is the section where we will decide where to go
			std::sort(datastruct.begin(), datastruct.end(),greatermark);//greatersample
			for(int i=0;i<num;i++)
			{
				if(datastruct[i].distance<11)
				{
					DirectionMax[Counter-1]=datastruct[i].distance;		//Add on version 0.7
					if(MaxDis < datastruct[i].distance&&Counter<5) 
					{
						MaxDis = datastruct[i].distance;
						TurnAng = datastruct[i].angle;
						//just adjust the algorithm 
						
						//ROS_INFO("loop out");
					}
					break;
				}
			}
			if(Counter==4)
			{	
				if(MaxDis<RangeMin)
				{
					State=Justify;
					Counter=0;
				}
				else if(DirectionMax[0]>5&&DirectionMax[1]>2.3&&DirectionMax[3]>6)	//Add on version 0.7
				{
					State=Circling;
					linear_=0;
					angular_=0;
					Ang=0;
				}
				else
				{
					if(TurnAng<-3.14159)	{						
						TurnAng=3.14159*2+TurnAng;
						}
					if(DirectionMax[0]<1.0&&DirectionMax[2]<1.1&&DirectionMax[3]>6&&TurnAng<0)
					  {
					    TurnAng=TurnAng*1.2; 
					  }
					if(DirectionMax[0]<1.0&&DirectionMax[2]<1.1&&DirectionMax[3]>6&&TurnAng>0)
					  {
					    TurnAng=TurnAng*1.1; 
					  }
					
					cout<<"MaxDis :"<<MaxDis<<"   TurnAng:"<<TurnAng<<endl;
					Counter=0;
					linear_=0;
					angular_=0;
					publish(angular_,linear_);
					Ang=0;
					State=Move;
				}
			}
			
		}

	}

}


