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

//Define the Key Value*******************
#define KEYCODE_U 0x41		//this means the "A"
#define KEYCODE_S 0x20		// this means the space 
#define KEYCODE_UP 0x26		// Up arrow 

//Define the state*******************
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
int seeNo = 0;	// Number of objects in view
bool detectFlag =false;
double DetectAng[3]={0,0,0};
double DetectDis[3]={0,0,0};
int PN2=0;

std::string commands="aplay /home/turtlebot/Group09/navibot/src/cow.wav";

//***********************************************************//
#include "imageDetect.cpp"
//**************************


//Warm Up Project//////////////////////////
double bumperState=0;
double Dis=0,Ang=0;
double DisOld = 0, DisOld1 = 0;
///////////////////////////////////////

//Open Project Navibot part//////////////////
std::vector<float> RawData(360,0);  //Data need to be analyze
std::vector<float> SB(360,0);  //Data need to be analyze
std::vector<float> data(360,0);
std::vector<float> SB2(360,0);
ros::Time period_start;		//Timer to control State
ros::Time move_start;	
bool Record_flag=true;
int State=Init;
double MaxDis=0,TurnAng=0;		//Where to Go
int Counter=0;
bool MoveFlag=false;
double RangeMin=0;
double AngInc=0;
double DirectionMax[4];
bool BlockTimer = false;
bool ReverseFlag=true;
int PN=0;
int num=0;
int CircleCounter=0;
double Freq =0.01;	
int TurnCounter=0;
//std::string commands="aplay cow.wav";


struct BaseData
{
	float distance;   //the distance to the object(it is illegal when greater than 10
	double  angle;  //the angle should justify
	bool legal;   // legal or illegal data
};
//************GreaterMark function to sort the data********************
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
}
void callback02(const sensor_msgs::LaserScan::ConstPtr& msg){
	RawData = msg->ranges;
	RangeMin= msg->range_min;
	AngInc=msg->angle_increment;
}


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
	TurtlebotTeleop turtlebot_teleop;


	///******************* InterSection with Image detection*******************
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	///**************************************************************


	ros::NodeHandle n;
	signal(SIGINT,quit);
	boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop));
	ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));
	ros::Timer timer2= n.createTimer(ros::Duration(0.01),boost::bind(&TurtlebotTeleop::detect,&turtlebot_teleop));

	ros::Subscriber sens_sub_ = n.subscribe<turtlebot_node::TurtlebotSensorState>("/turtlebot_node/sensor_state", 1000, callback01);
	ros::Subscriber sens_sub2_ = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, callback02);

	ros::spin();
	my_thread.interrupt() ;
	my_thread.join() ;
	return(0);
}
void TurtlebotTeleop::watchdog()
{

	if(ros::Time::now()>period_start+ros::Duration(5.5))
	{
		if(MoveFlag){
			//angular_=0;
			//linear_=0;
            //State=Start;
			if(BlockTimer)	
				{
					State=Circling;
					Record_flag=true;
			    }
			else
			{
			Record_flag=true;						//******** Add on version 0.7
			State=Start;
			cout<<"The Timer Working in Duration&MoveFlag"<<endl;
			}
			
		}
		else if (detectFlag){//do nothing
			State=Detection;
			//angular_=0;
			//linear_=0;
			//boost::mutex::scoped_lock lock(publish_mutex_);
			//publish(angular_, linear_);
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
//	boost::mutex::scoped_lock lock(publish_mutex_);			
//	publish(angular_,linear_);


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
			State=Start;
           // State=Detection;
			MoveFlag=true;
           
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

void TurtlebotTeleop::detect()
{
	//******************    Local Variable **********************
	//int PN=0;
	//int num=0;
	//********************************************
	switch(State)
	{	
	case Init:
		angular_=0;
		linear_=0;
		Ang=0;
		Counter=0;
		break;
	case Start:
		angular_=-3;
		linear_=0;
		if(Ang<-3.14159/2)
		{
			Ang=0;
			data=RawData;
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
			{	move_start=ros::Time::now();


			/*if(DirectionMax[0]<0.5||DirectionMax[1]<0.5||DirectionMax[2]<0.5||DirectionMax[3]<0.5)
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
				//publish(angular_,linear_);
				Ang=0;
				State=Justify;
			}*/


			if(DirectionMax[0]>3.2&&DirectionMax[1]>2.5&&DirectionMax[3]>5)	//Add on version 0.7
			{
				//State=Circling;
                State=Justify;	
                period_start=ros::Time::now();			
                linear_=0;
				angular_=0;
                TurnAng =0;//added by summer
                BlockTimer=true;;
				CircleCounter=1;
                Ang=0;
                Ang=0;
				Dis=0;
                Dis=0;
                system(commands.c_str());
				cout<<"I am turning"<<endl;
                publish(angular_,linear_);
			}
			else
			{
				if(TurnAng<-3.14159)	{						
					TurnAng=3.14159*2+TurnAng;
				}
				if((DirectionMax[0]<0.5||DirectionMax[2]<0.5)&&DirectionMax[3]>6&&TurnAng<0)
				{
					TurnAng=TurnAng*3; 
				}
				if((DirectionMax[0]<0.5||DirectionMax[2]<0.5)&&DirectionMax[3]>6&&TurnAng>0)
				{
					TurnAng=TurnAng*3; 
				}

				cout<<"MaxDis :"<<MaxDis<<"   TurnAng:"<<TurnAng<<endl;
				Counter=0;
				linear_=0;
				angular_=0;
				//publish(angular_,linear_);
				Ang=0;
				State=Move;
				PN=(TurnAng<0)?1:-1;
			}
			}

		}

		break;
	case Move:
		SB=RawData;
		SB2=RawData;
		std::sort(SB.begin(),SB.end(),lessmark);
		if(PN*Ang<PN*TurnAng&&PN!=0)
		{	
			if(SB.at(0)<0.45)
			{
			angular_=0.5;
			linear_=-0.1;
			}
			else{
			angular_=0;
			linear_=0.5;
			}
			Ang=0;
			TurnAng=0;
			MaxDis=0;
			PN=0;
		}
		else if(PN*Ang>PN*TurnAng&&PN!=0)
		{	
			angular_=-PN*3.0;
			linear_=0;
		}
		else
		{
			if(SB.at(0)<0.6)
			{
			angular_=0;
			linear_=-0.2;
			}
			if(SB.at(0)>0.6)
			{
				linear_=0.5;
				angular_=0;
			}
			if(SB2.at(150)>5&&SB2.at(210)>5)
			{linear_=0.5;
			angular_=0;
			}
		}
		//boost::mutex::scoped_lock lock(publish_mutex_);			
		//publish(angular_,linear_);
		break;
	case Circling:
		//cout<<"welcome to detection section11"<<endl;
		BlockTimer=true;
        if(!(CircleCounter%2))
        {
            linear_=1;
			angular_=0;
            publish(0,0);
        }
        else
        {
            linear_=0;
			angular_=-3;
            publish(0,0);
        }
            
		if(Ang<-1.24&&(CircleCounter%2))
		{

			Ang=0;
			Dis=0;
			CircleCounter+=1;
			cout<<"I am llll"<<endl;
		}
		if(Dis>1.7&&!(CircleCounter%2))
		{
			Ang=0;
			Dis=0;
			CircleCounter+=1;		
			cout<<"I am in circling"<<endl;
            cout<<"ANG:"<<Ang<<"Dis:"<<Dis<<endl;
		}
		if(CircleCounter==6)
		{
			angular_=0;
			linear_=0;
			MoveFlag=false;
			detectFlag=true;
            Ang=0;
            Dis=0;
			ReverseFlag = false;
			cout<<"go to detection"<<endl;
            cout<<"ANG:"<<Ang<<"Dis:"<<Dis<<endl;
			State=Detection;
            Freq=0.1;
		}
		cout<<"ANG:"<<Ang<<"Dis:"<<Dis<<endl;
		
		//boost::mutex::scoped_lock lock(publish_mutex_);			
		//publish(angular_,linear_);
		break;
	case Detection:
		detectFlag = true;
		if (!Detected){		
			BeginDetect=true;	
			std::cout << "Not detected yet..." << std::endl;
		}
        if(Detected)
        {
            detectFlag=true;
            switch(TurnCounter)
            {
                case 0:
                        PN=(DetectAng[0]<0)?1:-1;
                        cout <<"Ang"<<Ang<<endl;
                        angular_=-PN*0.3;
	                    if(PN*Ang>PN*DetectAng[0])
		                {	
			            angular_=-PN*0.3;
			            linear_=0;
                        }
                        if(PN*Ang<PN*DetectAng[0])
		                {	
			            angular_=0;
			            linear_=0.5;
			            Ang=0;
                        Dis=0;
			            PN=0;
                        TurnCounter+=1;
		                }
                    break;
                case 1:
	                    if(Dis>DetectDis[0]-0.5)
                        {
                            angular_=0;
			            linear_=-0.5;
			            Ang=0;
                        Dis=0;
			            PN=0;
                        TurnCounter+=1;
                        }                           
                    break;
                case 2:
                       PN=(DetectAng[1]<0)?1:-1;
                       if((-Dis)>(DetectDis[0]-0.5))
                        {
                        angular_=-PN*0.3;
			            linear_=0;
			            Ang=0;
                        Dis=0;
			            PN=0;
                        TurnCounter+=1;
                        }                           
                    break; 
                case 3:
                        PN=(DetectAng[1]<0)?1:-1;
                        cout<<"DD"<<DetectAng[1]<<endl;
                        angular_=-PN*0.3;
	                    if(PN*Ang>PN*DetectAng[1])
		                {	
			            angular_=-PN*0.3;
			            linear_=0;
                        }
                        if(PN*Ang<PN*DetectAng[1])
		                {	
			            angular_=0;
			            linear_=0.5;
			            Ang=0;
                        Dis=0;
			            PN=0;
                        TurnCounter+=1;
                        Dis=0;
		                }
                    break;
                case 4:
                    if(Dis>DetectDis[1]-0.2)
                        {
                            angular_=0;
                            linear_=0;
                            Ang=0;
                            Dis=1000;
                            cout<<"DetectAng1"<<DetectAng[0]<<"DetectAng2"<<DetectAng[1]<<endl;
                            if(system(commands.c_str())!=0)
					        {
					        ROS_ERROR("Could not play sound");
					        }
                        }
                        break;
                default:
                    angular_=angular_;
                    linear_=linear_;
                    break;
            }
         publish(angular_,linear_);
         cout<<"Counter"<<TurnCounter<<endl;
         cout <<"Dis"<<Dis<<endl;
                                   

        }
		
		break;
	case Justify:
		cout<<"Justify"<<endl;
        linear_=0;
        angular_=3.0;
        if(Ang>1.5)
        {
            State=Circling;
            Ang=0;
            Dis=0;
            linear_=0;
            angular_=0;
          }
            
		break;
	default:
		angular_=0;
		linear_=0;
		//boost::mutex::scoped_lock lock(publish_mutex_);			
		//publish(angular_,linear_);
		break;
	}
		boost::mutex::scoped_lock lock(publish_mutex_);			
		publish(angular_,linear_);

}


