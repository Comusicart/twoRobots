#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <cstdlib>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <knu_ros_lecture/ardrone_target.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

//드론에서 서브스크라이브 한 것과 goal의 rotation, translation간에 단위 환산  생각하기

//아래 하나는 새로 include한 것.
#define toRadian(degree) ((degree) * (M_PI / 180.))
#define velocity 0.25

//typedef actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> Client;
//드론으로부터 받아온 정보를 저장하는 변수
knu_ros_lecture::ardrone_target ardrone_scan;

boost::mutex mutex, droneMutex,odomMutex;
sensor_msgs::LaserScan g_scan;
nav_msgs::Odometry g_odom;
geometry_msgs::Twist cmdvel;
ros::Publisher movepub;

double target_distance; //목표지까지 남은 거리
double target_rotation;	//목표지까지 돌아야하는 각도

double taken_time = 0;
double cal_distance = 0;
double min_range_old;
bool near;
int leftRightCount;	//뒤로 획 돌기 전 횟수
int direction=0;
int obstacle = 0;


// lazerScan callback function
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	mutex.lock(); {
		g_scan = msg;
	} mutex.unlock();
}

// drone callback function
void droneMsgCallback(const knu_ros_lecture::ardrone_target& msg)
{
	droneMutex.lock();{
		ardrone_scan.tags_dis = msg.tags_dis;
		ardrone_scan.angular = msg.angular;
	}droneMutex.unlock();
}
void odomMsgCallback(const nav_msgs::Odometry &msg){
	odomMutex.lock();{
		g_odom=msg;
	}odomMutex.unlock();
}
void cal_dist(double x)
{
	//장애물을 만났을 경우
	//90도 회전을 해서 이동한 방향은 항상 0.5m
	if(obstacle == 1)
	{

        target_distance = sqrt(pow(direction*x,2.0) + pow((target_distance),2.0));
        target_rotation = atan(x/(target_distance-cal_distance));
        target_rotation=target_rotation*(( 180./M_PI));
        printf("x : %lf\n",x);
        printf("target_distance : %lf\n",target_distance);
        printf("cal_distance : %lf\n",cal_distance);
        printf("(target_distance-cal_distance) : %lf\n",target_distance-cal_distance);
        printf("target_rotation : %lf\n",target_rotation);
        //sleep(100);
        /*
        long version of code above.
        if(direction == 1)
            target_rotation = -(90-acos(x/target_distance));
        else
            target_rotation = 90-acos(x/target_distance);
        */
        cal_distance = 0;
	}
	//안만났을 경우
	else
	{	
        target_distance = target_distance - x;
		target_rotation = 0;
	}
	//target_distance에서 한번 이동한 만큼 거리를 뺀 값
}

// 장애물 회피 함수
void moveToSomeWhere(sensor_msgs::LaserScan& lrfScan)
{
	//가장 작은 range 값을 찾기 위한 변수
	double min_range = 0;
	double min_range_angle = 0;	
	int i;
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("/odom",100,&odomMsgCallback);
	ros::Publisher pub_reset=nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry",100);
	nav_msgs::Odometry odom;
	std_msgs::Empty odom_reset;
    //printf("move to target!!!!\n");//함수 실행 확인 함수

	int nRangeSize = (int)lrfScan.ranges.size();	//laserscan 값수
    //printf("%d\n", nRangeSize);

	if(nRangeSize == 0)	//처음 실행했을 때.
    {
        sleep(22);
        obstacle=0;
		//드론에게서 받은 방향으로 회전
        //cmdvel.angular.z = ardrone_scan.angular;
		cmdvel.linear.x = 0;	
        if(ardrone_scan.angular>0){
            cmdvel.angular.z=1.5;
			while(1){
				ros::spinOnce();
				odomMutex.lock();{
				odom=g_odom;
				}odomMutex.unlock();
                printf("leftmove_11\n");
                if(odom.pose.pose.orientation.z>ardrone_scan.angular-0.31){ // 값을 더 뺴야 함.
					cmdvel.angular.z=0;
					sleep(1);
					pub_reset.publish(odom_reset);
					sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
		}
		else{
            cmdvel.angular.z=-1.5;
			while(1){
				ros::spinOnce();
				odomMutex.lock();{
				odom=g_odom;
				}odomMutex.unlock();
                if(odom.pose.pose.orientation.z<ardrone_scan.angular+0.31){ // 값을 더 add야 함.
					cmdvel.angular.z=0;
					sleep(1);
					pub_reset.publish(odom_reset);
					sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
		}

        //sleep(5);   ////////////////////////////sleep/////////////////////

		return;
	}

	min_range = 10;	//min_range가 10이라고 가정
	
	for(int i=0 ; i < nRangeSize; i++) //가장 작은 Range 값 찾기
	{
		if(!isnan(lrfScan.ranges[i]))
		{
            //printf("range scan data in %d : %lf\n",i, lrfScan.ranges[i]);
		}
		if(lrfScan.ranges[i] < min_range && !isnan(lrfScan.ranges[i]))
		{
			min_range = lrfScan.ranges[i];
			min_range_angle = i;
		}
	}

    //printf("minimum range is [%f] at an angle of [%f] count [%d] \n", min_range, min_range_angle, leftRightCount);

	if(isnan(min_range))	//nan일때
	{
		//nan일 때, 직전 스캔 값이 0보다 크고 2보다 작을 경우에는 너무 가까운 경우라고 인식
		if(min_range_old > 0 && min_range_old <= 2)
		{
			near = true;
			printf("near!\n");
		}
        //nan일 때, 직전 스캔이 2보다 컸을 경우에는 너무 먼 경우라고 인식
		else if(min_range_old > 2)
		{
			near = false;
			printf("not near!!\n");
		}

		//너무 가까이 있어서 nan 일때는 후진을 하고, 드론에서 rotation고 translation을 받은 뒤 다시 goal을 설정한다.
		if(near)
		{

			cmdvel.linear.x = 0;
			cmdvel.angular.z = toRadian(180);
			printf("remake sinario!!!!!!!!!!!!\n");

			movepub.publish(cmdvel);
		}
		
		//앞에 아무 것도 없을 경우에는 아무 것도 하지 않는다.
	}
	else if(min_range <= 0.6)	//nan이 아니고 그냥 가까운 경우
	{	
        printf("min_rance<=0.6!!!!!\n");
        obstacle=1;
		ros::spinOnce();
		odomMutex.lock();{
			odom=g_odom;
		}odomMutex.unlock();
		cal_distance=odom.pose.pose.position.x;
		if(min_range_angle < nRangeSize/2)	//왼쪽으로 돌기 
		{
            cmdvel.angular.z=1.3;
			direction=1;
			//90도 회전
			cmdvel.linear.x = 0;
			while(1){
				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
				if(odom.pose.pose.orientation.z>0.39){
					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
			//rate.sleep();//살려두기
			/*
			printf("left!!!\n");
			for(i=0; i<20; i++)
			{
				cmdvel.linear.x = 0.5;
				cmdvel.angular.z = 0;
				movepub.publish(cmdvel);
			}
			*/	
            ros::Rate rate(10);
            cmdvel.linear.x = 0.5;
            cmdvel.angular.z = 0;
            int temp=0;
            while(ros::ok()){
                if(temp==20) break;
                movepub.publish(cmdvel);
                rate.sleep();
                temp++;
            }
            cmdvel.linear.x = 0;
            cmdvel.angular.z = -1.3;
            ros::spinOnce();
            odomMutex.lock();{
                odom=g_odom;
            }odomMutex.unlock();
            double x=odom.pose.pose.position.x;
			while(1){
				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
				if(odom.pose.pose.orientation.z<-0.39){
					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}


            cal_dist(x);    ///////////////////////cal_dist()//////////////


            cmdvel.linear.x = 0;
            cmdvel.angular.z = -0.3;
			while(1){
                printf("target_rotation : %lf\n",target_rotation);

				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
                printf("odom.pose.pose.orientation.z : %lf\n",odom.pose.pose.orientation.z);
                printf("(-1)*((target_rotation*7)/900) : %lf\n",(-1)*((target_rotation*7)/900));
                if(odom.pose.pose.orientation.z<(-0.8)*((target_rotation*7)/900)){

					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
			leftRightCount++;
		}
		else					//오른쪽으로 돌기
		{
            cmdvel.angular.z=-1.3;
			direction=-1;
			//90도 회전
			//cmdvel.angular.z = toRadian(-90);
			cmdvel.linear.x = 0;
			while(1){
				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
                if(odom.pose.pose.orientation.z<-0.39){
					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
			
			//rate.sleep();
			/*
                        printf("rightt!!!\n");
                        for(i=0; i<20; i++)

                                cmdvel.linear.x = 0.5;
                                cmdvel.angular.z = 0;
                                movepub.publish(cmdvel);
                        }
			*/
            ros::Rate rate(10);
            cmdvel.linear.x = 0.5;
            cmdvel.angular.z = 0;
            int temp=0;
            while(ros::ok()){
                if(temp==20) break;
                movepub.publish(cmdvel);
                rate.sleep();
                temp++;
            }
            /*
            movepub.publish(cmdvel);
            sleep(1);
            */
            cmdvel.linear.x = 0;
            cmdvel.angular.z = 1.3;
            ros::spinOnce();
            odomMutex.lock();{
                odom=g_odom;
            }odomMutex.unlock();
            double x=odom.pose.pose.position.x;
			while(1){
                //printf("\n\nfufufufuf\n\n");

				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
                if(odom.pose.pose.orientation.z>0.39){
					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
            }


            cal_dist(x);        /////////////////cal_dist()////////////////////



            cmdvel.linear.x = 0;
            cmdvel.angular.z = 0.3;
			while(1){
                printf("target_rotation : %lf\n",target_rotation);

				ros::spinOnce();
				odomMutex.lock();{
					odom=g_odom;
				}odomMutex.unlock();
                printf("odom.pose.pose.orientation.z : %lf\n",odom.pose.pose.orientation.z);
                printf("(-1)*((target_rotation*7)/900) : %lf\n",(-1)*((target_rotation*7)/900));
                if(odom.pose.pose.orientation.z>(-0.8)*((target_rotation*7)/900)){
					cmdvel.angular.z=0;
                    sleep(1);
					pub_reset.publish(odom_reset);
                    sleep(1);
					movepub.publish(cmdvel);
					break;
				}
				movepub.publish(cmdvel);
			}
                        leftRightCount++;
		}
		
		if(leftRightCount > 10)
		{
			printf("remake sinario!!!!!!!!!!!!!!!!!!");
			cmdvel.angular.z = toRadian(180);
			cmdvel.linear.x = velocity;
			printf("fuck!!!!\n");
			leftRightCount = 0;
            sleep(100);
		}
	}
	else //장애물이 없을 때, 타겟을 향하여 직진!!!
	{
        obstacle=0;
		//target_rotation만큼 돈다.
		//그리고 직진한다.
        //printf("march forward!!!!!\n");
		cmdvel.angular.z = 0;
		cmdvel.linear.x = 0.5;
        movepub.publish(cmdvel);
        double x=0.01;
        cal_dist(x);
	}

	if(isnan(min_range))
	{
        //printf("[%lf] min_range_old\n", min_range_old);
	}
	else
	{
		min_range_old = min_range;
        //printf("[%lf] min_range_old\n", min_range_old);
	}
	//남은 거리를 계산.
    printf("***target_distance : %lf\n",target_distance);
	return;
}

int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "avoidance_move");
	ros::NodeHandle moveHandle;
	ros::NodeHandle scanHandle;
	ros::NodeHandle droneHandle;
	sensor_msgs::LaserScan scan;

    ardrone_scan.tags_dis = 4.2;
    ardrone_scan.angular = -0.70;
	movepub = moveHandle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);
    ros::Subscriber droneScan = droneHandle.subscribe("Target_location",1000,&droneMsgCallback);
    ros::spinOnce();
	target_distance = ardrone_scan.tags_dis;
	ros::Subscriber subScan = scanHandle.subscribe("/scan", 10, &scanMsgCallback);
	
	min_range_old = -1;
	near = false;
	leftRightCount = 0;
	

    //main loop
        //ros::Rate rate(10);
	while(ros::ok()) {
		// callback 함수 call!
		ros::spinOnce();

		// receive the global '/scan' message with the mutex
		mutex.lock(); {
			scan = g_scan;
		} mutex.unlock();

		// scan으로 부터 움직일 방향 정하기
		moveToSomeWhere(scan);
		
        if(target_distance < 0)
			break;
		//rate.sleep();
	}

    printf("\n\n\n\n\n\n\n\n\n\n\n!!!!!!!!DONE!!!!!!!!\n\n\n\n\n\n\n\n\n");

	return 0;
}
