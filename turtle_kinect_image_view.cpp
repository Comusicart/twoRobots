#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ardrone_autonomy/Navdata.h>
#include <nav_msgs/Odometry.h>
#include <knu_ros_lecture/ardrone_target.h>


#define DRONE_SPEED 0.5
// 1:1200 , 2:1500 , 3:2000
#define DRONE_INITIAL_ALT 1250
#define DRONE_ALT_SPEED 2

using namespace cv;
using namespace std;
using namespace ros;

int lastx,lasty;
Publisher land_pub;
Publisher move_pub;
double tags_dis;
double drone_angular;
boost::mutex mutex[2];
ardrone_autonomy::Navdata g_navmsg;
nav_msgs::Odometry g_odommsg;
int turtle_scan = 1;
geometry_msgs::Twist movemsg;
int ExitFlag = 0;
double prev_altd;

void ChangeMovemsgPublishThread()
{
    geometry_msgs::Twist tempmsg;
    int flag=1;
    tempmsg.linear.x = 0;
    tempmsg.linear.y = 0;
    tempmsg.linear.z = 0;
    tempmsg.angular.z =  0;
    sleep(10);
    ROS_INFO("[***]MovePub) x : %.3lf  y : %.3lf z : %.3lf angz : %.3lf",movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
    while(1)
    {
        //ROS_INFO("[***]TempPub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,tempmsg.linear.x,tempmsg.linear.y,tempmsg.linear.z,tempmsg.angular.z );
        //ROS_INFO("[***]MovePub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
        if(movemsg.linear.x != tempmsg.linear.x){
            tempmsg.linear.x = movemsg.linear.x;
            ROS_INFO("[***][*]MovePub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
            flag = 1;
        }
        if(movemsg.linear.y != tempmsg.linear.y){
            tempmsg.linear.y = movemsg.linear.y;
            flag = 1;
            ROS_INFO("[***][*]MovePub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
        }
        if(movemsg.linear.z != tempmsg.linear.z){
            tempmsg.linear.z = movemsg.linear.z;
            flag = 1;
            ROS_INFO("[***][*]MovePub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
        }
        if(movemsg.angular.z != tempmsg.angular.z){
            tempmsg.angular.z = movemsg.angular.z;
            flag = 1;
            ROS_INFO("[***][*]MovePub) flag : %d x : %lf  y : %lf z : %lf angz : %lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
        }
        if(flag == 1){
            move_pub.publish(movemsg);
            ROS_INFO("[****]MovePub) flag : %d x : %.3lf  y : %.3lf z : %.3lf angz : %.3lf",flag,movemsg.linear.x,movemsg.linear.y,movemsg.linear.z,movemsg.angular.z );
            flag = 0;
        }
        if(ExitFlag == 1){
            break;
        }
    }


}

void navdataReceived(const ardrone_autonomy::Navdata& msg){

    mutex[0].lock();{
        g_navmsg=msg;
    }mutex[0].unlock();

}

geometry_msgs::Twist calDroneMove(const ardrone_autonomy::Navdata& msg){

    //geometry_msgs::Twist movemsg;
    movemsg.linear.z=0;

    // Alt MainTaining
    if(msg.altd>DRONE_INITIAL_ALT+50){
        movemsg.linear.z=-DRONE_ALT_SPEED;
    }
    else if(msg.altd<DRONE_INITIAL_ALT-50){
        movemsg.linear.z=DRONE_ALT_SPEED;
    }
    else movemsg.linear.z=0;
    // Alt MainTaining END

    // Target Lost
    if(msg.tags_type.empty()){        

        movemsg.linear.y=0;
        movemsg.linear.x=0;
        movemsg.angular.z=0;
        //ROS_INFO("target lost");
    }
    else{        
        if(msg.tags_count==1&&msg.tags_type[0]==131072){ //타켓못찾았을때(탐색모드)

            lastx=msg.tags_xc[0];
            lasty=msg.tags_yc[0];
            if(0<msg.tags_xc[0]&&msg.tags_xc[0]<400)
                movemsg.linear.y=DRONE_SPEED;
            else if(600<msg.tags_xc[0]&&msg.tags_xc[0]<1000)
                movemsg.linear.y=-DRONE_SPEED;
            else{
                movemsg.linear.y=0;
            }
            if(0<msg.tags_yc[0]&&msg.tags_yc[0]<400)
                movemsg.linear.x=DRONE_SPEED;
            else if(660<msg.tags_yc[0]&&msg.tags_yc[0]<1000)
                movemsg.linear.x=-DRONE_SPEED;
            else{
                movemsg.linear.x=0;
            }
            movemsg.angular.z = 0;

            //move_pub.publish(movemsg);
            ROS_INFO("[*]case 1) xc : %d  yc : %d",msg.tags_xc[0],msg.tags_yc[0]);
            ROS_INFO("case 1) x : %.3lf  y : %.3lf angz : %.3lf",movemsg.linear.x,movemsg.linear.y,movemsg.angular.z );
        }
        else  if(msg.tags_count==1&&msg.tags_type[0]==0){
            ROS_INFO("[*]Only Target Detected");           
            movemsg.linear.x=0;
            movemsg.linear.y=0;
            /* 각도조절 */
            /*
            if(0<msg.tags_xc[0]&&msg.tags_xc[0]<330)
                movemsg.angular.z=-0.0001;
            else if(660<msg.tags_xc[0]&&msg.tags_xc[0]<1000)
                movemsg.angular.z=0.0001;
            else
                movemsg.angular.z=0;
            */
            movemsg.angular.z=0;
            //move_pub.publish(movemsg);
        }
        else if(msg.tags_count==2&&msg.tags_type[0]==0){//타겟을찾았을
            lastx=msg.tags_xc[1];
            lasty=msg.tags_yc[1];            

            if(0< lastx && lastx <400)
                movemsg.linear.y=DRONE_SPEED;
            else if(600< lastx && lastx <1000)
                movemsg.linear.y=-DRONE_SPEED;
            else{
                movemsg.linear.y=0;
            }
            if(0< lasty && lasty <400)
                movemsg.linear.x=DRONE_SPEED;
            else if(600< lasty && lasty <1000)
                movemsg.linear.x=-DRONE_SPEED;
            else{
                movemsg.linear.x=0;
            }

                 /* 각도조절 */
            if(0<msg.tags_xc[0]&&msg.tags_xc[0]<330)
                movemsg.angular.z=0.1;
            else if(660<msg.tags_xc[0]&&msg.tags_xc[0]<1000)
                movemsg.angular.z=-0.1;
            else
                movemsg.angular.z=0;

            mutex[0].lock();{
                g_navmsg=msg;
            }mutex[0].unlock();
            //move_pub.publish(movemsg);
            ROS_INFO("case 2) x : %.3lf  y : %.3lf angular.z : %.3lf",movemsg.linear.x,movemsg.linear.y,movemsg.angular.z );

        }
        else {
            movemsg.linear.y=0;
            movemsg.linear.x=0;
            movemsg.linear.z=0;
            movemsg.angular.z=0;
            //move_pub.publish(movemsg);
            ROS_INFO("target lost222");
        }

    }

    //ROS_INFO("WHAT) x : %.3lf  y : %.3lf z : %.3lf",movemsg.linear.x,movemsg.linear.y,movemsg.linear.z );

    return movemsg;
}

void targetThread(){
    NodeHandle nh;
    ardrone_autonomy::Navdata navmsg;
    nav_msgs::Odometry odommsg;
    knu_ros_lecture::ardrone_target targetmsg;
    Publisher target_pub = nh.advertise<knu_ros_lecture::ardrone_target>("Target_location",1000);

    Rate rate(2);

    while(ok()){
        mutex[0].lock();{
            navmsg=g_navmsg;
        }mutex[0].unlock();        
        if(navmsg.tags_count==2){
            mutex[1].lock();{
                odommsg=g_odommsg;
            }mutex[1].unlock();
            tags_dis=targetmsg.tags_dis=navmsg.tags_distance[0];
            drone_angular=targetmsg.angular=odommsg.pose.pose.orientation.z;
            target_pub.publish(targetmsg);
            ROS_INFO("[**][**]tags_dis : %.3lf  drone_angular : %.3lf",tags_dis,drone_angular);
            ROS_INFO("publiros::NodeHandle nh;sh!!!!!");
            break;
        }
        if(navmsg.tags_count>0&&navmsg.tags_type[0]==0){
            mutex[1].lock();{
                odommsg=g_odommsg;
            }mutex[1].unlock();
            tags_dis=targetmsg.tags_dis=navmsg.tags_distance[0];
            drone_angular=targetmsg.angular=odommsg.pose.pose.orientation.z;
            ROS_INFO("[**][**]tags_dis : %.3lf  drone_angular : %.3lf",tags_dis,drone_angular);
        }
        if(ExitFlag==1){
            break;
        }
        rate.sleep();
    }

}
void odomReceived(const nav_msgs::Odometry &msg){
    mutex[1].lock();{
        g_odommsg=msg;
    }mutex[1].unlock();
}



geometry_msgs::Twist DroneScanning(const ardrone_autonomy::Navdata& msg){

    //geometry_msgs::Twist movemsg;
    /*

    // Alt MainTaining
    if(msg.altd>DRONE_INITIAL_ALT+50){
        movemsg.linear.z=-DRONE_ALT_SPEED;
    }
    else if(msg.altd<DRONE_INITIAL_ALT-50){
        movemsg.linear.z=DRONE_ALT_SPEED;
    }
    else movemsg.linear.z=0;
    // Alt MainTaining END
    */

    movemsg.linear.x=0;
    movemsg.linear.y=0;
    movemsg.linear.z=0;
    movemsg.angular.x=0;
    movemsg.angular.y=0;
    movemsg.angular.z=-0.2;
    //ROS_INFO("SCANNING");

    if(msg.tags_count>0&&msg.tags_type[0]==0){
        turtle_scan = 0;
        movemsg.angular.x = 0;
        movemsg.angular.y = 0;
        movemsg.angular.z = 0.2;
        sleep(1);
        //move_pub.publish(movemsg);
        ROS_INFO("[*]case 1) xc : %d  yc : %d",msg.tags_xc[0],msg.tags_yc[0]);
    }

    move_pub.publish(movemsg);    
    return movemsg;
}

void moveThread(){
    NodeHandle nh;
    move_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    //geometry_msgs::Twist movemsg;
    ardrone_autonomy::Navdata navmsg;
    nav_msgs::Odometry odommsg;

    //Rate rate(10);

    while(1){
        mutex[0].lock();{
            navmsg=g_navmsg;
        }mutex[0].unlock();
        /*
        mutex[1].lock();{
            odommsg=g_odommsg;
        }mutex[1].unlock();
        */

        if(turtle_scan==1){
            //movemsg=
            DroneScanning(navmsg);
        }
        else {
            //movemsg=
            calDroneMove(navmsg);
            //move_pub.publish(movemsg);
        }

        //move_pub.publish(movemsg);
        if(ExitFlag==1){
            break;
        }
        //rate.sleep();
    }
}


void poseMessageReceivedRGB(const sensor_msgs::Image& msg) {
    //ROS_INFO("seq = %d / width = %d / height = %d / step = %d", msg.header.seq, msg.width, msg.height, msg.step);
    //ROS_INFO("encoding = %s", msg.encodnavmsging.c_strater());
    //vector<unsigned char> data = msg.data;
    ros::NodeHandle nh;
    //geometry_msgs::Twist movemsg;

    Mat image = Mat(msg.height, msg.width, CV_8UC3);
    memcpy(image.data, &msg.data[0], sizeof(unsigned char)*msg.data.size());
    imshow("RGB Preview", image);
    char ch= waitKey(30);
    if(ch=='t'){
        ServiceClient client = nh.serviceClient<std_srvs::Empty>("/ardrone/togglecam");
        std_srvs::Empty togglecamasg;
        if(!client.call(togglecamasg)){
            ROS_INFO("failed to call service /ardrone/togglecam");
        }
        else{
             ROS_INFO("toglecam");
        }
    }else if(ch=='l'){
        ExitFlag = 1;
        sleep(1);
        movemsg.linear.x=0;
        movemsg.linear.y=0;
        movemsg.linear.z=0;
        movemsg.angular.z=0;
        move_pub.publish(movemsg);

        land_pub =nh.advertise<std_msgs::Empty>("/ardrone/land",1);
        std_msgs::Empty landmsg;
        land_pub.publish(landmsg);
        ROS_INFO("land!!");
    }else if(ch=='s'){
        turtle_scan = 1;
        ROS_INFO("SCAN!!");
    }
}

geometry_msgs::Twist InitialTakeOff(const ardrone_autonomy::Navdata& msg){

    //geometry_msgs::Twist movemsg;
    movemsg.linear.x=0;
    movemsg.linear.y=0;
    movemsg.linear.z=0;

    double altd_delta = prev_altd - msg.altd;
    if(altd_delta > 100){
        ROS_INFO("AnyThing exist below me");
        /*
         *movemsg.linear.z=0;
         * prev_updown_flag = 1;
         * updown flag  가 1일 때 DRONE_INITIAL_ALT 에 altd_delta 를 뺀다.
        */
    }
    if(altd_delta < -100){
        ROS_INFO("AnyThing below me shade out");
        /*
         *movemsg.linear.z=0;
         * prev_updown_flag = -1;
         * updown flag  가 -1일 때 DRONE_INITIAL_ALT 에 altd_delta 를 더다한.
        */
    }

    if(msg.altd>DRONE_INITIAL_ALT+50){
        movemsg.linear.z=-DRONE_ALT_SPEED;
    }
    else if(msg.altd<DRONE_INITIAL_ALT-50){
        movemsg.linear.z=DRONE_ALT_SPEED;
    }
    else movemsg.linear.z=0;

    ROS_INFO("[*][*]Initial Take Off)Linea.z : %.3lf",movemsg.linear.z );

    if(msg.tags_count==1&&msg.tags_type[0]==131072){ //타켓못찾았을때(탐색모드)

        if(0<msg.tags_xc[0]&&msg.tags_xc[0]<400)
            movemsg.linear.y=DRONE_SPEED;
        else if(600<msg.tags_xc[0]&&msg.tags_xc[0]<1000)
            movemsg.linear.y=-DRONE_SPEED;
        else{
            movemsg.linear.y=0;
        }
        if(0<msg.tags_yc[0]&&msg.tags_yc[0]<400)
            movemsg.linear.x=DRONE_SPEED;
        else if(660<msg.tags_yc[0]&&msg.tags_yc[0]<1000)
            movemsg.linear.x=-DRONE_SPEED;
        else{
            movemsg.linear.x=0;
        }
        //move_pub.publish(movemsg);
        ROS_INFO("[*]case 0) xc : %d  yc : %d",msg.tags_xc[0],msg.tags_yc[0]);
        ROS_INFO("case 0) x : %.3lf  y : %.3lf angz : %.3lf",movemsg.linear.x,movemsg.linear.y,movemsg.angular.z );
    }
    prev_altd = movemsg.linear.z;

    return movemsg;
}

void InitialTakeOffThread(){
    NodeHandle nh;
    move_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    ardrone_autonomy::Navdata navmsg;
    geometry_msgs::Twist tempmsg;
    Rate rate(10);

    while(ok()){
        mutex[0].lock();{
            navmsg=g_navmsg;
        }mutex[0].unlock();

        tempmsg=InitialTakeOff(navmsg);
        //move_pub.publish(movemsg);

        if(tempmsg.linear.z == 0){
            sleep(1);
            boost::thread t1(&moveThread);
            break;
        }
        if(ExitFlag==1){
            break;
        }
        rate.sleep();
    }
}


int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtlenavmsg_kinect_image_view");
    ros::NodeHandle nh;

    ros::Subscriber subRgb = nh.subscribe("/ardrone/image_raw", 10, &poseMessageReceivedRGB);
    Subscriber bottomsub=nh.subscribe("/ardrone/navdata",100,&navdataReceived);
    Subscriber odomsub=nh.subscribe("/ardrone/odometry",100,&odomReceived);



    boost::thread t0(&InitialTakeOffThread);
    Publisher takeoff_pub=nh.advertise<std_msgs::Empty>("/ardrone/takeoff",1000);
    std_msgs::Empty upmsg;

    sleep(1);
    takeoff_pub.publish(upmsg);
    ROS_INFO("take off");
    boost::thread t2(&targetThread);

    boost::thread t3(&ChangeMovemsgPublishThread);
    // Create a subscriber object


    // Let ROS take over
    ros::spin();

    return 0;
}

