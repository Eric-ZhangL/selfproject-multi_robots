// #include "Public.h"
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <fstream>

#include <tf/tf.h>

// #include "ugv_bit/motor_ctr.h"
// #include "ugv_bit/gps_data.h"

#include <geometry_msgs/Twist.h>
// #include <geometry_msgs.msg/Quaternion.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float32.h>

// #include <autoware_msgs/ControlCommandStamped.h>
#include <nav_msgs/Path.h>

using namespace std;


geometry_msgs::Twist gazebo_control;

// ugv_bit::gps_data car_pose;

ros::Publisher gazebo_car_control_pub,planner_GPS_pub,car_velocity_pub,real_car_tra;
ros::Subscriber map_origin_sub,gazebo_car_pose_sub,control_sub;

double base_x=0.0,base_y=0.0;
double steer_rad;
vector<geometry_msgs::Point> Path_real;


void MapOriginCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    base_x = msg->point.x;
    base_y = msg->point.y;
}


//获取小车的位置信息（GPS） 、速度信息并发布出去
void CarPoseCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // std::cout<<"111112222222"<<std::endl;
    geometry_msgs::PoseStamped car_pose;

    std::vector<std::string> model_names = msg->name;

    //只存model_base
    for(size_t i = 0; i < model_names.size(); i++)
    {
        if(model_names[i] == "mobile_base")//获取对应模型信息
        {
            car_pose.header.frame_id="base_link";
            car_pose.header.stamp=ros::Time::now();
            car_pose.pose.position.x = msg->pose[i].position.x + base_x;
            car_pose.pose.position.y = msg->pose[i].position.y + base_y;

            car_pose.pose.orientation = msg->pose[i].orientation;

            /*tf::quaternionTFToMsg(msg->pose[i].orientation,utm_coordinate.pose.orientation);

            tf::Quaternion quat; //定义一个四元数
            tf::quaternionMsgToTF(msg->pose[i].orientation,quat); //取出方向存储于四元数
            double roll,pitch,yaw; //定义存储r\p\y的容器
            tf::Matrix3x3(quat).getRPY(roll,pitch,yaw); //进行转换
            car_pose.pose.orientation.z = yaw;

tf::Quaternion q;
        geometry_msgs::PoseStamped utm_coordinate;
        double Heading_angle;
        if(msg.Angle <= 90.0)
        {
            Heading_angle = 90.0 - msg.Angle;
        }
        else
        {
            Heading_angle = 360.0 - msg.Angle + 90.0;
        }
        q.setRPY(0, 0, Heading_angle/180.0*3.1416); 
        // current_pose.pose.orientation = q.normalized();
        q.normalized();
        tf::quaternionTFToMsg(q,utm_coordinate.pose.orientation);*/


            //小车的速度和转向信息
            geometry_msgs::TwistStamped vehicle_status;
            //速度信息
            vehicle_status.twist.linear.x = sqrt(pow(msg->twist[i].linear.x,2)+pow(msg->twist[i].linear.y,2));
            //转向好像没用上
            vehicle_status.twist.angular.z = steer_rad;//规划模组需要的弧度制数据，以左转为正

            planner_GPS_pub.publish(car_pose);
            car_velocity_pub.publish(vehicle_status);

            break;
        }
    }
    
/**********************publish really car traj***************/
    

}

//接收规划的控制信息，转为gazebo仿真所需的信息，然后发送转换后的控制信息给小车
void CarControlCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Twist gazebo_control;
    gazebo_control.angular.z = msg->angular.z;//控制量为弧度，转为四轮转向模型
    // steer_rad = msg->cmd.steering_angle;//规划模组需要的弧度制数据，以左转为正
    gazebo_control.linear.x = msg->linear.x;//m/s
    
    

    gazebo_car_control_pub.publish(gazebo_control);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pathtrack");
    ros::NodeHandle nh;

    map_origin_sub = nh.subscribe("/map_origin",1,&MapOriginCallback);
    control_sub = nh.subscribe("/car_cmd", 1 ,&CarControlCallback);//接受规划的控制信息，然后转换为gazebo仿真所需的信息
    gazebo_car_control_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/cmd_vel",1);//发送转换后的控制信息给gazebo小车
    gazebo_car_pose_sub = nh.subscribe("/gazebo/model_states",10, &CarPoseCallback);//gazebo小车的状态
    
    
    planner_GPS_pub = nh.advertise<geometry_msgs::PoseStamped>("/gps_base/UTM_coordinate",10);//将gazebo状态转换为GPS后发出
    //发布速度信息 似乎没有转向信息
    car_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity",10);//发布小车的速度和转向信息

    ros::Rate loop_rate(20);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        
    }

    return 0;
}
