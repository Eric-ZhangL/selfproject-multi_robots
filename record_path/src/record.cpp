#include <fstream>
#include <vector>
#include <ros/ros.h>
// #include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <sys/time.h>
#include <nav_msgs/Path.h>
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

ros::Subscriber PathCtr_sub, GPSdata_sub, map_origin_sub;

ros::Publisher full_path_pub, CurPos_pub;

struct _Point
{
    double x,y;
};

_Point cur_pos;
_Point base;
vector<_Point> record_track_points;

double angle2east;//弧度制
bool record_track_flag = false;
char DataFolderPath[255];
geometry_msgs::Quaternion quat;

bool CheckPoint(_Point p)
{
    if(fabs(p.x)+fabs(p.y)>10) 
        return true;
    return 
        false;
}

//获取文件所在路径
void GetPackagePath(char*path, const char* pkg_name)
{
    char cmd[100]="rospack find ";
    strcat(cmd, pkg_name);
    FILE *fp=popen(cmd,"r");
    fgets(path, 256, fp);
    pclose(fp);
}

//返回两点的距离
double P2P_r(_Point p1,_Point p2)
{
    // _Spcs p=P2P(p1,p2);
    // return p.r;
    double a;
    double x=p2.x-p1.x, y=p2.y-p1.y;

    a=x*x+y*y;
    if(fabs(a)<0.000001) 
        return 0;
    else 
        return sqrt(a);
}

void MapOriginCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    base.x = msg->point.x;
    base.y = msg->point.y; 
}


//路径点写入文件
void SaveTrack(char* filename)
{
    //if(record_track_points.size()==0)  return;

    FILE *fp=fopen(filename, "w");
    for(int i=0;i<record_track_points.size();i++)
    {
        fprintf(fp, "%.2f %.2f\n", record_track_points.at(i).x, record_track_points.at(i).y);
    }
    fclose(fp);
}

void Pub_Record_Path()
{
    /*visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "track_points";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.points.clear();*/

    nav_msgs::Path viapoints;
    viapoints.header.frame_id="map";
    viapoints.poses.clear();

    for (int i = 0; i < record_track_points.size(); i++)
    {
  //       geometry_msgs::Point p;
  //       p.x = record_track_points.at(i).x-base.x;
		// p.y = record_track_points.at(i).y-base.y;
		// marker.points.push_back(p);

        geometry_msgs::PoseStamped path_point;
        path_point.pose.position.x = record_track_points.at(i).x - base.x;
        path_point.pose.position.y = record_track_points.at(i).y - base.y;
        viapoints.poses.push_back(path_point);
    }
    // path_points_pub.publish(marker);
    full_path_pub.publish(viapoints);//用于rviz显示录制的路径，不存储
}

//以marker代替小车，发布小车位置
void PubCarPos()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "car_pos";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
   
    marker.pose.position.x = cur_pos.x-base.x;
    marker.pose.position.y = cur_pos.y-base.y;
    marker.pose.position.z = 0;

    // geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(angle2east);
    marker.pose.orientation.x = quat.x;
    marker.pose.orientation.y = quat.y;
    marker.pose.orientation.z = quat.z;
    marker.pose.orientation.w = quat.w;

    marker.scale.x = 1;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // if(g) 
    //     marker.color.g=1.0f,  marker.color.r = 0.0f;
    // else 
    marker.color.g = 0.0f,  marker.color.r = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    CurPos_pub.publish(marker);
}

void KeyCtrCallback(const std_msgs::Int32::ConstPtr& msg)
{
    if(msg->data==1 && !record_track_flag)
    {
        record_track_flag=true;
        record_track_points.clear();
        ROS_INFO("begin path record!");
    }
    else if(record_track_flag && msg->data>='1' && msg->data<='9')
    {
        record_track_flag=false;
        char filename[100];
        sprintf(filename,"%s/%c.txt",DataFolderPath, msg->data);
        SaveTrack(filename);
        ROS_INFO("Stop recording! path save to %s!", filename);
    }
 
}

//获得当前小车位置
void GPSDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    if(!CheckPoint(base))  
        base = {msg->pose.position.x, msg->pose.position.y};

    cur_pos.x = msg->pose.position.x;
    cur_pos.y = msg->pose.position.y;
    quat = msg->pose.orientation;

    // double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
    // tf::Matrix3x3(msg->pose.orientation).getRPY(roll, pitch, yaw); //进行转换
    // angle2east = yaw;

    PubCarPos();
}

void run()
{    
    if(record_track_flag && record_track_points.size()<10000)    //记录轨迹数据
    {
        if(record_track_points.empty())  
            record_track_points.push_back(cur_pos);
        else if(P2P_r(cur_pos,record_track_points.back())>1.0)//即1m存入一个点
        {
            record_track_points.push_back(cur_pos);
        }
    }
    Pub_Record_Path();
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "record_path");

    ros::NodeHandle pn("~");
    string pkg_name,folder_name;
    pn.param<string>("pkg_name", pkg_name, "");
    pn.param<string>("folder_name", folder_name, "");

    GetPackagePath(DataFolderPath, pkg_name.c_str());
    DataFolderPath[strlen(DataFolderPath)-1]=0;
    sprintf(DataFolderPath,"%s/%s",DataFolderPath,folder_name.c_str());

    ros::NodeHandle nh;
 
    PathCtr_sub = nh.subscribe("key_record", 1, &KeyCtrCallback);
    GPSdata_sub = nh.subscribe("/gps_base/UTM_coordinate", 1, &GPSDataCallback);//获取小车当前GPS位置
    map_origin_sub = nh.subscribe("map_origin",10, &MapOriginCallback);//启动路径规划时，采用此作为map系原点的utm坐标
    // path_points_pub = nh.advertise<visualization_msgs::Marker> ("path_points", 1);//用于rviz显示录制的路径点
    full_path_pub = nh.advertise<nav_msgs::Path>("/record_path/path",1);//用于rviz显示录制的路径
    CurPos_pub = nh.advertise<visualization_msgs::Marker>("/record_path/car_marker",1);//车的map系坐标，并可以在rviz显示

    


    ros::Rate loop_rate(50);
    while (ros::ok())
	{
        run();
        ros::spinOnce();
        loop_rate.sleep();
        //ROS_INFO("OK");
	}

	return 0;
}
