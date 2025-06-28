#include <serial/serial.h> 
#include "ros/ros.h"
#include <NavConversion.h>
#include <common/public.h>
#include <gps/MyGPS_msg.h>
#include <std_msgs/Float64MultiArray.h>
#include <common/pudp.h>
#include <queue>
#include <cstring>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

using namespace std;

class TGPSData
{
private:
    int  sock;
    string utm_zone_data;
    TTimer tmr_speed;
    int Lat_state, Lon_state, Ang_state;
   

public:
    double Lat_raw, Lon_raw;  //  原始值
    double Lat, Lon, Angle;    // 转换值   纬度  经度  角度
    double UTM_Y, UTM_X;   //  当前坐标点
    double Pre_Y, Pre_X;   //  上一次坐标点
    double PreY,PreX;
    float speed;   //  移动速度  m/s
    float Hight;   //高程

    int count = 0;
    TTimer tmr;

    // TDataFilter *df_X, *df_Y, *df_Ang;   //对数据进行均值滤波处理
    int Quality_factor;
    int ReadyState;
    int Resolve(const char *buf);
   

    TGPSData()
    {
        Lat=Lon=Angle=0;
        Lat_raw=Lon_raw=0;
        Quality_factor=0;
        PreX=PreY=0;

        ClearState();
    };

    void ClearState()
    {
        ReadyState=0;
        Lat_state=Lon_state=Ang_state=0;
    }
};


class TGPS_Base
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher  gps_pub, gps_raw_pub,map_origin_pub ;
    ros::Subscriber gps_sim_sub;
    serial::Serial *ser;
    PUDP *udp;

    double utmx_zero, utmy_zero;//零点位置，车启动瞬间，标记为零点位置
    
public:
    // int HeartBeat;
    TGPSData gdata;
    
    bool simmode=false;
    
	TGPS_Base();
    //void GPSSimDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);  // 处理原始GPS数据
    int DataProc(const char *buf);    
    //void simdata();
    void run();
};


int TGPSData::Resolve(const char *buf)
{
    if(tmr.GetValue() >= 1.0)
    {
        // printf("count=%d\n",count);
        count = 0;

        tmr.Clear();
    }
    //vector<string> ss = split(buf, "$");
    Ang_state=0;
    vector<string> str = split(buf, ",");
    if ((str[0] == "$GPGGA")||(str[0] == "$GNGGA"))
    {
        count++;
        if (str[2] != "") Lat_raw = atof(str[2].data()), Lat_state = 1;
        else Lat_state = 0;
        if (str[4] != "") Lon_raw = atof(str[4].data()), Lon_state = 1;
        else Lon_state = 0;
        if (str[3] == "S") Lat_raw = -Lat_raw;
        if (str[5] == "W") Lon_raw = -Lon_raw;

        if (str[6] != "") Quality_factor = atoi(str[6].c_str());
        else Quality_factor = 0;
     
        LatLonConvert(Lat_raw,Lon_raw, &Lat, &Lon);
        LLtoUTM(Lat,Lon, UTM_Y, UTM_X, utm_zone_data);  //  Y 北向  X 正东

        if(tmr_speed.GetValue()>0.3)  
        {
            float d=P2P(Pre_X, Pre_Y, UTM_X, UTM_Y);
            Pre_X=UTM_X, Pre_Y=UTM_Y;
            speed=d/tmr_speed.GetValue();
            tmr_speed.Clear();
        }
    }
    else if ((str[0] == "$GPHDT")||(str[0] == "$GNHDT"))
    {
        if (str[1] != "")
        {
            Angle = atof(str[1].c_str());
            // Angle = 180 - Angle;   
            Ang_state = 1;
        }
        else Ang_state = 0;
    }

    // printf("ABC=%s\n",buf);

    return Ang_state;
}


TGPS_Base::TGPS_Base()
{
    nh=new ros::NodeHandle;
    nh_local=new ros::NodeHandle("~");  //  局部空间
    gps_pub = nh_local->advertise<gps::MyGPS_msg>("GPS_Base",10);
    gps_raw_pub = nh_local->advertise<std_msgs::String>("GPSRaw_Base",10);

    map_origin_pub = nh_local->advertise<geometry_msgs::PointStamped>("map_origin",1); //map系原点的utm坐标
    
    string gpsBuf;

    //gps_sim_sub = nh->subscribe<geometry_msgs::PoseStamped>("/gps_sim_pos", 10, &TGPS_Base::GPSSimDataCallback,this);
    

    int tmp_count=0;
    TTimer gps_tmr;

    utmx_zero=utmy_zero=0;
    nh_local->getParam("utmx_zero",utmx_zero);
    nh_local->getParam("utmy_zero",utmy_zero);
    string m="";
    nh_local->getParam("mode",m);
    ser=NULL;  udp=NULL;

    nh_local->getParam("simmode",simmode);

    if(m=="UDP")
    {
        string ip="192.168.3.5";
        int port=1234;
        nh_local->getParam("IP",ip);
        nh_local->getParam("port",port);
        udp = new PUDP(1234);
        udp->AddRemoteIP(ip, port);

        udp->Send("OK");    //must have
    }
    else if(m=="SER")
    {
        string devname="";
        int baud=0;
        nh_local->getParam("devname",devname);
        nh_local->getParam("baud",baud);
        ROS_INFO("%s %d",devname.c_str(), baud);
        if(devname!="" && baud>0) 
        {
            ser=new serial::Serial;
            ser->setPort(devname); 
            ser->setBaudrate(baud); 
            serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
            ser->setTimeout(to); 
            ser->open(); 
        }
    }
}

int TGPS_Base::DataProc(const char *buf)
{
    int res=gdata.Resolve(buf);
    // printf("res=%d buf=%s\n",res,buf);
    geometry_msgs::PointStamped map_origin_msg;
    std_msgs::String strmsg;
    strmsg.data=buf;
    gps_raw_pub.publish(strmsg);

    if(res)   //  必须有角度
    {
        gdata.ClearState();

        bool setMapZeroFlag=false;
        nh_local->getParam("setMapZeroFlag",setMapZeroFlag);
        if(setMapZeroFlag)   //将当前位置设置为Map零点
        {
            
            nh_local->setParam("setMapZeroFlag",false);
            utmx_zero=gdata.UTM_X;
            utmy_zero=gdata.UTM_Y;
            //将gdata.UTM_X; UTM_Y; 设置给参数  utmx_zero   utmy_zero
            nh_local->setParam("utmx_zero",gdata.UTM_X);
            nh_local->setParam("utmy_zero",gdata.UTM_Y);

            map_origin_msg.header.seq = 0;
            map_origin_msg.header.stamp = ros::Time::now();    //如果有问题就使用Time(0)获取时间戳，确保类型一致
            map_origin_msg.header.frame_id = "map";
            map_origin_msg.point.x = utmx_zero;
            map_origin_msg.point.y = utmy_zero;
            map_origin_msg.point.z = 0;
            map_origin_pub.publish(map_origin_msg);     //车的起始位置作为map原点
        }
                        
        gps::MyGPS_msg msg;
        msg.header.frame_id="map";
        msg.header.stamp=ros::Time::now();

        msg.Angle=gdata.Angle; 
        msg.Lat=gdata.Lat,  msg.Lon=gdata.Lon;
        msg.UTM_Y=gdata.UTM_Y,  msg.UTM_X=gdata.UTM_X;
        // msg.Angle=-gdata.Angle+90;  //180-gdata.Angle+90.0;
        // msg.UTM_Y=-gdata.UTM_X,  msg.UTM_X=gdata.UTM_Y;
        // msg.map_y=-gdata.UTM_X+utmx_zero,  msg.map_x=gdata.UTM_Y-utmy_zero;

        msg.Vel=gdata.speed;
        msg.Quality=gdata.Quality_factor;
        
        gps_pub.publish(msg);
        //  printf("%.2f\n",msg.Vel);
    }    

    return res;
}

// void TGPS_Base::simdata()
// {
//     static TTimer tmr;
//     geometry_msgs::Point p;
//     p.x=6*sin(6.28*0.05*tmr.GetValue());
//     p.y=6*cos(6.28*0.05*tmr.GetValue());
    
//     gps::MyGPS_msg msg;
//     msg.header.frame_id="map";
//     msg.header.stamp=ros::Time::now();

//     msg.Lat=msg.Lon=1,  msg.Angle=-10;
//     msg.UTM_X=p.x;
//     msg.UTM_Y=p.y;
//     msg.map_x=msg.map_y=0;
//     msg.Vel=0;
//     msg.Quality=4;
//     gps_pub.publish(msg);
    
// }

// void TGPS_Base::GPSSimDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     if(!simmode) return;
    
//     //ROS_INFO("%.2f",msg->data[2]);
//     gps::MyGPS_msg gps_msg;
//     gps_msg.header.frame_id="map";
//     gps_msg.header.stamp=ros::Time::now();
//     gps_msg.map_x=msg->pose.position.x;
//     gps_msg.map_y=msg->pose.position.y;
//     gps_msg.Angle=tf::getYaw(msg->pose.orientation)*180/3.1416;
//     // printf("%.3f\n",angle);

//     gps_msg.Quality=4;
//     gps_pub.publish(gps_msg);
// }

void TGPS_Base::run()
{
    static string gpsBuf;
    static TTimer gps_tmr;

    char msg[1024];
    int n=0;

    // simdata();

    if(udp!=NULL)
    {
        if (gps_tmr.GetValue() > 1)
        {
            udp->Send("OK");    //must have
            gps_tmr.Clear();
        }

        n = udp->Recv();
        if(n>0) strcpy(msg, udp->rec_buf);
    }
    else if(ser!=NULL) 
    {
        n=ser->available();
        n=ser->read((unsigned char *)msg,n);
    }

    for (int i = 0; i < n; ++i)
    {
        if (msg[i]!= '\n')  gpsBuf.push_back(msg[i]);
        else
        {
            gpsBuf.push_back('\n');
            // cout<<gpsBuf<<endl;

            DataProc(gpsBuf.c_str());
            gpsBuf.clear();
        }

    }

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gps_base");
    ROS_INFO_STREAM("gps base start by wsk");
	TGPS_Base gps;
  
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
        //printf("A\n");
        gps.run();
	    ros::spinOnce();
        loop_rate.sleep();
	}
	return 0;
}
