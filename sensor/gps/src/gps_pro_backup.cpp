#include <serial/serial.h> 
#include <NavConversion.h>
#include <common/public.h>
#include <gps/MyGPS_msg.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <fstream>
// #include <smartcar_msgs/Lane.h>
// #include <smartcar_msgs/Waypoint.h> 
// User defined includes
#include <common/display_marker.h>
#include <gpxdata.h>

// #include <dynamic_reconfigure/server.h>
// #include <GPS/gpsparamConfig.h>

using namespace std;

class TGPS_Pro: public Thread
{
private:
    ros::NodeHandle *nh, *nh_local;
    ros::Publisher path_cur_pub, path_load_pub, path_save_pub;// globalpath_pub;
    ros::Publisher marker_pub;
    ros::Publisher map_origin_pub;
    ros::Publisher map_carPose_pub;
    ros::Subscriber gps_base_sub;
    
    vector<geometry_msgs::Point> PathSaveBuf,PathLoadBuf, Path_ll;  
    geometry_msgs::Point utm2map(geometry_msgs::Point utm_p);
    bool saveflag;
    double utmx_zero, utmy_zero;

    string path_filename="/home/bit/gps_data/1.txt";

public:
    int HeartBeat;

    TGPS_Pro();
    //void GPSDataCallback(const gps::MyGPS_msg::ConstPtr& msg);
    void GPSDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void run();
    void SavePath(char* filename);
    void LoadPath(char* filename);

    void SaveGpx(char *filename);
    void LoadGPx(char *filename);

    // void GenFileName(char* filename);
};

TGPS_Pro::TGPS_Pro()
{
    nh = new ros::NodeHandle;  
    nh_local = new ros::NodeHandle("~");
    //path_cur_pub = nh->advertise<nav_msgs::Path>("GPS_Path",10);
    path_load_pub = nh->advertise<nav_msgs::Path>("Path_Load",10);
    path_save_pub = nh->advertise<nav_msgs::Path>("Path_Save",10);
    // globalpath_pub  = nh->advertise<smartcar_msgs::Lane>("global_path",1);
    

    gps_base_sub = nh->subscribe<geometry_msgs::PoseStamped>("/gps_base/UTM_coordinate", 1, &TGPS_Pro::GPSDataCallback,this);

    //gps_base_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &TGPS_Pro::GPSDataCallback,this);
    // gps_base_sub = nh->subscribe<std_msgs::String>("/gps_base/GPS_RawData", 1, &TGPS_Pro::GPSDataCallback,this);
    HeartBeat = 0;
    map_origin_pub = nh->advertise<geometry_msgs::PointStamped>("/map_origin",1);

    map_carPose_pub = nh->advertise<geometry_msgs::PoseStamped>("/map_carPose",1);
    
    marker_pub = nh->advertise<visualization_msgs::Marker>("car_marker", 1);

    saveflag=false;

    start();  
}

geometry_msgs::Point TGPS_Pro::utm2map(geometry_msgs::Point utm_p)
{
    double utmx_zero=0, utmy_zero=0;
    ros::param::get("/gps_base/utmx_zero",utmx_zero);
    ros::param::get("/gps_base/utmy_zero",utmy_zero);

    geometry_msgs::Point map_p;
    //map_p.y=-utm_p.x+utmx_zero,  map_p.x=utm_p.y-utmy_zero;   //原来

    map_p.y=utm_p.y-utmx_zero,  map_p.x=utm_p.x-utmy_zero;   //原来
    return map_p;    
}


void TGPS_Pro::GPSDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    bool setMapZeroFlag=false;
    geometry_msgs::PointStamped map_origin_msg;
    nh_local->getParam("setMapZeroFlag",setMapZeroFlag);
    if(setMapZeroFlag)   //  将当前位置设置为Map零点
    { 
            nh_local->setParam("setMapZeroFlag",false);
            utmx_zero=msg->pose.position.x;
            utmy_zero=msg->pose.position.y;
            nh_local->setParam("utmx_zero",msg->pose.position.x);
            nh_local->setParam("utmy_zero",msg->pose.position.y);

            map_origin_msg.header.seq = 0;
            map_origin_msg.header.stamp = ros::Time(0);    //如果有问题就使用Time(0)获取时间戳，确保类型一致
            map_origin_msg.header.frame_id = "map";
            map_origin_msg.point.x = utmx_zero;
            map_origin_msg.point.y = utmy_zero;
            map_origin_msg.point.z = 0;
            map_origin_pub.publish(map_origin_msg);     //车的起始位置作为map原点
    }
    
    geometry_msgs::Point map_point,utm_point;
    utm_point.x=msg->pose.position.x;    
    utm_point.y=msg->pose.position.y;
    map_point=utm2map(utm_point);
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = map_point.x;
    pose_stamped.pose.position.y = map_point.y;
	//pose_stamped.pose.position.z = msg->Vel;  //速度信息

    // printf("%.5f %.5f %.3f\n", utm_point.x, utm_point.y, msg->Angle);
    // ROS_INFO("%d",msg->Quality);
    //geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw((-msg->Angle+90)/180*3.1416);

    // pose_stamped.pose.orientation = msg->pose.orientation;
    // pose_stamped.header.stamp=ros::Time::now();
    // pose_stamped.header.frame_id="map";
    // //printf("%.5f %.5f %.3f\n",msg->map_x, msg->map_y, msg->Angle);

    // //marker_pub.publish(displayCarPosition(pose_stamped, msg->Quality));

    static tf::TransformBroadcaster br1;
    tf::Transform transform;
    tf::Quaternion quaternion;
    transform.setOrigin(tf::Vector3(map_point.x, map_point.y, 0)); //base_link在map中的位置
    //std::cout<<"map_point_car_x: "<<map_point.x<<" map_point_car_y: "<<map_point.y<<std::endl;
    tf::quaternionMsgToTF(msg->pose.orientation, quaternion);

    transform.setRotation(quaternion);  //base_link在map中的旋转四元数
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    
    geometry_msgs::PoseStamped mapCarPose;
    mapCarPose.header.frame_id="map";
    mapCarPose.header.stamp=ros::Time::now();
    mapCarPose.pose.position.x = map_point.x;
    mapCarPose.pose.position.y = map_point.y;
    mapCarPose.pose.orientation = msg->pose.orientation;
    map_carPose_pub.publish(mapCarPose);

    // transform.setOrigin(tf::Vector3(0.66, 0.0, 0.67));
    // quaternion.setX(0);
    // quaternion.setY(0);    
    // quaternion.setZ(0);    
    // quaternion.setW(1); 
    // transform.setRotation(quaternion);  
    // br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "velo"));


    if(saveflag && PathSaveBuf.size()<100000)
    {
        geometry_msgs::Point p1, p2;
        if(PathSaveBuf.size()>0) p1=*(PathSaveBuf.end()-1); 
        else p1.x=p1.y=p1.z=0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
        PathSaveBuf.push_back(p1);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

        p2.x=map_point.x, p2.y=map_point.y,  p2.z=0;
        if(P2P(p1,p2)>0.1)      //0.3 改为 0.1
        {
            PathSaveBuf.push_back(p2);
            Path_ll.push_back(utm_point);

            nav_msgs::Path path_msg;
            path_msg.header.stamp=ros::Time::now();
            path_msg.header.frame_id= "map";

            for(int i=0;i<PathSaveBuf.size();i++)
            {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = PathSaveBuf[i].x;
                pose_stamped.pose.position.y = PathSaveBuf[i].y;
                
                // ROS_INFO("%d",msg->Quality);

                pose_stamped.header.stamp=ros::Time::now();
                pose_stamped.header.frame_id="map";
                path_msg.poses.push_back(pose_stamped);
            }

            path_save_pub.publish(path_msg);    //path_save的是map坐标
        }
        
    }
}



void TGPS_Pro::SaveGpx(char *filename)
{
    if (PathSaveBuf.size() == 0)  return;

    TGpxData gd;
    gd.trackcount = 1;
    gd.track[0].name = "track";
    for (int i = 0; i < Path_ll.size(); i++)
    {
        gd.track[0].Add(Path_ll[i]);
    }

    gd.Save(filename);
}


//写入文件时候用的是utm坐标
void TGPS_Pro::SavePath(char* filename)
{
    if(PathSaveBuf.size()==0)  return;

    FILE *fp=fopen(filename, "w");

    for(int i=0;i<Path_ll.size();i++)
    {
        fprintf(fp, "%.8f %.8f\n", Path_ll[i].x, Path_ll[i].y);
    }
    fclose(fp);
    //ROS_INFO("I heard: [%d]", record_track_points.size());    
}


//path_load 在map坐标系下
void TGPS_Pro::LoadPath(char *filename)
{
    PathLoadBuf.clear();
    if(strcmp(filename,"clear")!=0)
    {
        std::ifstream file_oxts(filename);
        std::string line = "";
        while (getline(file_oxts, line)) 
        {
            vector<string> ss = split(line, " ");
            geometry_msgs::Point utm_p;

            utm_p.x = atof(ss[0].data());
            utm_p.y = atof(ss[1].data());
            
            //string utm_zone_data;
            //LLtoUTM(Lat,Lon, utm_p.y, utm_p.x, utm_zone_data);  
            
            geometry_msgs::Point map_p=utm2map(utm_p);

            PathLoadBuf.push_back(map_p);
            printf("%.2f %.2f\n",map_p.x,map_p.y);
        }
        printf("there are %d trajectory points.\n", PathLoadBuf.size());
    }    

    try
    {
            if(PathLoadBuf.size()>1)
            {
                //  smartcar_msgs::Lane globalpath;
                nav_msgs::Path path_msg;
                path_msg.header.stamp=ros::Time::now();
                path_msg.header.frame_id="map";
                for(int i=0;i<PathLoadBuf.size() - 1;i++)
                {
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.pose.position.x = PathLoadBuf[i].x;
                    pose_stamped.pose.position.y = PathLoadBuf[i].y;
                    pose_stamped.header.stamp=ros::Time::now();
                    pose_stamped.header.frame_id="map";
                    path_msg.poses.push_back(pose_stamped);

                    //  smartcar_msgs::Waypoint way_point;
                    //  way_point.is_lane = 1;
                    //  way_point.speed_limit = 3.0;
                    //  way_point.pose.pose.position.x= PathLoadBuf[i].x;
                    //  way_point.pose.pose.position.y=PathLoadBuf[i].y;
                    //  way_point.pose.pose.position.z=0;
                    //  double roll=0;double pitch=0;
                    //  double yaw=cast_from_0_to_2PI_Angle(atan2(PathLoadBuf[i + 1].y - PathLoadBuf[i].y,  PathLoadBuf[i + 1].x -  PathLoadBuf[i].x));
                    //  way_point.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(roll,  pitch, yaw);
                    //  globalpath.waypoints.push_back(way_point);
                }
                path_load_pub.publish(path_msg);           //path_load在map坐标下
                // globalpath_pub.publish(globalpath);
            }
            
    }
    catch(out_of_range& e)
    {
        cerr<<e.what()<<endl;
    }
    catch(...)
    {
        printf("Local Path Plan Error!\n");
    }
}

void TGPS_Pro::run()
{
    TTimer hb_tmr;
    int tmp_count=0;
    
    while(1)
    {
        usleep(10000);
        if(hb_tmr.GetValue()>=1)
        {
            HeartBeat=tmp_count/hb_tmr.GetValue();
            hb_tmr.Clear();
            tmp_count=0;
        }

        bool flag=false;
        nh_local->getParam("saveflag",flag);
        if(!saveflag && flag)  
        {
            printf("begin save path!\n");
            saveflag=flag;
            PathSaveBuf.clear();
            Path_ll.clear();
        }
        else if(saveflag && !flag)
        {
            printf("stop save path!\n");
            saveflag=flag;
            char buf[200];
            string fname;
            ros::param::get("/gps_pro/savefilename", fname);
            // cout << "fname=" << fname << endl;
            // cout << "111111111111111111111111111111111111111111111111111111111" <<endl;
            if(fname!="")
            {
                strcpy(buf,fname.c_str());
                SavePath(buf);
            }    
        }

        // nh_local->getParam("loadfilename",pathfilename);

        // if(path_filename!="")
        // {
        //     printf("load path %s\n", path_filename.c_str());
        //     char buf[200];
        //     strcpy(buf,path_filename.c_str());
        //     LoadPath(buf);
        //     path_filename="";
        // }

        string pathfilename="";
        nh_local->getParam("loadfilename",pathfilename);

        if(pathfilename!="")
        {
            printf("load path %s\n", pathfilename.c_str());
            char buf[200];
            strcpy(buf,pathfilename.c_str());
            LoadPath(buf);
            nh_local->setParam("loadfilename","");
        }
                
        ros::spinOnce();
    }
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "GPS_Pro");
	ROS_INFO_STREAM("gps pro start");
	
    TGPS_Pro gps;
    ros::Rate loop_rate(50);
	while (ros::ok())
	{
        ros::spinOnce();
        loop_rate.sleep();

	}
	return 0;
	
}
