#include <vector>
#include "record_path/gps_data.h" 
#include "Public.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

using namespace std;

#define Rad2Deg 180/3.1415926
#define Deg2Rad 3.1415926/180
#define pi 3.1415926

struct _Point
{
    double x,y;
};

class TPathTrack
{
private:
    _Point base_o;
    char DataFolderPath[255];
        
    bool CheckPoint(_Point p)
    {
        if(fabs(p.x)+fabs(p.y)>10) return true;
        return false;
    }

    ros::NodeHandle nh;
    ros::Subscriber GPSdata_sub; 
    ros::Subscriber PathCtr_sub;
    
    ros::Publisher CurPos_pub;
    ros::Publisher path_points_pub;
    ros::Publisher full_path_pub;
        
    void KeyCtrCallback(const std_msgs::Int32::ConstPtr& msg);
    void GPSDataCallback(const record_path::gps_data::ConstPtr& msg);
    void GetPackagePath(char*path, const char* pkg_name);
    
    void PubCarPos(bool g);
    void Pub_Record_Points();

public:
    _Point cur_pos;
    float angle2north;

    bool record_track_flag;
    vector<_Point> record_track_points;

    nav_msgs::Path record_path;

    TPathTrack();

    void SaveTrack(char* filename);
    void run();
};


