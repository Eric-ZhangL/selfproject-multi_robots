#ifndef GPXDATA_H_
#define GPXDATA_H_

#include "tinyxml.h"
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <geometry_msgs/Point.h>


using namespace std;

#define MaxPointCount 10000
#define MaxTrackCount 10000

// struct _GpxPoint   // 轨迹点
// {
//     double lon,lat;
// };

class TGpxTrack    //  轨迹类
{
public:

    double xsmall,xbig,ysmall,ybig;
    vector<geometry_msgs::Point> track_points;//存放转换后的点
    vector<geometry_msgs::Point> points;
    string name;

    TGpxTrack()
    {
        Clear();
        //points.clear();
        name="";
    };

    void Add(geometry_msgs::Point p)
    {
        if(points.size()>=MaxPointCount)   return;

        points.push_back(p);

        //point[count]=p;
        //count++;
    };

    void Clear(void)
    {
        //count=0;
        points.clear();
    };
};

class TGpxData    // gpx类，包含多轨迹
{
public:
    TGpxTrack track[MaxTrackCount];
    int trackcount;

    TGpxData();
    void SaveTrack(TiXmlElement *xmlElement0, TGpxTrack pgt);
    void Save(char* filename);

    void LoadTrack(TiXmlElement* e0, TGpxTrack* pgt);
    void Load(char* filename);    
};

#endif

