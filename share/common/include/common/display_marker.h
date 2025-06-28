// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>


// C++ includes
#include <vector>
#include <string>

//const std::string MAP_FRAME = "map";
using namespace std;

visualization_msgs::Marker displayCarPosition(geometry_msgs::PoseStamped pose_stamped, int Quality)
{

  //marker.type = visualization_msgs::Marker::SPHERE;
  //marker.action = visualization_msgs::Marker::ADD;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    //marker.header.stamp = ros::Time();   //用哪个？
    marker.ns = "car_position_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0f;

    marker.frame_locked = true;
    marker.lifetime = ros::Duration();
    // marker.pose.position = position;
    marker.pose.position.x = pose_stamped.pose.position.x;
    marker.pose.position.y = pose_stamped.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation= pose_stamped.pose.orientation;

    if(Quality == 4)
    {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
    }
    else
    {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
    }
    return marker;
}


visualization_msgs::Marker displayAimPoint(geometry_msgs::PoseStamped pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "aim_point_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.pose.position = pose.pose.position; //斟酌一下
    return marker;
}

visualization_msgs::Marker GetVisualMarker(geometry_msgs::Point p, std_msgs::ColorRGBA color)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.lifetime = ros::Duration();
    marker.pose.position = p;
    marker.color=color;
    marker.color.a = 1.0f;
    // marker.color.r = 0.0f;
    // marker.color.g = 1.0f;
    // marker.color.b = 0.0f;
    return marker;
}

nav_msgs::Path GetVisualPath(vector<geometry_msgs::Point> path_buf)
{
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="map";
    for(int i=0;i<path_buf.size();i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position=path_buf[i];
        pose.header.stamp=ros::Time::now();
        pose.header.frame_id="map";
        path.poses.push_back(pose);
    }
    return path;
}

std_msgs::ColorRGBA GetColor(float r,float g,float b)
{
    std_msgs::ColorRGBA color;
    color.r=r, color.g=g, color.b=b;
    return color;
}




