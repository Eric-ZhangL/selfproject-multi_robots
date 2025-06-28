#ifndef _PP_FOLLOWER_H_
#define _PP_FOLLOWER_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>


#include <visualization_msgs/Marker.h>
#include <common/display_marker.h>


#include <time.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <ctime>
#include <string>

class PurePursuit
{
private:
    ros::Subscriber trajectory_sub_;
    ros::Subscriber car_pose_sub_, plan_vel_sub_;
    ros::Publisher control_cmd_pub_;

    ros::Publisher pre_point_pub;


    nav_msgs::Path path_;
    geometry_msgs::PoseStamped car_pose_;
    int idx_;
    bool goal_reached_ ;
    bool get_path_flag_;
    double ld_;
    double pos_tol_;
    double v_max_, v_, w_max_,wheel_base;
    
public:
    ros::NodeHandle nh;
    tf::TransformListener *tf_;
    PurePursuit();
    ~PurePursuit();
    void TrajectoryCallback(const nav_msgs::Path& msg);
    void mapCarPoseCallback(const geometry_msgs::PoseStamped& msg);
    void PlanVelCallback(const geometry_msgs::Twist& msg);
    double GetDistance(cv::Point2d point_1,cv::Point2d point_2);
};



#endif