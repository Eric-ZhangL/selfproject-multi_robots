#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "record_path/gps_data.h" 
#include "geometry_msgs/Point.h"

geometry_msgs::Point initpos, carpos;
ros::Publisher initpos_pub, carpos_pub;
 
void GPSCallback(const record_path::gps_data::ConstPtr msg)
{
    if(fabs(initpos.x+initpos.y+initpos.z)<10)
    {
        initpos.x=msg->utm_x;
        initpos.y=msg->utm_y;
    }
    initpos_pub.publish(initpos);

    carpos.x=msg->utm_x-initpos.x;
    carpos.y=msg->utm_y-initpos.y;
    carpos_pub.publish(carpos);


    // tf广播器
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    
    double x=msg->utm_x-initpos.x;
    double y=msg->utm_y-initpos.y;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    float angle=msg->angle2north+90;
    q.setRPY(0, 0, angle/180*3.14);
    transform.setRotation(q);
 
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
    
}


int main(int argc, char** argv)
{
    initpos.x=initpos.y=initpos.z=0;
    
    // 初始化节点
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle node;
    ros::Subscriber gps_sub = node.subscribe("GPS_Data", 10, &GPSCallback);
    initpos_pub = node.advertise<geometry_msgs::Point> ("initpos_point", 1);
    carpos_pub = node.advertise<geometry_msgs::Point> ("carpos_point", 1);

	ros::Rate rate(100);
	while (node.ok())
    {
   	    ros::spinOnce(); 
		rate.sleep();
	}

    return 0;
};


