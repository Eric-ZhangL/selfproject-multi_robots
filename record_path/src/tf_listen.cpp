#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PointStamped point, map_point;
tf::TransformListener *listener;

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    map_point.header.frame_id = "map";
	
	map_point.header.stamp = ros::Time();
	map_point.point.x=msg->pose.position.x;
	map_point.point.y=msg->pose.position.y;
	map_point.point.z=msg->pose.position.z;

	//printf("%.2f %.2f\n",map_point.point.x,map_point.point.y);
    //tf::TransformListener listener(ros::Duration(10));
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "tf_listen");

	ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("my_pose", 10);
	ros::Subscriber localtarget_sub = n.subscribe("/move_base_simple/goal", 1, &GoalCallback);

	//tf::TransformListener listener(ros::Duration(10));
	listener=new tf::TransformListener(ros::Duration(10));

	ros::Rate loop_rate(10);

	

    // geometry_msgs::PointStamped point, map_point;
	// point.header.frame_id = "base_link";
	
	// point.header.stamp = ros::Time();
	// point.point.x = 1.0;
	// point.point.y = 0.0;
	// point.point.z = 0.0;

	while (ros::ok())
	{
        geometry_msgs::PoseStamped msg;
		msg.header.stamp=ros::Time();
		msg.header.frame_id="base_link";
		msg.pose.position.x=1;
		msg.pose.position.y=0;
		msg.pose.position.z=0;
		pub.publish(msg);



  	    // try
		// {
		// 	listener->transformPoint("base_link", map_point, point);
		// 	ROS_INFO("map(%.2f, %.2f) --> base_link(%.2f, %.2f)", map_point.point.x, map_point.point.y, point.point.x, point.point.y);
		// }catch(tf::TransformException& ex)
		// {
		// 	ROS_ERROR("Received an exception");// trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
		// }
		  
		  
		

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}