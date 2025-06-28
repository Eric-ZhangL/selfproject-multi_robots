// #include "Public.h"
#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <fstream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// #include "ugv_bit/motor_ctr.h"
// #include "ugv_bit/gps_data.h"

#include <geometry_msgs/Twist.h>
// #include <geometry_msgs.msg/Quaternion.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/DTLane.h>
#include <autoware_msgs/ControlCommandStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <cmath>
#include <deque>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

// #include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <gps/MyGPS_msg.h>

#include <common/public.h>

//#include "op_planner/PlannerH.h"

//#include "op_planner/PlanningHelpers.h"

using namespace std;

#define PI acos(-1) 

// struct _Point
// {
//     double x,y;
// };

geometry_msgs::Twist gazebo_control;

// ugv_bit::gps_data car_pose;


ros::Publisher current_pose_pub,initial_pose_pub,vehicle_status_pub,autoware_GlobalPlannerPaths_pub,gazebo_car_control_pub,PointCloud_pub;
ros::Publisher cmd_pub, real_car_tra;
ros::Publisher autoware_latticePaths_pub;
ros::Subscriber map_origin_sub,planner_car_control_sub,GPS_data_sub,GloblePath_sub,control_sub,lidar_sub;


bool control_msg_received = false;
geometry_msgs::TwistStamped vehicle_status;


double base_x,base_y;
double car_width,car_length;

autoware_msgs::LaneArray laneArray_pub;
string lidar_frame;
double obs_range;
TTimer tmr_speed;

tf::TransformListener *tf_;

double *quadratic_fitting(int N, deque<_Point> *reference)//N为拟合的次数
{
    double *temp = new double[N+1];

    // 创建A矩阵
    Eigen::MatrixXd A(reference->size(), N + 1);
    
    for (unsigned int i = 0; i < reference->size(); ++i) 
    {  // 遍历所有点
        
        for (int n = N, dex = 0; n >= 1; --n, ++dex) 
        {  // 遍历N到1阶
            A(i, dex) = pow(reference->at(i).x, n);
        }
        
        A(i, N) = 1;  //
    }
    
    // 创建B矩阵
    Eigen::MatrixXd B((*reference).size(), 1);
    
    for (unsigned int i = 0; i < (*reference).size(); ++i) 
    {
        B(i, 0) = reference->at(i).y;
    }
    
    // 创建矩阵W
    Eigen::MatrixXd W;
    W = (A.transpose() * A).inverse() * A.transpose() * B;

    // quadratic_function out;

    for(unsigned int i = 0; i<W.rows(); i++)
    {
        temp[i] = W(i,0);
    }

    return temp;//返回系数
}

//判断是求哪个点的导数
int label_determined(int current, int max)
{
    if(current < 2)
    {
        return current;
    }
    else if(max - current < 2)
    {
        return (max - current);
    }
    else
        return 2;
}

//解方程组
/*void *Doolittle(int n,vector<double> *A,vector<double> *b)//n为阶数 A为系数矩阵（依次为第一行、第二行...） b为常数矩阵
{
    double *temp = new double[n];
    double *L = new double[n*n];//开辟L矩阵空间
    double *U = new double[n*n];//开辟U矩阵空间
    double *y = new double[n];//开辟y矩阵空间
    // double *x = new double[n];
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            *(U + i*n + j) = 0;//暂时全部赋值为0
            if (i==j)
            {
                *(L + i*n + j) = 1;//对角线赋值为1
            }
            else
            {
                *(L + i*n + j) = 0;//其他暂时赋值为0
            }
        }
    }
    for (int k = 0; k < n; k++)//计算u和l矩阵的值
    {
        for (int j = k; j < n; j++)
        {
            *(U + k*n + j) = A.at(k*n + j);//*(A + k*n + j);//第一行
            for (int r = 0; r < k; r++)//接下来由L的前一列算u的下一行
            {
                *(U + k*n + j) = *(U + k*n + j) - (*(L + k*n + r)*(*(U + r*n + j)));
            }
        }
        for (int i = k+1; i < n; i++)//计算L的列
        {
            *(L + i*n + k) = A.at(k*n + j);//*(A + i*n + k);
            for (int r = 0; r < k; r++)
            {
                *(L + i*n + k) = *(L + i*n + k) - (*(L + i*n + r)*(*(U + r*n + k)));
            }
            *(L + i*n + k) = *(L + i*n + k) / (*(U + k*n + k));
        }
    }
    for (int i = 0; i < n; i++)//由Ly=b算y
    {
        *(y + i) = b.at(i);//*(b + i);
        for (int j = 0; j < i; j++)
        {
            *(y + i) = *(y + i) - *(L + i*n + j)*(*(y + j));

        }
    }
    for (int i = n-1; i >= 0; i--)//由Ux=y算x
    {
        *(x + i) = *(y + i);
        for (int j = i+1; j < n; j++)
        {
            *(y + i) = *(y + i) - *(U + i*n + j)*(*(x + j));
        }
        *(x + i) = *(y + i) / (*(U + i*n + i));
    }
    // cout << "解：\n";//得出解
    for (int i = 0; i < n; i++)
    {
        temp[i] = *(x + i);
        // cout <<"x"<<i+1<<"："<< *(x + i) << endl;
    }
    delete[]L;//释放空间
    delete[]U;
    delete[]y;
    return temp;//返回系数
}*/

//计算路径点的方向，相对于z轴旋转的弧度
double angle_calculation(int label,int N,deque<_Point> *reference)//拟合之后，对第label个点求导；N次拟合
{
    double *coefficient;
    // solved_coefficient = Doolittle(4, &coefficient_matrix, &constant_matrix);
    coefficient = quadratic_fitting(N, reference);
    double slope = 0.0;
    double yaw_out;

    for(int i=0;i<N;i++)
    {
        slope += (N-i)*(*(coefficient+i))*pow(reference->at(label).x, N-1-i);
    }
    delete coefficient;

    if(label == 0 || label == N)
    {
        if(slope >= 0)
        {
            if(slope > 1)
            {
                if(reference->at(N).y - reference->at(0).y > 0)
                    yaw_out = atan(slope);
                else
                    yaw_out = -PI + atan(slope);
            }
            else
            {
                if(reference->at(N).x - reference->at(0).x > 0)
                    yaw_out = atan(slope);
                else
                    yaw_out = -PI/2 + atan(slope);
            }
        }
        else
        {
            if(slope < -1)
            {
                if(reference->at(N).y - reference->at(0).y > 0)
                    yaw_out = -PI - atan(-slope);
                else
                    yaw_out = - atan(-slope);
            }
            else
            {
                if(reference->at(N).x - reference->at(0).x < 0)
                    yaw_out = -PI - atan(-slope);
                else
                    yaw_out = - atan(-slope);
            }
        }
    }
    else
    {
        if(slope >= 0)
        {
            if(slope > 1)
            {
                if(reference->at(label+1).y - reference->at(label-1).y > 0)
                    yaw_out = atan(slope);
                else
                    yaw_out = -PI + atan(slope);
            }
            else
            {
                if(reference->at(label+1).x - reference->at(label-1).x > 0)
                    yaw_out = atan(slope);
                else
                    yaw_out = -PI/2 + atan(slope);
            }
        }
        else
        {
            if(slope < -1)
            {
                if(reference->at(label+1).y - reference->at(label-1).y > 0)
                    yaw_out = -PI - atan(-slope);
                else
                    yaw_out = - atan(-slope);
            }
            else
            {
                if(reference->at(label+1).x - reference->at(label-1).x < 0)
                    yaw_out = -PI - atan(-slope);
                else
                    yaw_out = - atan(-slope);
            }
        }
    }
    return yaw_out;
}

//曲率半径
double radius_of_curvature(deque<_Point> *reference)
{
    double curvity;
    _Point P1,P2,P3;
    P1 = reference->at(0);
    P2 = reference->at(1);
    P3 = reference->at(2);
    //计算曲率部分，使用正弦定理 a/sinA = 2R
    if(reference->at(0).x == reference->at(1).x && reference->at(1).x == reference->at(2).x)//三点横坐标相同，即共线，直接标记曲率为0
    {
        curvity = 0;
    }
    else
    {
        double dis1,dis2,dis3;
        double cosA,sinA,dis;
        dis1 = sqrt((P1.x - P2.x)*(P1.x - P2.x) + (P1.y - P2.y)*(P1.y - P2.y));
        dis2 = sqrt((P1.x - P3.x)*(P1.x - P3.x) + (P1.y - P3.y)*(P1.y - P3.y));
        dis3 = sqrt((P2.x - P3.x)*(P2.x - P3.x) + (P2.y - P3.y)*(P2.y - P3.y));
        dis = dis1*dis1 + dis3*dis3 - dis2*dis2;
        cosA = dis/(2*dis1*dis3);//余弦定理求角度
        sinA = sqrt(1 - cosA*cosA);//求正弦
        curvity = 0.5*dis2/sinA;//正弦定理求外接圆半径
        // curvature = 1/curvity;//半径的倒数是曲率，半径越小曲率越大
    }
    return curvity;
}

//计算B相对于A的方位角（A点正北方向到达B点所需要旋转的角度）
double azimuth_calculation(geometry_msgs::Point A, geometry_msgs::Point B)
{
    double diff_x,diff_y;
    double out_data;

    diff_x = B.x - A.x;
    diff_y = B.y - A.y;

    if(diff_x < 0.00001 && diff_x > -0.00001)
    {
        if(diff_y > 0)
        {
            out_data = 0;
        }
        else
        {
            out_data = PI;
        }
    }
    else if(diff_x > 0)
    {
        if(diff_y > 0)
        {
            out_data = atan(diff_x/diff_y);
        }
        else
        {
            out_data = PI/2 + atan(-diff_y/diff_x);
        }
    }
    else if(diff_x < 0)
    {
        if(diff_y < 0)
        {
            out_data = PI + atan(-diff_x/-diff_y);
        }
        else
        {
            out_data = PI*3/2 + atan(diff_y/-diff_x);
        }
    }
    return out_data;    
}

void MapOriginCallback(const geometry_msgs::PointStamped& msg)
{
    base_x = msg.point.x;
    base_y = msg.point.y;
    // initial_pose_pub.publish(msg);

    /*static tf::TransformBroadcaster br;
    tf::Transform transform;
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;
    // transform.setOrigin( tf::Vector3(x, y, 0.0) );
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    q.normalized();
    transform.setRotation(q);*/
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "map"));
}

/*void CarControlCallback(const ugv_bit::motor_ctr::ConstPtr& msg)
{
    // gazebo_control.angular.z = msg->angle*3.1415926/180.0;//控制量为角度，转为gazebo中的弧度
    // gazebo_control.linear.x = msg->mode * msg->speed/1000.0;//发布的是电机转速，最高2500，此处自定义一个转速与车辆速度的对应关系，即0.001m/r；
    // gazebo_car_control_pub.publish(gazebo_control);
    
    vehicle_status.twist.angular.z = msg->angle*3.1416/180.0;
    control_msg_received = true;
    
}*/

void ControlCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& msg)
{
    // geometry_msgs::Twist gazebo_control;
    // gazebo_control.angular.z = msg->cmd.steering_angle/2;//控制量为角度，转为gazebo中的弧度
    // gazebo_control.linear.x = msg->cmd.linear_velocity;//发布的是电机转速，最高2500，此处自定义一个转速与车辆速度的对应关系，即0.001m/r；
    // gazebo_car_control_pub.publish(gazebo_control);
    //printf("entering controlcallback\n");
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.y = cmd_msg.linear.z = 0;
    cmd_msg.linear.x = msg->cmd.linear_velocity;
    cmd_msg.angular.z=msg->cmd.steering_angle/2;

    //printf("cmd.x=%.2f\n",cmd_msg.linear.x);

    cmd_pub.publish(cmd_msg);
}

// void GPSDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     geometry_msgs::PoseStamped current_pose;
//     current_pose.header = msg->header;
//     current_pose.pose.position.x = msg->pose.position.x - base_x;
//     current_pose.pose.position.y = msg->pose.position.y - base_y;
//     current_pose.pose.position.z = 0.0;
//     current_pose.pose.orientation = msg->pose.orientation;
//     current_pose_pub.publish(current_pose);

// }
geometry_msgs::Point utm2map(geometry_msgs::Point utm_p)
{
    double utmx_zero=0, utmy_zero=0;
    ros::param::get("/gps_base/utmx_zero",utmx_zero);
    ros::param::get("/gps_base/utmy_zero",utmy_zero);

    geometry_msgs::Point map_p;
    //map_p.y=-utm_p.x+utmx_zero,  map_p.x=utm_p.y-utmy_zero;
    map_p.x=utm_p.x-utmx_zero,  map_p.y=utm_p.y-utmy_zero;
    return map_p;    
}

// void GPS_base_DataCallback(const gps::MyGPS_msg::ConstPtr& msg)
// {
//     geometry_msgs::Point map_point,utm_point;
    
//     utm_point.x=msg->UTM_X;    utm_point.y=msg->UTM_Y;
//     map_point=utm2map(utm_point);
    
//     geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw((-msg->Angle+90)/180*3.1416);

//     geometry_msgs::PoseStamped current_pose;
//     current_pose.header = msg->header;
//     current_pose.pose.position.x = map_point.x;
//     current_pose.pose.position.y = map_point.y;
//     current_pose.pose.position.z = 0.0;
//     current_pose.pose.orientation = goal_quat;
//     current_pose_pub.publish(current_pose);
// }


void GPS_base_DataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Point map_point,utm_point;
    
    utm_point.x=msg->pose.position.x;    utm_point.y=msg->pose.position.y;
    map_point=utm2map(utm_point);
    
    

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Point Pre_pose;
    double speed;
    current_pose.header = msg->header;
    current_pose.pose.position.x = map_point.x;
    current_pose.pose.position.y = map_point.y;
    //current_pose.pose.position.x = msg->pose.position.x-base_x;
    //current_pose.pose.position.y = msg->pose.position.y-base_y;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation = msg->pose.orientation;
    current_pose_pub.publish(current_pose);
    
    //发布一个微分的速度信息
    if(tmr_speed.GetValue()>0.3)    //tmr_speed 定时器
    {
        float d=P2P(Pre_pose,current_pose.pose.position);
        Pre_pose=current_pose.pose.position;
        speed=d/tmr_speed.GetValue();
        tmr_speed.Clear();
        //publish velocity
        geometry_msgs::TwistStamped current_velocity;
        current_velocity.header=msg->header;
        current_velocity.twist.linear.x=speed;
        current_velocity.twist.linear.y=current_velocity.twist.linear.z=0;
        current_velocity.twist.angular.x=current_velocity.twist.angular.y=0;
        //geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw((-msg->Angle+90)/180*3.1416);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->pose.orientation,quat);

        double roll, pitch, yaw;//定义存储r\p\y的容器
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

        current_velocity.twist.angular.z=yaw;
        vehicle_status_pub.publish(current_velocity);
    }
}


void PoinCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZ> *in_pcl_point = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::fromROSMsg (*input, *in_pcl_point);

    pcl::PointCloud<pcl::PointXYZ> out_pcl_point;
    
    sensor_msgs::PointCloud2 out;
    geometry_msgs::PointStamped pose_source,pose_out;
    pcl::PointXYZ out_point;

    for(int i=0; i<in_pcl_point->points.size(); i++)
    {
        double points_dis;
        points_dis=sqrt(pow(in_pcl_point->points[i].x,2)+pow(in_pcl_point->points[i].y,2));
        if( (abs(in_pcl_point->points[i].x)<car_length  && abs(in_pcl_point->points[i].y) <car_width/2)||points_dis>obs_range)
           continue;
       
        pose_source.header.stamp = ros::Time(0);
        pose_source.header.frame_id = lidar_frame;
        
        pose_source.point.x = in_pcl_point->points[i].x;
        pose_source.point.y = in_pcl_point->points[i].y;
        pose_source.point.z = in_pcl_point->points[i].z;

        try
        {
            tf_->transformPoint("map",pose_source,pose_out);//"map" is target frame
        }     
        catch(tf::TransformException ex)
        {
            ROS_WARN("Transform exception:%s",ex.what());
            //return;
        }

        out_point.x = pose_out.point.x;
        out_point.y = pose_out.point.y;
        out_point.z = pose_out.point.z;

        out_pcl_point.push_back(out_point);
    }
    pcl::toROSMsg(out_pcl_point, out);
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";

    PointCloud_pub.publish(out);
    delete in_pcl_point;
}

//接收a*规划的路径
void GloblePathCallback(const nav_msgs::Path::ConstPtr& input)
{
    /*nav_msgs::Path msg;
    // geometry_msgs::PoseStamped single_path_point;

    msg.poses.resize(input->poses.size());
    for(int i;i<input->poses.size();i++)
    {
        msg.poses[i].pose.position.x = input->poses[i].pose.position.x;
        msg.poses[i].pose.position.y = input->poses[i].pose.position.y;
        // msg->poses.push_back(single_path_point);
    }*/


    laneArray_pub.lanes.clear();
    // autoware_msgs::LaneArray laneArray_pub;
    laneArray_pub.id = 0;

    autoware_msgs::Lane lane_pub;
    lane_pub.header.frame_id = "map";
    lane_pub.header.stamp = ros::Time::now();
    lane_pub.increment = 1;
    lane_pub.lane_id = 2;
    
    lane_pub.lane_index = 1;
    lane_pub.cost = 1;
    lane_pub.closest_object_distance = 20;
    lane_pub.closest_object_velocity = 0;
    lane_pub.is_blocked = false;

    autoware_msgs::Waypoint points;
    // vector<WayPoint> points_sommth;
    points.lane_id = 2;
    points.left_lane_id = 1;
    points.right_lane_id = 3;
    points.stop_line_id = 0;
    points.cost = 0.1;
    points.time_cost = 0;
    points.direction = 0;

    deque<_Point> fitting_reference_points;
    deque<_Point> curvature_reference_points;
    _Point in_point;



    for(int i=0; i<5; i++)    
    {
        in_point.x = input->poses[i].pose.position.x;
        in_point.y = input->poses[i].pose.position.y;
        fitting_reference_points.push_back(in_point);
    }

    for(int i=0; i<3; i++)    
    {
        in_point.x = input->poses[i].pose.position.x;
        in_point.y = input->poses[i].pose.position.y;
        curvature_reference_points.push_back(in_point);
    }

    for(int i=0; i<input->poses.size();i++)
    {
        //global id
        points.gid = i;
        //local id
        points.lid = i;

        points.pose.pose.position.x = input->poses[i].pose.position.x;
        points.pose.pose.position.y = input->poses[i].pose.position.y;
        points.pose.pose.position.z = 0;

        if(i>2 || i< input->poses.size()-2)
        {
            fitting_reference_points.pop_front();
            in_point.x = input->poses[i+2].pose.position.x;
            in_point.y = input->poses[i+2].pose.position.y;
            fitting_reference_points.push_back(in_point);   //fitting_reference_point放顺次相接的5个点
        }
        if(i>1 || i< input->poses.size()-1)
        {
            curvature_reference_points.pop_front();
            in_point.x = input->poses[i+1].pose.position.x;
            in_point.y = input->poses[i+1].pose.position.y;
            curvature_reference_points.push_back(in_point);   //curvature_reference_point放顺次相接的3个点
        }
        
        tf::Quaternion q;
        q.setRPY(0, 0, angle_calculation(label_determined(i,input->poses.size()-1),4,&fitting_reference_points));
        // current_pose.pose.orientation = q.normalized();
        q.normalized();
        tf::quaternionTFToMsg(q,points.pose.pose.orientation);

        points.twist.twist.linear.x = 1;
        points.twist.twist.angular.z = 0;

        points.dtlane.dist = i;
        if(i!=0)
            points.dtlane.dir = azimuth_calculation(input->poses[i-1].pose.position, input->poses[i].pose.position);
        else
            points.dtlane.dir = 0;
        points.dtlane.apara = 0;
        points.dtlane.r = radius_of_curvature(&curvature_reference_points);
        points.dtlane.slope = 0;
        points.dtlane.cant = 0;
        points.dtlane.lw = 2;
        points.dtlane.rw = 2;

        lane_pub.waypoints.push_back(points);
        //points_sommth.push_back(points);
    }

    //PlannerHNS::PlanningHelpers::FixPathDensity(points_smooth,0.1);
    // PlannerHNS::PlanningHelpers::SmoothPath(points_smooth, 0.49, 0.35 , 0.01);
    // for(int i=0; i<input->poses.size();i++)
    // {
    //     points.pose.pose.position.x = points_smooth.pose.poaition.x;
    //     points.pose.pose.position.y = points_smooth.pose.poaition.y;
    //     points.pose.pose.position.z = 0;
    //     lane_pub.waypoints.push_back(points);
    // }

    // autoware_latticePaths_pub.publish(lane_pub);
    laneArray_pub.lanes.push_back(lane_pub);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "autoware_transform");
    ros::NodeHandle nh;

    ros::NodeHandle pn;
    string lidar_topic;
    pn.getParam("/autoware_transform/car_width", car_width);
    pn.getParam("/autoware_transform/car_length", car_length);
    pn.getParam("/autoware_transform/lidar_topic", lidar_topic);
    pn.getParam("/autoware_transform/lidar_frame", lidar_frame);
    pn.getParam("/autoware_transform/obs_range",obs_range);

    tf_=new tf::TransformListener;

    map_origin_sub = nh.subscribe("/map_origin",1,&MapOriginCallback);

    // planner_car_control_sub = nh.subscribe("/car_velocity", 10, &CarControlCallback);//接收到的实际小车速度，

    //GPS_data_sub = nh.subscribe("/gps_base/GPS_Base", 1, &GPS_base_DataCallback);
    GPS_data_sub = nh.subscribe("/gps_base/UTM_coordinate", 1, &GPS_base_DataCallback);

    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose",1);
    // initial_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/initialpose",1);
    vehicle_status_pub = nh.advertise<geometry_msgs::TwistStamped>("/current_velocity",1);//接收到的实际小车速度信息转换为规划所需的格式

    // gazebo_car_control_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/cmd_vel",1);
    control_sub = nh.subscribe("/mpc_ctrl_cmd", 1 ,&ControlCallback);//接受规划的控制信息，然后转换为gazebo仿真所需的信息
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/car_cmd", 10);

    GloblePath_sub = nh.subscribe("global_path_map", 1, &GloblePathCallback);
    autoware_GlobalPlannerPaths_pub = nh.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array",1);
    autoware_latticePaths_pub = nh.advertise<autoware_msgs::Lane>("/lane_waypoints",1);

    lidar_sub = nh.subscribe(lidar_topic, 1, &PoinCloudCallback);
    PointCloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_map",1);

    ros::Rate loop_rate(30);
    
    while(ros::ok())
    {
        ros::spinOnce();
        autoware_GlobalPlannerPaths_pub.publish(laneArray_pub);
        loop_rate.sleep();  
    }

    return 0;
}