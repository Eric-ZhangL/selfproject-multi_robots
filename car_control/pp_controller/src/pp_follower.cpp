#include "pp_controller/pp_follower.h"


PurePursuit::PurePursuit(){
    nh.param("robot_name",robot_name,std::string("mobile_base"));
    trajectory_sub_=nh.subscribe("/"+robot_name+"/Path_Load", 10, &PurePursuit::TrajectoryCallback,this);
    car_pose_sub_ = nh.subscribe("/"+robot_name+"/map_carPose", 10, &PurePursuit::mapCarPoseCallback, this);

    pre_point_pub=nh.advertise<visualization_msgs::Marker>("/"+robot_name+"/pp/pre_point_visual",10);
    control_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/"+robot_name+"/car_cmd", 1);
    tf_=new tf::TransformListener;

    goal_reached_ = false;
    get_path_flag_ = false;
    //yu预瞄距离
    ld_ = 0.85;


    pos_tol_ = 0.05;
    v_max_ = 3.0;
    v_=1.0;
    // v_ = v_max_;
    w_max_ = 0.75;
    wheel_base=1.04;
}

PurePursuit::~PurePursuit()
{
    
}

void PurePursuit::TrajectoryCallback(const nav_msgs::Path& msg){
    path_ = msg;

    path_.header.frame_id="map";
    path_.header.stamp=ros::Time(0);
    for(int i=0;i<path_.poses.size();i++){
        path_.poses[i].header.frame_id="map";
        path_.poses[i].header.stamp=ros::Time(0);
        path_.poses[i].pose.orientation.x=0;
        path_.poses[i].pose.orientation.y=0;
        path_.poses[i].pose.orientation.z=0;
        path_.poses[i].pose.orientation.w=1;
    }

    idx_ = 0;
    if (msg.poses.size() > 0){
        goal_reached_ = false;
    }
    else{
        goal_reached_ = true;
        ROS_WARN_STREAM("Received empty path!");
    }
    get_path_flag_ = true ;
}
void PurePursuit::mapCarPoseCallback(const geometry_msgs::PoseStamped& msg){
    car_pose_ = msg;
    cv::Point2d car_point;
    geometry_msgs::Twist cmd_vel;

    car_point.x = car_pose_.pose.position.x;
    car_point.y = car_pose_.pose.position.y;

    // std::cout<<"path_end_point: "<<car_point.x<<"  "<<car_point.y<<std::endl;
    // if(!path_.poses.empty()){
    //     std::cout<<"path_end_point: "<<path_.poses.back().pose.position.x<<"  "<<path_.poses.back().pose.position.y<<std::endl;
    // }
    double min_dis = DBL_MAX;
    int min_index = 0;
    for(size_t i = 0;i < path_.poses.size();i++){
        cv::Point2d current_point;
        current_point.x = path_.poses[i].pose.position.x;
        current_point.y = path_.poses[i].pose.position.y;
        if(GetDistance(current_point,car_point) < min_dis){
            min_dis = GetDistance(current_point,car_point);
            min_index = i;
        }
    }
    geometry_msgs::PoseStamped base_lookahead_point;

    geometry_msgs::Point pre_point;//yu预瞄点
    for (; idx_ < path_.poses.size(); idx_++){

        cv::Point2d current_point;
        current_point.x = path_.poses[idx_].pose.position.x;
        current_point.y = path_.poses[idx_].pose.position.y;
        if (GetDistance(current_point,car_point) > ld_ && idx_ >= min_index){
            pre_point=path_.poses[idx_].pose.position;
            // pre_point.x=path_.poses[idx_].pose.position.x;
            try{
                // std::cout<<"111111"<<std::endl;
                // std::cout<<path_.poses[idx_].header.frame_id<<std::endl;
                tf_->transformPose("base_link",path_.poses[idx_],base_lookahead_point);
            }    
            catch(tf::TransformException ex){
                ROS_WARN("Transform exception:%s",ex.what());
                // return;
            }
        break;
        }
    }

    //reach goal
    if (!path_.poses.empty() && idx_ >= path_.poses.size()){
        geometry_msgs::PoseStamped base_end_point;
        try{
            tf_->transformPose(robot_name+"/base_link",path_.poses.back(),base_end_point);
        }    
        catch(tf::TransformException ex){
            ROS_WARN("Transform exception:%s",ex.what());
            return;
        }
        //到达终点的误差范围内
        if (fabs(base_end_point.pose.position.x) <= pos_tol_){
            // We have reached the goal
            goal_reached_ = true;
            get_path_flag_=false;
            std::cout<<"reach goal!"<<std::endl;
            // Reset the path
            path_ = nav_msgs::Path();
        }
        else{
            tf::StampedTransform transform;
            try{
                tf_->lookupTransform("map",robot_name+"/base_link",ros::Time(), transform);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                return;
            }

            tf::Quaternion q;
            double roll, pitch, yaw;
            q = transform.getRotation();
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw); //进行转换

            double k_end = tan(yaw); // Slope of line defined by the last path pose
            double l_end = base_end_point.pose.position.y - k_end * base_end_point.pose.position.x;
            double a = 1 + k_end * k_end;
            double b = 2 * l_end;
            double c = l_end * l_end - ld_ * ld_;
            double D = sqrt(b*b - 4*a*c);
            double x_ld = (-b + copysign(D,v_)) / (2*a);
            double y_ld = k_end * x_ld + l_end;

            base_lookahead_point.pose.position.x = x_ld;
            base_lookahead_point.pose.position.y = y_ld;

            base_lookahead_point.pose.position.y = base_end_point.pose.position.z;
        }
    }

    if(!goal_reached_){
        double yt = base_lookahead_point.pose.position.y;
        double ld_2 = ld_ * ld_;
        cmd_vel.angular.z = std::min(atan(wheel_base/ ld_2 * yt), w_max_);
        // cmd_vel.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_);
        // Set linear velocity for tracking.
        cmd_vel.linear.x = std::min( v_, v_max_);
    }
    // std::cout<<"z:"<<cmd_vel.angular.z<<"  "<<"v:"<<cmd_vel.linear.x<<std::endl;
    
    pre_point_pub.publish((GetVisualMarker(pre_point, GetColor(0, 1, 0))));

    if(get_path_flag_)
        control_cmd_pub_.publish(cmd_vel);
    if(goal_reached_)
    {
        // Stop moving.
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        control_cmd_pub_.publish(cmd_vel);
    }
}

double PurePursuit::GetDistance(cv::Point2d point_1,cv::Point2d point_2){
    return sqrt(pow(point_1.x-point_2.x,2)+pow(point_1.y-point_2.y,2));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PurePursuit controller;
    ros::spin();
    return 0;
}