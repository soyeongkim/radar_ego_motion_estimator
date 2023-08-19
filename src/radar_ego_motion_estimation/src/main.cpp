/*
 * @copyright Automotive Intelligence Lab, Konkuk University
 * @author kimchuorok@gmail.com
 * @file main.cpp
 * @brief 
 * @version 1.0
 * @date 2023-08-19
 */

#include "main.hpp"

template<typename pointT>
RadarEgoMotionEstimation<pointT>::RadarEgoMotionEstimation():
s_input_points_("rslidar_points")
{
    ros::NodeHandle nh;
    std::string radar_ego_motion_ini_path;

    nh.getParam("/radar_ego_motion_estimation/d_vehicle_length_r_m_"  , d_vehicle_length_r_m_);
    nh.getParam("/radar_ego_motion_estimation/s_input_points_"  , s_input_points_);
    nh.getParam("/radar_ego_motion_estimation/str_radar_ego_motion_ini_path", radar_ego_motion_ini_path);

    if(radar_ego_motion_vel_estimator_.Init(radar_ego_motion_ini_path) == false)
    {
        ROS_ERROR_STREAM("radar_ego_motion_estimation inirialization fail");
    }

    rossub_radar_points_ = nh.subscribe(s_input_points_, 1, &RadarEgoMotionEstimation<pointT>::CallbackRadarPoints, this);

    rospub_static_radar_points_ = nh.advertise<sensor_msgs::PointCloud2>("/static_radar_points", 10);
    rospub_radar_ego_motion_odom_ = nh.advertise<nav_msgs::Odometry>("/radar_ego_motion_odom", 10);
}

template<typename pointT>
RadarEgoMotionEstimation<pointT>::~RadarEgoMotionEstimation()
{}

template<typename pointT>
void RadarEgoMotionEstimation<pointT>::RunRadarEgoMotionEstimation()
{
    if (!b_lidar_scan_exist_)
    {
        return;
    }

    // Ego motion vel estimator
    sensor_msgs::PointCloud2 inlier_radar_msg;
    radar_ego_motion_vel_estimator_.estimate(i_point_cloud_, radar_ego_motion_init_vel_, radar_ego_motion_init_vel_sigma_, inlier_radar_msg);

    // pub ego motion vel estimation for debugging
    nav_msgs::Odometry radar_ego_motion_odom_msgs;
    radar_ego_motion_odom_msgs.pose.pose.position.x = radar_ego_motion_init_vel_[0];
    radar_ego_motion_odom_msgs.pose.pose.position.y = radar_ego_motion_init_vel_[1];
    radar_ego_motion_odom_msgs.pose.pose.position.z = radar_ego_motion_init_vel_[2];

    radar_ego_motion_odom_msgs.pose.covariance[0] = radar_ego_motion_init_vel_sigma_[0];
    radar_ego_motion_odom_msgs.pose.covariance[1] = radar_ego_motion_init_vel_sigma_[1];
    radar_ego_motion_odom_msgs.pose.covariance[2] = radar_ego_motion_init_vel_sigma_[2];

    // just put yaw rate value
    radar_ego_motion_odom_msgs.pose.pose.orientation.w = (radar_ego_motion_init_vel_[1]/d_vehicle_length_r_m_)*RAD2DEG;
    rospub_radar_ego_motion_odom_.publish(radar_ego_motion_odom_msgs);

    // Pub static points for debugging
    inlier_radar_msg.header.frame_id = s_radar_frame_id_;
    inlier_radar_msg.header.stamp = h_radar_header_.stamp;

    rospub_static_radar_points_.publish(inlier_radar_msg);

    b_lidar_scan_exist_ = false;
}

template<typename pointT>
void RadarEgoMotionEstimation<pointT>::CallbackRadarPoints(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    i_point_cloud_ = *msg;
    b_lidar_scan_exist_ = true;

    h_radar_header_ = msg->header;
    s_radar_frame_id_ = msg->header.frame_id;
}

int main(int argc, char **argv)
{
    std::string str_nodename = "radar_ego_motion_estimation";
	ros::init(argc, argv, str_nodename);
    ros::NodeHandle nh;
	
    int i_init_loop_frequency = 50;
    
    ros::Rate rosrate_loop_rate(i_init_loop_frequency);

    RadarEgoMotionEstimation<pcl::PointXYZI> RadarEgoMotionEstimation;

    while(ros::ok())
    {
            ros::spinOnce();
            RadarEgoMotionEstimation.RunRadarEgoMotionEstimation();
            rosrate_loop_rate.sleep();
    }

    return 0;
}