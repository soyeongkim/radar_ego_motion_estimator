#ifndef __RadarEgoMotionEstimation_HPP__
#define __RadarEgoMotionEstimation_HPP__

// ROS header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "sensor_msgs/PointCloud2.h"

// Utility header
#include <pcl_conversions/pcl_conversions.h> // pcl_conversions::toPCL
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <novatel_oem7_msgs/INSPVAX.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/Float32MultiArray.h>

// Visualization
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// ego motion estimation
#include "radar_ego_velocity_estimator/radar_ego_velocity_estimator.h"

struct PointRadarTarget
{
    PCL_ADD_POINT4D;
    float intensity;
    float power;          // Signal-to-Noise Ratio
    float range;        // Range (meters)
    float vel;        // Speed (meters per second)
    float azi_angle;      // Azimuth angle (degrees)
    float ele_angle;    // Elevation angle (degrees)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    // make sure our new allocators are aligned
} EIGEN_ALIGN16;    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT( PointRadarTarget,        
                                ( float, x, x )
                                ( float, y, y )
                                ( float, z, z )
                                ( float, intensity, intensity)
                                ( float, power, power )
                                ( float, range, range )
                                ( float, vel, vel )
                                ( float, azi_angle, azi_angle )
                                ( float, ele_angle, ele_angle )
                                )

template<typename pointT>
class RadarEgoMotionEstimation
{
    public:
        RadarEgoMotionEstimation();
        ~RadarEgoMotionEstimation();
        void Initialization();

    private:
        const double RAD2DEG = 57.2958;
        const double DEG2RAD = 0.0175;
        const double KPH2MPS = 0.2778;
        const double MPS2KPH = 3.6;

    private:
        bool b_init_pose_flag_ = false;
        bool b_lidar_scan_exist_ = false;

        double d_init_lat_;
        double d_init_lon_;
        double d_init_heading_;
        double d_init_height_;

        const double D_GEOD_A = 6378137.0;//SemiMajorAxis
        const double D_GEOD_E2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2
        const double D_RAD_2_DEG = 180 / M_PI;
        const double D_DEG_2_RAD = M_PI / 180;
    
    // ROS variable
    private:
        ros::Subscriber rossub_radar_points_;
        ros::Subscriber rossub_novatel_pose_;

        ros::Publisher rospub_static_radar_points_;
        ros::Publisher rospub_radar_ego_motion_odom_;

        std_msgs::Header h_radar_header_;

        std::string s_radar_frame_id_;

        novatel_oem7_msgs::INSPVAX inspvax_curr_novatel_msg_;
        sensor_msgs::PointCloud2 i_point_cloud_;

    // launch param
    private:
        std::string s_input_points_;
        double d_vehicle_length_r_m_;
    
    // ego motion estimation
        reve::RadarEgoVelocityEstimator radar_ego_motion_vel_estimator_;
        Eigen::Vector3d radar_ego_motion_init_vel_;
        Eigen::Vector3d radar_ego_motion_init_vel_sigma_;

    public:
        void RunRadarEgoMotionEstimation();
    
    private:
        void CallbackRadarPoints(const sensor_msgs::PointCloud2::ConstPtr &msg);

};
#endif