#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "utils.h"

/*ROS2 msgs*/
#include <nav_msgs/msg/odometry.hpp>

/*TF libraries*/
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

/*PX4 odometry msg*/
#include <px4_msgs/msg/vehicle_odometry.hpp>

/*Eigen libraries*/
#include <Eigen/Eigen>
#include <Eigen/Dense>

class OdomRTABMAP : public rclcpp::Node {

    public:
        /*Constructor*/
        OdomRTABMAP();

    private:
        void tf_rtabmap_listener();
        void t265_odom_cb( const nav_msgs::msg::Odometry::SharedPtr );
        void px4_odom_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr );
        void tf_base_publisher();
        void odom_px4_publisher();
        void odom_viz_publisher();

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_px4_sub;

        rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_px4_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_viz_pub;

        rclcpp::TimerBase::SharedPtr _timer_tf_in;
        rclcpp::TimerBase::SharedPtr _timer_px4_out;
        rclcpp::TimerBase::SharedPtr _timer_viz_out;
        rclcpp::TimerBase::SharedPtr _timer_tf_out;

        Eigen::Vector3d _rtabmap_pos, _t265_pos, _px4_pos, _viz_pos, _t265_vel, _px4_vel, _viz_vel, _t265_ang_vel, _px4_ang_vel;
        Eigen::Vector4d _rtabmap_quat, _t265_quat, _px4_quat, _viz_quat;
        Eigen::Vector3d _temp_rpy;

        Eigen::Vector3d _px4_pos_in;
        Eigen::Vector4d _px4_quat_in; 

        /*Rotation matrices*/
        Eigen::Matrix3d _R_rtabmap2ned, _R_t265toNed;
        Eigen::Vector4d _qx;

        /*Listener*/
        std::string _target_frame, _source_frame;
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener{nullptr};

        /*Broadcaster*/
        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

        /*Parameters*/
        bool _use_t265, _use_rtabmap;


};