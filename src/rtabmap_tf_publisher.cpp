#include "../include/odometry_launch_pkg/rtabmap_tf_publisher.h"

OdomRTABMAP::OdomRTABMAP() : rclcpp::Node("rtabmap_tf_publisher") {

    /*Get parameters*/
    this->declare_parameter<bool>("use_t265", true);
    _use_t265 = this->get_parameter("use_t265").as_bool();
    this->declare_parameter<bool>("use_rtabmap", false);
    _use_rtabmap = this->get_parameter("use_rtabmap").as_bool();
    this->declare_parameter<std::string>("target_frame", "base_link");
    _target_frame = this->get_parameter("target_frame").as_string();
    this->declare_parameter<std::string>("source_frame", "map");
    _source_frame = this->get_parameter("source_frame").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    if( _use_t265 ) {
        _odom_sub = 
                this->create_subscription<nav_msgs::msg::Odometry>("/t265/odom/sample", qos, 
                std::bind(&OdomRTABMAP::t265_odom_cb, this, std::placeholders::_1));
    }

    if( _use_rtabmap ) {
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        _timer_tf_in = 
            this->create_wall_timer(10ms, std::bind(&OdomRTABMAP::tf_rtabmap_listener, this));
    }

    _tf_broadcaster =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    _odom_px4_sub = 
        this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, 
        std::bind(&OdomRTABMAP::px4_odom_cb, this, std::placeholders::_1));

    /*Publishers*/
    _odom_px4_pub = 
        this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
        // this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_mocap_odometry", 10);
    _odom_viz_pub =
        this->create_publisher<nav_msgs::msg::Odometry>("/rtabmap_tf_publisher/odom", 10);

    /*Timer callbacks publishers*/
    _timer_px4_out = 
        this->create_wall_timer(20ms, std::bind(&OdomRTABMAP::odom_px4_publisher, this));
    _timer_viz_out =
        this->create_wall_timer(20ms, std::bind(&OdomRTABMAP::odom_viz_publisher, this));

    _R_t265toNed = utilities::rotx(-M_PI);
    _qx = utilities::rot2quat( _R_t265toNed );

}

void OdomRTABMAP::t265_odom_cb( const nav_msgs::msg::Odometry::SharedPtr t265_odom ) {

    _t265_pos << t265_odom->pose.pose.position.x, t265_odom->pose.pose.position.y, t265_odom->pose.pose.position.z;
    _t265_vel << t265_odom->twist.twist.linear.x, t265_odom->twist.twist.linear.y, t265_odom->twist.twist.linear.z;
    _t265_quat << t265_odom->pose.pose.orientation.w, t265_odom->pose.pose.orientation.x, t265_odom->pose.pose.orientation.y, t265_odom->pose.pose.orientation.z;
    _t265_ang_vel << t265_odom->twist.twist.angular.x, t265_odom->twist.twist.angular.y, t265_odom->twist.twist.angular.z;
}

void OdomRTABMAP::px4_odom_cb( const px4_msgs::msg::VehicleOdometry::SharedPtr px4_odom ) {

    _px4_pos_in << px4_odom->position[0], px4_odom->position[1], px4_odom->position[2];
    _px4_quat_in << px4_odom->q[0], px4_odom->q[1], px4_odom->q[2], px4_odom->q[3];

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom_ned";
    t.child_frame_id = "base_link_frd";
    t.transform.translation.x = _px4_pos_in[0];
    t.transform.translation.y = _px4_pos_in[1];
    t.transform.translation.z = _px4_pos_in[2];
    t.transform.rotation.w = _px4_quat_in[0];
    t.transform.rotation.x = _px4_quat_in[1];
    t.transform.rotation.y = _px4_quat_in[2];
    t.transform.rotation.z = _px4_quat_in[3];

    _tf_broadcaster->sendTransform(t);
    

}

void OdomRTABMAP::tf_rtabmap_listener() {


    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = _tf_buffer->lookupTransform(_target_frame, _source_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }

    _rtabmap_pos[0] = transform.transform.translation.x;
    _rtabmap_pos[1] = transform.transform.translation.y;
    _rtabmap_pos[2] = transform.transform.translation.z;

    _rtabmap_quat[0] = transform.transform.rotation.w;
    _rtabmap_quat[1] = transform.transform.rotation.x;
    _rtabmap_quat[2] = transform.transform.rotation.y;
    _rtabmap_quat[3] = transform.transform.rotation.z;

}

void OdomRTABMAP::odom_px4_publisher() {

        /*From FLU to NED*/
        _px4_pos = _R_t265toNed*_t265_pos;
        _px4_vel = _R_t265toNed*_t265_vel;
        _px4_quat = utilities::rot2quat( utilities::Q2R( _t265_quat )*_R_t265toNed );
        // _viz_quat = utilities::rot2quat( utilities::Q2R( _t265_quat )*_R_t265toNed );
        // _temp_rpy = utilities::quatToRpy( _viz_quat );
        // _temp_rpy[0] += M_PI;
        // _px4_quat = utilities::RpyToQuat( _temp_rpy );
        // _px4_quat = utilities::quat_product( _qx, _t265_quat ) ;
        _px4_ang_vel = _R_t265toNed*_t265_ang_vel;


        auto odom_px4 = std::make_unique<px4_msgs::msg::VehicleOdometry>();
        odom_px4->pose_frame = 2;
        odom_px4->timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
            odom_px4->position[0] = _px4_pos[0];
            odom_px4->position[1] = _px4_pos[1];
            odom_px4->position[2] = _px4_pos[2];
            odom_px4->q[0] = NAN;
            odom_px4->q[1] = NAN;
            odom_px4->q[2] = NAN;
            odom_px4->q[3] = NAN;
            odom_px4->velocity_frame = 2;
            odom_px4->velocity[0] = _px4_vel[0];
            odom_px4->velocity[1] = _px4_vel[1];
            odom_px4->velocity[2] = _px4_vel[2];
            // odom_px4->angular_velocity[0] = _px4_ang_vel[0];
            // odom_px4->angular_velocity[1] = _px4_ang_vel[1];
            // odom_px4->angular_velocity[2] = _px4_ang_vel[2];
            // odom_px4->velocity[0] = NAN;
            // odom_px4->velocity[1] = NAN;
            // odom_px4->velocity[2] = NAN;
            odom_px4->angular_velocity[0] = NAN;
            odom_px4->angular_velocity[1] = NAN;
            odom_px4->angular_velocity[2] = NAN;

        // else if( !_use_t265 && _use_rtabmap ) {
        //     odom_px4->position[0] = _rtabmap_pos[0];
        //     odom_px4->position[1] = _rtabmap_pos[1];
        //     odom_px4->position[2] = _rtabmap_pos[2];
        // }

        // std::cout<<"Pubblico"<<std::endl;
        _odom_px4_pub->publish( std::move(odom_px4) );
}

void OdomRTABMAP::odom_viz_publisher() {

    auto odom_viz = std::make_unique<nav_msgs::msg::Odometry>();
    odom_viz->header.stamp = this->now();
    odom_viz->header.frame_id = "map";
    odom_viz->child_frame_id = "base_link_odom";
    odom_viz->pose.pose.position.x = _px4_pos[0];
    odom_viz->pose.pose.position.y = _px4_pos[1];
    odom_viz->pose.pose.position.z = _px4_pos[2];
    odom_viz->pose.pose.orientation.w = _px4_quat[0];
    odom_viz->pose.pose.orientation.x = -_px4_quat[1];
    odom_viz->pose.pose.orientation.y = _px4_quat[2];
    odom_viz->pose.pose.orientation.z = _px4_quat[3];
    odom_viz->twist.twist.linear.x = _px4_vel[0];
    odom_viz->twist.twist.linear.y = _px4_vel[1];
    odom_viz->twist.twist.linear.z = _px4_vel[2];
    odom_viz->twist.twist.angular.x = _px4_ang_vel[0];
    odom_viz->twist.twist.angular.y = _px4_ang_vel[1];
    odom_viz->twist.twist.angular.z = _px4_ang_vel[2];

    _odom_viz_pub->publish( std::move(odom_viz) );
}

int main(int argc, char *argv[]) {

    std::cout<< "Starting Odometry republisher node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomRTABMAP>());

    rclcpp::shutdown();
    return 0;
}