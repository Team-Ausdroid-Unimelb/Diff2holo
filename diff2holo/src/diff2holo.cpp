#include <diff2holo.h>

Diff2Holo::Diff2Holo(tf2_ros::Buffer& tfBuffer) : 
    tfBuffer_(tfBuffer) {
    ros::NodeHandle nh;

    vel_sub_ = nh.subscribe<Twist>("/unicycle_vel", 10, 
        boost::bind(&Diff2Holo::twistCallback, this, _1));
    vel_pub_ = nh.advertise<Twist>("/cmd_vel", 10);
    nh.param<std::string>("unicycle_frame", unicycle_frame, "Robot_r");
    nh.param<double>("v_max", v_max, 1.0);
    nh.param<double>("w_max", w_max, 1.0);

    zeroVel(vel_unicycle); 
    zeroVel(vel_mecanum);
}

void Diff2Holo::get_transform() {
    ros::Duration timeout(0.1);

    if (tfBuffer_.canTransform(unicycle_frame, "base_link", 
        ros::Time(0), timeout))
        trans = tfBuffer_.lookupTransform(unicycle_frame, "base_link", 
            ros::Time(0), timeout);
    else 
        ROS_WARN("Transform from %s to \"base_link\" is not available", 
            unicycle_frame.c_str());
}
/**
 * @brief Translate velocity command for a unicycle chassis
 *        to a mecanum chassis, assuming the robot moves in a plane. 
 *        Rotation of a frame transform is not used in this function.
*/
void Diff2Holo::convert2D(const Twist& in, Twist& out, 
    const TransformStamped& trans) {
    if (trans.child_frame_id != "base_link" || 
        trans.header.frame_id != unicycle_frame) {
        ROS_WARN("Invalid robot frame id, velocity command conversion abort.");
        return; 
    }

    double x = trans.transform.translation.x;
    double y = trans.transform.translation.y;
    out.linear.x = in.linear.x - in.angular.z*y;
    out.linear.y = -in.angular.z*x;
    out.angular.z = in.angular.z;
}

void Diff2Holo::twistCallback(const Twist::ConstPtr& vel) {
    if (vel->linear.x < v_max && vel->linear.x > -v_max)
        vel_unicycle.linear.x = vel->linear.x;
    else 
        vel_unicycle.linear.x = v_max;
    if (vel->angular.z < w_max && vel->angular.z > -w_max)
        vel_unicycle.angular.z = vel->angular.z;
    else 
        vel_unicycle.angular.z = w_max; 
        
    get_transform();
    convert2D(vel_unicycle, vel_mecanum, trans);

    vel_pub_.publish(vel_mecanum);
}

/**
 * @brief Put value of a Twist variable to zero
}
*/
void Diff2Holo::zeroVel(Twist& vel) {
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
}
 