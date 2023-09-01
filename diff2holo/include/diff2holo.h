#include <string>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using namespace geometry_msgs;

class Diff2Holo {
public: 
    Twist vel_unicycle; // Input
    Twist vel_mecanum; // Output 
    std::string unicycle_frame; // Frame name of input velocity

    double v_max, w_max; 
    
    Diff2Holo(tf2_ros::Buffer& tfBuffer);
    void twistCallback(const Twist::ConstPtr& vel);

private: 
    ros::Subscriber vel_sub_;
    ros::Publisher vel_pub_;
    tf2_ros::Buffer& tfBuffer_;

    TransformStamped trans; // Stores frame transform info

    void get_transform();
    void zeroVel(Twist& vel);
    void convert2D(const Twist& in, Twist& out, const TransformStamped& trans); 
};
