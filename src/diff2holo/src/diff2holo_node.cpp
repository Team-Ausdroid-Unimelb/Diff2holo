#include <diff2holo.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "diff2holo_node");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListen(tfBuffer);

    Diff2Holo diff2holo(tfBuffer); 
    // Diff2Holo diff2holo;

    // ros::Rate loop_rate(6.0);
    // while (ros::ok()) {
    //     diff2holo.take_transform();
    //     // try {
    //     //     std::cout << "print something" << std::endl; 
    //     //     diff2holo.trans = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0));
    //     // } catch (tf2::TransformException &ex) {
    //     //     ROS_WARN("%s",ex.what());
    //     //     continue; 
    //     // }
    //     // std::cout << diff2holo.vel_mecanum.linear.x << std::endl; 

    //     loop_rate.sleep();
    // }

    ros::spin();

    return 0;
}
