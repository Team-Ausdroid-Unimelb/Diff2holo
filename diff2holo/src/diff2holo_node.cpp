#include <diff2holo.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "diff2holo_node");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListen(tfBuffer);

    Diff2Holo diff2holo(tfBuffer); 

    ros::spin();

    return 0;
}
