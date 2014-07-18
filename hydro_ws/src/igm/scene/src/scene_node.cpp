#include <ros/ros.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "scene_node");
    ros::NodeHandle nh("~");



    ros::Rate loop_rate(60);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
