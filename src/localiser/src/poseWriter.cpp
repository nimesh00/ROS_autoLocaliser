#include "ros/ros.h"
#include "AMR.h"

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "poseManager");

    ros::NodeHandle nh;
    
    // Initializing the AMR object
    AMR robot(nh);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // writing pose values to file
        robot.write_pose_file();

        ros::spinOnce();
        loop_rate.sleep();
    }
}
