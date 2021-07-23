#ifndef _AMR_H
#define _AMR_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "rosbag/view.h"
#include "rosbag/bag.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

class AMR {
    private:
        geometry_msgs::Pose curr_pose;
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        ros::Publisher pose_publisher;
        ros::Subscriber pose_subscriber; 
        std::string filepath;
        std::fstream pose_file;
        rosbag::Bag bag;
        void pose_callback(const geometry_msgs::Pose::ConstPtr&);
    public:
        AMR(ros::NodeHandle); 
        ~AMR();
        void read_pose_file();
        void write_pose_file();
};

#endif
