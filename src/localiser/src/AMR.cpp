#include "AMR.h"

// callback for /robot_pose topic subscriber
void AMR::pose_callback(const geometry_msgs::Pose::ConstPtr& msg) {
    // Updating the current pose values
    curr_pose = *msg;
}

// Constructor
AMR::AMR(ros::NodeHandle nh) {
    // Initializing the initial pose variable values
    init_pose.header.seq = 0;
    init_pose.header.stamp = ros::Time::now();
    init_pose.pose.pose.position.x = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.position.z = 0.0;
    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = 0.0;
    init_pose.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 36; i++) {
        init_pose.pose.covariance[i] = 0.0;
    }

    // Publisher for setting the intial position
    pose_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    // Subscriber for reading the current position
    pose_subscriber = nh.subscribe<geometry_msgs::Pose>("/robot_pose", 10, &AMR::pose_callback, this);

    // pose file path relative to the package path
    filepath = ros::package::getPath("localiser") + "/pose.txt";

    // Read the pose values from file
    read_pose_file();

    // waiting for the subscriber for /initialpose topic to initialize properly
    while (pose_publisher.getNumSubscribers() < 1) {
    }

    // Publishing the initial pose values read from the file previously
    pose_publisher.publish(init_pose);
}

// File Reading function
void AMR::read_pose_file() {
    // Opening the Pose file
    std::ifstream pose_reader(filepath);
    std::string dataline;

    // Reading the file data if it exists
    if (pose_reader.is_open()) {
        // Position Data
        getline(pose_reader, dataline);
        init_pose.pose.pose.position.x = std::stof(dataline);
        getline(pose_reader, dataline);
        init_pose.pose.pose.position.y = std::stof(dataline);
        getline(pose_reader, dataline);
        init_pose.pose.pose.position.z = std::stof(dataline);

        // Orientation Data
        getline(pose_reader, dataline);
        init_pose.pose.pose.orientation.x = std::stof(dataline);
        getline(pose_reader, dataline);
        init_pose.pose.pose.orientation.y = std::stof(dataline);
        getline(pose_reader, dataline);
        init_pose.pose.pose.orientation.z = std::stof(dataline);
        getline(pose_reader, dataline);
        init_pose.pose.pose.orientation.w = std::stof(dataline);

    } else {
        ROS_INFO("File Stream not open\n");
    }
    pose_reader.close();
}

// File writing function
void AMR::write_pose_file() {
    // Opening the pose file for writing the current pose values. trunc to empty the file before writing.
    std::ofstream pose_writer(filepath, std::ios::trunc);

    // If file is open then write the pose data in each line
    if (pose_writer.is_open()) {
        pose_writer << curr_pose.position.x << std::endl;
        pose_writer << curr_pose.position.y << std::endl;
        pose_writer << curr_pose.position.z << std::endl;
        pose_writer << curr_pose.orientation.x << std::endl;
        pose_writer << curr_pose.orientation.y << std::endl;
        pose_writer << curr_pose.orientation.z << std::endl;
        pose_writer << curr_pose.orientation.w;
        pose_writer.close();
    } else {
        ROS_ERROR("Could not open file to write pose!\n");
    }
}

// Destructor
AMR::~AMR() {
}
