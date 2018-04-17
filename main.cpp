#include <iostream>
#include <ros/init.h>
#include <csignal>
#include "DisplayInterface.h"

int main(int argc,char** argv) {
    std::string node_name = "display_interface";
    ros::init(argc,argv,node_name);
    DisplayInterface display;

    ros::Duration d(5);

    while(ros::ok()){
        //pose
        sgbot::Pose2D pose(1.0,1.0,0.0);
        display.displayPose(pose);

        //pose with covariance
        sgbot::la::Matrix<float,3,3> matrix;
        matrix(0,0) = 2;
        matrix(0,1) = 5;
        matrix(1,0) = 8;
        matrix(1,1) = 1;
        matrix(2,2) = 0;
        display.displayPoseWithCovariance(pose,matrix);

        //point
        sgbot::Point2D point2D(2,2);
        display.displayPoint(point2D);

        //lidar2d
        sgbot::sensor::Lidar2D lidar2D;
        lidar2D.setOrigin(sgbot::Point2D(0,0));
        for (int i = 0; i < 10; ++i) {
            sgbot::Point2D local_point2D(i,i);
            lidar2D.addPoint(local_point2D);
        }
        display.displayScan(lidar2D);

        //update
        display.update();


        ros::spinOnce();
        ROS_INFO("publish");


        d.sleep();
    }

    std::cout << "Hello, World!" << std::endl;
    return 0;
}