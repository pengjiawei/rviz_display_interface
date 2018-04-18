//
// Created by root on 18-4-17.
//

#include "DisplayInterface.h"
#include <tf/tf.h>
void DisplayInterface::displayScan(const sgbot::sensor::Lidar2D &scan) {
    int num_readings = scan.getCount();
    int laser_frequency = 40;

    laserScan.angle_min = -1.57;
    laserScan.angle_max = 1.57;
    laserScan.angle_increment = 3.14 / num_readings;
    laserScan.time_increment = (1 / laser_frequency) / (num_readings);
    laserScan.range_min = 0.0;
    laserScan.range_max = 100.0;

    laserScan.ranges.resize(scan.getCount());
    for (int i = 0; i < num_readings; ++i) {
        //does Lidar2d need a distance method?
        laserScan.ranges[i] = sgbot::distance(scan.getOrigin(),scan.getPoint(i));
    }
    laserScan.header.frame_id = "map";
}

void DisplayInterface::displayPose(const sgbot::Pose2D &pose) {
    geo_pose.pose.position.x = pose.x();
    geo_pose.pose.position.y = pose.y();
    geo_pose.pose.position.z = 0.0;
    geo_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());

    geo_pose.header.frame_id = "map";
}

void DisplayInterface::displayPoseWithCovariance(const sgbot::Pose2D &pose,
                                                 const sgbot::la::Matrix<float, 3, 3> &covariance) {
    //initial pose
    poseWithCovarianceStamped.pose.pose.position.x = pose.x();
    poseWithCovarianceStamped.pose.pose.position.y = pose.y();
    poseWithCovarianceStamped.pose.pose.position.z = 0.0;
    poseWithCovarianceStamped.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
    //initial covariance
    poseWithCovarianceStamped.pose.covariance[0] = covariance(0,0);//x and x
    poseWithCovarianceStamped.pose.covariance[1] = covariance(0,1);//x and y
    poseWithCovarianceStamped.pose.covariance[6] = covariance(1,0);//y and x
    poseWithCovarianceStamped.pose.covariance[6 + 1] = covariance(1,1);//y and y

    //R,P,Y
    poseWithCovarianceStamped.pose.covariance[5] = covariance(0,2);//x and theta
    poseWithCovarianceStamped.pose.covariance[11] = covariance(1,2);//y and theta
    poseWithCovarianceStamped.pose.covariance[30] = covariance(2,0);//theta and x
    poseWithCovarianceStamped.pose.covariance[31] = covariance(2,1);//theta and y
    poseWithCovarianceStamped.pose.covariance[35] = covariance(2,2);//theta and theta

    poseWithCovarianceStamped.header.frame_id = "map";
}

void DisplayInterface::displayPoint(const sgbot::Point2D &point) {
    pointStamped.point.x = point.x();
    pointStamped.point.y = point.y();

    pointStamped.header.frame_id = "map";
}

