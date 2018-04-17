//
// Created by root on 18-4-17.
//

#ifndef RVIZ_DISPLAY_INTERFACE_DISPLAYINTERFACE_H
#define RVIZ_DISPLAY_INTERFACE_DISPLAYINTERFACE_H
//sgbot
#include <sensor/lidar2d.h>
#include <type/pose2d.h>
#include <type/point2d.h>
#include <linear-algebra/matrix.h>
//ros
#include <ros/node_handle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class DisplayInterface {
public:
    DisplayInterface(ros::NodeHandle nh = ros::NodeHandle("~")) : nh_(nh){
        scan_publisher = nh_.advertise<sensor_msgs::LaserScan>("LaserScan",1);
        //rviz can only subscribe PoseStamped
        pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>("Pose",1);
        //the same
        pose_covariance_publisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("PoseWithCovariance",1);
        point_publisher = nh_.advertise<geometry_msgs::PointStamped>("Point",1);
    }

    virtual ~DisplayInterface(){
        ROS_INFO("reset");
        reset();
    }

    void displayScan(const sgbot::sensor::Lidar2D& scan);
    void displayPose(const sgbot::Pose2D& pose);
    void displayPoseWithCovariance(const sgbot::Pose2D& pose,const sgbot::la::Matrix<float,3,3>& covariance);
    void displayPoint(const sgbot::Point2D& point);

    void update(){
        scan_publisher.publish(laserScan);
        pose_publisher.publish(geo_pose);
        pose_covariance_publisher.publish(poseWithCovarianceStamped);
        point_publisher.publish(pointStamped);
    }
    void reset(){
        scan_publisher.publish(sensor_msgs::LaserScan());
        pose_publisher.publish(geometry_msgs::PoseStamped());
        pose_covariance_publisher.publish(geometry_msgs::PoseWithCovarianceStamped());
        point_publisher.publish(geometry_msgs::PointStamped());
    }
private:
    ros::Publisher scan_publisher;
    ros::Publisher pose_publisher;
    ros::Publisher pose_covariance_publisher;
    ros::Publisher point_publisher;
//    const std::string scan_topic_name,pose_topic_name,pose_covariance_topic_name,point_topic_name;
    ros::NodeHandle nh_;

    sensor_msgs::LaserScan laserScan;
    geometry_msgs::PoseStamped geo_pose;
    geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;
    geometry_msgs::PointStamped pointStamped;

};


#endif //RVIZ_DISPLAY_INTERFACE_DISPLAYINTERFACE_H
