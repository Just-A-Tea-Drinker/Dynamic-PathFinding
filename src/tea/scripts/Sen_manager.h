//#pragma once
#include "NAV_manager.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>
#include <functional>
class Sen_manager
{
    public:
    //variables
    std::vector<float>Target;
    std::vector<std::vector<float>>Nodes;
    std::vector<float>ranges;
    std::vector<std::vector<float>>NewRanges;
    std::vector<float>Smallest_ranges;

    // de/con/structor
    Sen_manager();
    ~Sen_manager();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void Move();
    void Obs_Checking();
    void vectorSplit();
    
    private:
    
    float Lin_Vel;
    float Ang_Vel;
    ros::NodeHandle nh;
    ros::Subscriber Position;
    ros::Subscriber ScanSub;
    ros::Publisher velPub;

};