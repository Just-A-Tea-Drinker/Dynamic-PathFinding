
#include <ros/ros.h>

#include "Sen_manager.h"
#include "A_search.h"
#include "Re_learn.h"
#include "Robo_control.h"
#include <iostream>
#include <string>
//this is the main condcutor of the controllers


int main(int argc, char** argv)
{
    
    //testing the scan data reading
  
    ros::init(argc, argv, "tea");

    ROS_INFO("Starting Robot");
    Sen_manager Activescan;
    ROS_INFO("Checking for obstacles");
    


    ros::spin();


}