#include "Headers.h"
#include "Robo_control.h"


//this is the main condcutor of the controllers


int main(int argc, char** argv)
{
    
    //testing the scan data reading
  
    ros::init(argc, argv, "tea");

    ROS_INFO("Starting Robot");
    
    TeaRobot Robot;
    //ros::spin();


}