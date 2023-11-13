#include "Sen_manager.h"



void Sen_manager::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    NewRanges.clear();
    vectorSplit();
    Obs_Checking();
    Move();
    

    
};
void Sen_manager::vectorSplit()
{
    int index = 1;
    std::vector<float> set;
    for(float i:ranges)
    {
        if(index%20 ==0)
        {
            NewRanges.push_back(set);
            set.clear();
        }
        else
        {
            set.push_back(i);
        }
        
        index++;
    }
    //changing the  data to be 360 rather than the recorded 720
    for(int x= 0;x<NewRanges.size();x++)
    {
        
        float mini = *min_element(NewRanges[x].begin(),NewRanges[x].end());
        Smallest_ranges.push_back(mini);
        //ROS_INFO("LEN%f,%d",mini,x);
    }
    std::vector<float>conversion;
    
    for(int i=0;i<NewRanges.size(); i+=2)
    {
        if(i == 0)
        {
            
            conversion.push_back((NewRanges[i][0]+NewRanges[i][35])/2);
        }
        else
        {
            conversion.push_back((NewRanges[i][0]+NewRanges[i+1][0])/2);
        }
    }
    Smallest_ranges = conversion;
};
void PosCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    if(msg->child_frame_id == "robot_footprint")
    {
        //ROS_INFO("position(x,y)=(%f,%f",msg->pose.pose.position.x,msg->pose.pose.position.y);
        //BEGGINING THE MOVEMENT 
        
    
    }
   
};

//subscribing to the scan topic and wheel velocity topic to get status
Sen_manager::Sen_manager()
{
    ROS_INFO("Checking Targets");
    NAV_manager GetInfo;
    Target = GetInfo.Target;
    Nodes = GetInfo.Node_contents;
    ROS_INFO("Target found");
    ROS_INFO("Subscribbing to Scanning,Odemetry and Vel");
    Lin_Vel = 0.0f;
    Ang_Vel = 0.0f;
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ScanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&Sen_manager::ScanCallback,this);
    Position =nh.subscribe<nav_msgs::Odometry>("/odom",10,PosCallback);
    ROS_INFO("Subscribbed!");




};
void Sen_manager::Obs_Checking()
{
    //checking the closest value

    // // for(int x =0;x<Smallest_ranges.size();x++)
    // // {
    // //     ROS_INFO("LEN%f,%d",Smallest_ranges[x],x);
    // // }
    //changing vels based on sensor data
    //LEFT
    if(Smallest_ranges[0]<0.6 && Smallest_ranges[14]<0.6 && Smallest_ranges[4]>0.6)
    {
        ROS_INFO("TURNING left");
        Ang_Vel = 1.0f;
        Lin_Vel = 0.0f;

    }
    //RIGHT
    else if(Smallest_ranges[0]<0.6 && Smallest_ranges[14]>0.6&& Smallest_ranges[4]<0.6)
    {
        ROS_INFO("TURNING right");
        Ang_Vel = -1.0f;
        Lin_Vel = 0.0f;
    }
    //FOWARDS
    else if(Smallest_ranges[0]>0.6 && Smallest_ranges[15]>0.6 && Smallest_ranges[3]>0.6)
    {
        ROS_INFO("FORWAD");
        Ang_Vel = 0.0f;
        Lin_Vel = 1.0f;
    }
    //FRONT LEFT & RIGHT
    else if(Smallest_ranges[14]<0.6&& Smallest_ranges[4]>0.6)
    {
        ROS_INFO("FRONT LEFT");
        Ang_Vel = 0.5f;
        Lin_Vel = 0.0f;
    }
    else if(Smallest_ranges[16]>0.6&& Smallest_ranges[2]<0.6)
    {
        ROS_INFO("FRONT RIGHT");
        Ang_Vel = -0.5f;
        Lin_Vel = 0.0f;
    }
    
    //ADJ FOR SIDES
    else if(Smallest_ranges[0]>0.6 && Smallest_ranges[13]<0.6 && Smallest_ranges[5]>0.6)
    {
        ROS_INFO("AD L");
        Ang_Vel+=0.3f;

    }
    else if(Smallest_ranges[0]>0.6 && Smallest_ranges[13]>0.6 && Smallest_ranges[5]<0.6)
    {
        ROS_INFO("AD L");
        Ang_Vel-=0.3f;
        
    }


};
void Sen_manager::Move()
{
    //ROS_INFO("LaserScan (rn,angle)=(%f",msg->ranges[0]);
    
    
    //ROS_INFO("Front: %f",ranges[0]);
    ros::Rate loop_rate(0.75f);
    geometry_msgs::Twist msg;
    while (ros::ok())
    {
        msg.linear.x = Lin_Vel;
        msg.angular.z= Ang_Vel;
        velPub.publish(msg);
        ros::spinOnce();
        //loop_rate.sleep();
    }
};
Sen_manager::~Sen_manager()
{

};
