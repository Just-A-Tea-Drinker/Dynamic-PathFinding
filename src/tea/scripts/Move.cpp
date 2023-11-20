#include "Move.h"

Movement::Movement()
{
    ROS_INFO("Movement controller active");
    ROS_INFO("Publishing to twist");
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    Ang_Vel = 0.0f;
    Lin_Vel = 0.0f;
    headingSet = false;
};
Movement::~Movement()
{
    ROS_INFO("Movement controller closed");
};

void Movement::Move()
{
    geometry_msgs::Twist msg;
    msg.linear.x = Lin_Vel;
    msg.angular.z= Ang_Vel;
    velPub.publish(msg);
};
//method used for navigating each node without collisions
void Movement::Obs_avoid()
{
    std::cout<<"Child "<<WayPoint[0]<<" "<<WayPoint[1]<<std::endl;
    std::cout<<"Current position "<<X<<" "<<Y<<std::endl;
    //std::cout<<<<std::endl;
    if(WayPoint[0]==int(X) && WayPoint[1]==100)
    {
        ROS_INFO("Node Reached");
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
    }
    else if(ranges[9]<0.6 && ranges[5]<0.6 && ranges[13]>0.6)
    {
        //ROS_INFO("TURNING left");
        Ang_Vel = 1.0f;
        Lin_Vel = 0.0f;
                

    }
    //RIGHT
    else if(ranges[9]<0.6 && ranges[5]>0.6&& ranges[13]<0.6)
    {
        //ROS_INFO("TURNING right");
        Ang_Vel = -1.0f;
        Lin_Vel = 0.0f;
                
    }
    //FOWARDS
    else if(ranges[9]>0.6 && ranges[6]>0.6 && ranges[12]>0.6)
    {
        //ROS_INFO("FORWAD");
        Ang_Vel = 0.0f;
        Lin_Vel = 1.0f;
    }
    //FRONT LEFT & RIGHT
    else if(ranges[5]<0.6&& ranges[13]>0.6)
    {
        //ROS_INFO("FRONT LEFT");
        Ang_Vel = 0.5f;
        Lin_Vel = 0.0f;
                
    }
    else if(ranges[7]>0.6&& ranges[11]<0.6)
    {
        //ROS_INFO("FRONT RIGHT");
        Ang_Vel = -0.5f;
        Lin_Vel = 0.0f;
                
    }
            
    //ADJ FOR SIDES
    else if(ranges[9]>0.6 && ranges[4]<0.6 && ranges[14]>0.6)
    {
        //ROS_INFO("AD L");
        Ang_Vel+=0.3f;
                

    }
    else if(ranges[0]>0.6 && ranges[4]>0.6 && ranges[14]<0.6)
    {
        //ROS_INFO("AD L");
        Ang_Vel-=0.3f;
                
                
    }
};

 float Movement::ModCalc(float a, float b)
{
    float mod;
    // Handling negative values
    if (a < 0)
        mod = -a;
    else
        mod =  a;
    if (b < 0)
        b = -b;
 
    // Finding mod by repeated subtraction
     
    while (mod >= b)
        mod = mod - b;
 
    // Sign of result typically depends
    // on sign of a.
    if (a < 0)
        return -mod;
 
    return mod;
}
 

void Movement::HeadingCalc()
{
    
    float x_change,y_change;
    headingSet = false;
    x_change = WayPoint[0]-X;
    y_change = WayPoint[1]-Y;
    angle = atan2(y_change,x_change);
    angle = ModCalc((angle+1.5707f),3.1415f)-1.5707f;
    heading = 0.5*(angle-yaw);
    Ang_Vel = heading;
    //std::cout<<"yaw: "<<yaw<<std::endl;
    //std::cout<<"ang: "<<angle<<std::endl;
    //std::cout<<fmod(angle,yaw)<<std::endl;
    if(int(angle*100)==int(yaw*100))
    {
 
        Move();
        Ang_Vel = 0.0f;
        headingSet = true;
        
    }
    else
    {
        Move();
    }
   
    
    
    
};