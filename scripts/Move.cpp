#include "Move.h"

Movement::Movement()
{
    //setting up the topic publisher
    ROS_INFO("Movement controller active");
    ROS_INFO("Publishing to twist");
    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    Ang_Vel = 0.0f;
    Lin_Vel = 0.0f;
    headingSet = false;
    //whilst no object has been dected this is needed to start the traversal
    ObjDect = false;
    pathDect = false;
    count = 0;
};
Movement::~Movement()
{
    ROS_INFO("Movement controller closed");
};

void Movement::Move()
{
    //simply used to compile the message and send the message to the twist topic
    geometry_msgs::Twist msg;
    msg.linear.x = Lin_Vel;
    msg.angular.z= Ang_Vel;
    velPub.publish(msg);
};
//method used for navigating each node without collisions
void Movement::Obs_avoid()
{
    
    //testing whether or not the target has been found
    if(ObjDect==false&&sqrt((EndPoint[0]-X)*(EndPoint[0]-X) +((EndPoint[1]-Y)*(EndPoint[1]-Y)))<=0.1)
    {
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
        headingSet = false;
        Move();
        return;

    }
    //testing making sure the check point is checked
    if(ObjDect==false&&sqrt(((WayPoint[0]-X)*(WayPoint[0]-X)) +((WayPoint[1]-Y)*(WayPoint[1]-Y)))>=0.85)
    {
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
        headingSet = false;
        //ObjDect = true;
        return;
    }
    //testing whether or not the target has been found
    if(ObjDect==true&&sqrt((EndPoint[0]-X)*(EndPoint[0]-X) +((EndPoint[1]-Y)*(EndPoint[1]-Y)))<=0.1)
    {
        //ROS_INFO("Target Found");
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
        headingSet = false;
        Move();
        return;

    }
    if(ObjDect==true&&sqrt(((WayPoint[0]-X)*(WayPoint[0]-X)) +((WayPoint[1]-Y)*(WayPoint[1]-Y)))<=0.1)
    {
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
        headingSet = false;
        //ObjDect = true;
        return;
    }
    //testing making sure the check point is checked
    if(pathDect==true&&sqrt(((WayPoint[0]-X)*(WayPoint[0]-X)) +((WayPoint[1]-Y)*(WayPoint[1]-Y)))>=way_dist+0.1)
    {
        Lin_Vel = 0.0f;
        Ang_Vel = 0.0f;
        headingSet = false;
        //ObjDect = true;
        return;
    }
    //changing vels based on sensor data
    //LEFT
    if(ranges[180]<0.4 && ranges[115]<0.4 && ranges[245]>0.4)
    {
       
        Ang_Vel = 1.0f;
        Lin_Vel = 0.0f;

    }
    //RIGHT
    else if(ranges[180]<0.4 && ranges[115]>0.4&& ranges[245]<0.4)
    {
        
        Ang_Vel = -1.0f;
        Lin_Vel = 0.0f;
    }
    //FOWARDS
    else if(ranges[180]>0.4 && ranges[145]>0.4 && ranges[215]>0.4)
    {
        
        Ang_Vel = 0.0f;
        Lin_Vel = 1.0f;
    }
    
    //FRONT LEFT & RIGHT
    else if(ranges[130]<0.4&& ranges[230]>0.4)
    {
        
        Ang_Vel = 0.5f;
        Lin_Vel = 0.0f;
    }
    else if(ranges[155]>0.4&& ranges[205]<0.4)
    {
        
        Ang_Vel = -0.5f;
        Lin_Vel = 0.0f;
    }
    //CHECKING IF SOMETHING IS DIRRECTLY IN FRONT WITH NOTHING LEFT OR RIGHT
    else if(ranges[180]<0.4&&ranges[178]<0.4)
    {
        //going left
        Ang_Vel = 0.5f;
        Lin_Vel = 0.0f;
    }
    else if(ranges[180]<0.4&&ranges[178]<0.4)
    {
        //going right
        Ang_Vel = -0.5f;
        Lin_Vel = 0.0f;
    }
    //ADJ FOR SIDES
    else if(ranges[180]>0.4 && ranges[90]<0.4 && ranges[270]>0.4)
    {
        
        Ang_Vel+=0.3f;

    }
    else if(ranges[0]>0.4 && ranges[90]>0.4 && ranges[270]<0.4)
    {
        
        Ang_Vel-=0.3f;
        
    }
    //if movement not possible
    else if(ranges[180]<0.4&&headingSet ==true&&ranges[225]<0.4&&ranges[135]<0.4 )
    {
        std::cout<<"Movement not possible"<<std::endl;
        Ang_Vel += 0;
        Lin_Vel = 0.0;
        headingSet = false;
        
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
    //angle = ModCalc((angle+1.5707f),3.1415f)-1.5707f;
    heading = 0.5*(angle-yaw);
    Ang_Vel = heading;
    
    if(int(angle*100)==int(yaw*100))
    {
 
        Move();
        Ang_Vel = 0.0f;
        headingSet = true;
        count=0;
       
        
    }
    else
    {
        if(count!=500)
        {
            count++;
        }
        else
        {
            std::cout<<"Inactivitiy detected forcing a move"<<std::endl;
            count=0;
            Ang_Vel = 0.0f;
            headingSet = true;
                
        }
        Move();
        
    }
   
    
    
    
};