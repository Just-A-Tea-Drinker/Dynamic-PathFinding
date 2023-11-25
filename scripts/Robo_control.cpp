#include "Robo_control.h"

TeaRobot::TeaRobot()
{
    //Setting up the publishers and Subscribers
    ROS_INFO("Subscribbing to Scanning,Odemetry");
    ros::AsyncSpinner spinner(0);
   
    //ros::spinOnce();
    ROS_INFO("Subscribbed!");

    ROS_INFO("Setting up controllers");
    NAV_manager *GetInfo = new NAV_manager();
    Movement *Move_control = new Movement();
    
    //backing up pointers
    NavObjs.push_back(GetInfo);
    MoveObjs.push_back(Move_control);
    
    //extracting information from activated classes stored as pointers
    Target = GetInfo->Target;
    Nodes = GetInfo->Node_contents;

    
    //std::cout<<"best child found at x: "<<LearnObjs[0]->BestChild[0]<<" y:"<<LearnObjs[1]<<std::endl;
    ROS_INFO("Target found x: %f y: %f",Target[0],Target[1]);
    //actually starting the nodes  down here due to the multithread mechanic
    spinner.start();
    
    ScanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",1,&TeaRobot::ScanCallback,this);
    Position =nh.subscribe<nav_msgs::Odometry>("/odom",1,&TeaRobot::PosCallback,this);
    
    ros::waitForShutdown();
};
TeaRobot::~TeaRobot()
{};

void TeaRobot::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
    Smallest_ranges.clear();
    NewRanges.clear();
    conversion.clear();
    ranges = msg->ranges;
    increment = msg->angle_increment;
    if(ranges.size()>1&&FirstPos == false)
    {
        
        learnStart();
        
    }
    
    
    
    
};
void TeaRobot::PosCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    
    //clearing the previous frame
    if(msg->child_frame_id == "robot_footprint")
    {
        //ROS_INFO("position(x,y)=(%f,%f",msg->pose.pose.position.x,msg->pose.pose.position.y);
        StartX =  msg->pose.pose.position.x;
        StartY = msg->pose.pose.position.y;
        X =  msg->pose.pose.position.x;
        Y = msg->pose.pose.position.y;
        x = msg->pose.pose.orientation.x;
        y = msg->pose.pose.orientation.y;
        z = msg->pose.pose.orientation.z;
        w = msg->pose.pose.orientation.w;
        tf::Quaternion q(x, y, z, w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        if(FirstPos ==true)
        {
            
            //std::cout<<"first call"<<std::endl;
            FirstPos = false;
        }
        else
        {
            
       
        
            child = LearnObjs[0]->BestChild;
            
            if(child.size()>0&&LearnObjs[0]->Re_Learn_Frame.size()>1)
            {
                //Setting up the movement controller to calculate headings and move
                
                MoveObjs[0]->ranges=ranges;
                if(count1>0)
                {
                    //std::cout<<"child "<<child[0]<<" "<<child[1]<<std::endl;
                    //std::cout<<"count "<<count1<<std::endl;
                    //std::cout<<"child "<<MoveObjs[0]->WayPoint[0]<<" "<<MoveObjs[0]->WayPoint[1]<<std::endl; 
                    MoveObjs[0]->X=X;
                    MoveObjs[0]->Y=Y;
                    MoveObjs[0]->roll=roll;
                    MoveObjs[0]->pitch=pitch;
                    MoveObjs[0]->yaw=yaw;
                    MoveObjs[0]->w=w;
                    
                    if(MoveObjs[0]->headingSet ==false)
                    {
                        MoveObjs[0]->HeadingCalc();
                        head = MoveObjs[0]->angle;
                    }
                    if(MoveObjs[0]->headingSet ==true)
                    {
                        
                        if(sqrt((MoveObjs[0]->WayPoint[0]-X)*(MoveObjs[0]->WayPoint[0]-X) +((MoveObjs[0]->WayPoint[1]-Y)*(MoveObjs[0]->WayPoint[1]-Y)))<=0.05)
                        {
                            ROS_INFO("NODE REACHED");
                            count1 = 0;
                        }
                        if(sqrt((Target[0]-X)*(Target[0]-X) +((Target[1]-Y)*(Target[1]-Y)))<=0.05)
                        {
                            ROS_INFO("End Goal Reached");
                            //count1 = 0;
                        }
                        MoveObjs[0]->Obs_avoid();
                        MoveObjs[0]->Move();
                    }
                    
                }
                else
                {
                   MoveObjs[0]->WayPoint =child;
                   MoveObjs[0]->EndPoint = Target;
                   std::cout<<"child "<<MoveObjs[0]->WayPoint[0]<<" "<<MoveObjs[0]->WayPoint[1]<<std::endl; 
                    
                    count1++;
                }
                
                
                
                

            }
            else
            {
                std::cout<<"no valid children"<<std::endl;
            }
            child.clear();
            }
        }
        
      
    

};
void TeaRobot::learnStart()
{
    //using the complete 360 range data
    
    if(count<1)
    {
        Re_learn *Learning = new Re_learn(Target);
        LearnObjs.push_back(Learning);
    }
    if(count<1)
    {
        LearnObjs[0]->FindChildren(StartX,StartY,ranges,yaw);
        count++;

    }
    else
    {
        LearnObjs[0]->FindChildren(X,Y,ranges,yaw);
    }
    
    
};
