#include "Robo_control.h"

TeaRobot::TeaRobot()
{
    //Setting up the publishers and Subscribers
    ROS_INFO("Subscribbing to Scanning,Odemetry");
    
    ScanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",10,&TeaRobot::ScanCallback,this);
    Position =nh.subscribe<nav_msgs::Odometry>("/odom",1,&TeaRobot::PosCallback,this);
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
    
};
TeaRobot::~TeaRobot()
{};

void TeaRobot::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //std::cout<<"data"<<std::endl;
    Smallest_ranges.clear();
    NewRanges.clear();
    conversion.clear();
    ranges = msg->ranges;
    vectorSplit();
    
    
    
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
       
    
        child = LearnObjs[0]->BestChild;
        
        if(child.size()>0&&LearnObjs[0]->Re_Learn_Frame.size()>1)
        {
            //Setting up the movement controller to calculate headings and move
            tf::Quaternion q(x, y, z, w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            MoveObjs[0]->ranges=conversion;

            MoveObjs[0]->X=X;
            MoveObjs[0]->Y=Y;
            MoveObjs[0]->roll=roll;
            MoveObjs[0]->pitch=pitch;
            MoveObjs[0]->yaw=yaw;
            MoveObjs[0]->w=w;
            std::cout<<"best child "<< child[0]<<" "<<child[1]<<std::endl;
            if(count1<1)
            {
                std::cout<<"best child "<< child[0]<<" "<<child[1]<<std::endl;
                MoveObjs[0]->WayPoint =child;
                MoveObjs[0]->EndPoint = Target;
                count1++;
            }
            else
            {
                if(MoveObjs[0]->headingSet ==false)
                {
                    MoveObjs[0]->HeadingCalc();
                    head = MoveObjs[0]->angle;
                }
                else if (MoveObjs[0]->headingSet ==true)
                {
                    MoveObjs[0]->Obs_avoid();
                    MoveObjs[0]->Move();
                }
            }
            
            
            
            //std::cout<<head<<std::endl;
            //std::cout<<yaw<<std::endl;
                
            
           
            
 
            
            if(child[0]==X&&child[1]==Y)
            {
                ROS_INFO("NODE REACHED");
            }
            //std::cout<<"best child x: "<<child[0]<<" y: "<<child[1]<<std::endl;
            
            

            
        }
        else
        {
            std::cout<<"no valid children"<<std::endl;
        }
        child.clear();
        
    }  
    

};
void TeaRobot::vectorSplit()
{
    int index = 0;
    
    std::vector<float> set;
    //converting  the 360 into 18 segments of 20 degrees

    for (float i:ranges)
    {
        if(index==17)
        {
            NewRanges.push_back(set);
            set.clear();
            index=0;
        }
        else
        {
            set.push_back(i);
            index++;
        }
        
    }
    
    //finding the smallest values in each segment
    for(int x= 0;x<NewRanges.size();x++)
    {
        
        float mini = *min_element(NewRanges[x].begin(),NewRanges[x].end());
        Smallest_ranges.push_back(mini);
        
    }
    conversion = Smallest_ranges;
    //exception as this has dependencies
    
    if(count<1)
    {
        Re_learn *Learning = new Re_learn(Target);
        LearnObjs.push_back(Learning);
    }
    if(count<1)
    {
        LearnObjs[0]->FindChildren(StartX,StartY,conversion,yaw);
        count++;

    }
    else
    {
        LearnObjs[0]->FindChildren(X,Y,conversion,yaw);
    }
    
};
