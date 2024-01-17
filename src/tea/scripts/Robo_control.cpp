#include "Robo_control.h"

TeaRobot::TeaRobot()
{
    //Setting up the publishers and Subscribers
    ROS_INFO("Subscribbing to Scanning,Odemetry");
    ros::AsyncSpinner spinner(0);
   
    ROS_INFO("Subscribbed!");

    ROS_INFO("Setting up controllers");
    NavObjs= std::make_unique<NAV_manager>();
    //NAV_manager *GetInfo = new NAV_manager();
    MoveObjs= std::make_unique<Movement>();
 
    //backing up pointers
    //NavObjs=GetInfo;
    //MoveObjs.=Move_control;
    
    //extracting information from activated classes stored as pointers
    Target = NavObjs->Target;
    Nodes = NavObjs->Node_contents;

    ROS_INFO("Target found x: %f y: %f",Target[0],Target[1]);
    //actually starting the nodes  down here due to the multithread mechanic
    spinner.start();
    
    ScanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",1,&TeaRobot::ScanCallback,this);
    Position =nh.subscribe<nav_msgs::Odometry>("/odom",1,&TeaRobot::PosCallback,this);
    
    ros::waitForShutdown();
};
TeaRobot::~TeaRobot()
{
    ROS_INFO("Robot was shut down sucessfully");
};

void TeaRobot::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //taking and storing the /scan data
    ranges = msg->ranges;
    //the learning algorithm has a dependancy on /odom, so it must wait untill the odom is giving msg
    
    if(ranges.size()>1&&FirstPos == false)
    {
        learnStart();
    }
 
};
void TeaRobot::PosCallback(const nav_msgs::Odometry::ConstPtr& msg)
{   
    
    //taking the odom data if the odom is coming from the main robot
    if(msg->child_frame_id == "robot_footprint")
    {
        //taking the position and orientation of the robot
        StartX =  msg->pose.pose.position.x;
        StartY = msg->pose.pose.position.y;
        X =  msg->pose.pose.position.x;
        Y = msg->pose.pose.position.y;
        x = msg->pose.pose.orientation.x;
        y = msg->pose.pose.orientation.y;
        z = msg->pose.pose.orientation.z;
        w = msg->pose.pose.orientation.w;

        //converting the quaternion into eular
        tf::Quaternion q(x, y, z, w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        //checking if this is the first function call
        //makes sure that the range data has had the time to be called
        if(FirstPos ==true)
        {
            FirstPos = false;
  
        }
        else
        {   
            //setting up the control code to only take orders when following a path of nodes
            if(SearchObj->usePath==true)
            {
                //this is where the control of what nodes need to be travelled to
                //placed in method for readability
                PathFollow();
            }
            else
            {
                //this is the code responsible for 'blindly' but 'informed' moving decisions
                //placed in a method for readability
                explore();
            }
            
        }
    } 
};

void TeaRobot::learnStart()
{
    //using the complete 360 range data
    for(int i =0;i<ranges.size();i++)
    {
        if(isinf(ranges[i]))
        {
            ranges[i] = 5.0f;
        }
    }
    
    if(count<1)
    {
        START.push_back(StartX);
        START.push_back(StartY);
        //activating the A_search if there are nodes
        SearchObj = std::make_unique<A_search>(START,Target,NavObjs->Node_contents);
        //activate the reinforcement learning learning section of the algorithm
        
        LearnObjs=std::make_unique<Re_learn>(Target,START);
        LearnObjs->FindChildren(StartX,StartY,ranges,yaw);
        
        count++;
    }
    else
    {
        LearnObjs->FindChildren(X,Y,ranges,yaw);
    }
};
void TeaRobot::explore()
{
    //As this is not the first call the learning algorithm should of found the best child
    //This will give us a general direction of travel
    child = LearnObjs->BestChild;
    MoveObjs->ranges=ranges;
                    
    //only start the movement if both the best child and the children have been found
    if(child.size()>0&&LearnObjs->Re_frame.size()>1)
    {

        //count1 is used to again control the order of execution due to dependancies
        if(count1>0)
        {
            //traveral and main target has been set now to start moving
            MoveObjs->X=X;
            MoveObjs->Y=Y;
            MoveObjs->roll=roll;
            MoveObjs->pitch=pitch;
            MoveObjs->yaw=yaw;
            MoveObjs->w=w;
                    
            //if the heading is false then the heading is calculated
            //adding the node into a collection nodes seemingly unlinked whereby they are stitched together to form a cartesian ish map
                        
                    
            if(MoveObjs->headingSet ==false)
            {
                MoveObjs->HeadingCalc();
                head = MoveObjs->angle;
            }
            if(MoveObjs->headingSet ==true)
            {
                            
                if(sqrt(((MoveObjs->WayPoint[0]-X)*(MoveObjs->WayPoint[0]-X)) +((MoveObjs->WayPoint[1]-Y)*(MoveObjs->WayPoint[1]-Y)))>=0.85 )
                {
                    ROS_INFO("Check Point reached");
                    count1 =0;
                                
                    
                    if(LearnObjs->toregress == false)
                    {
                        LearnObjs->forbidden=prevChild;
                    }
                                
                    frame = LearnObjs->Re_frame;
                    if(LearnObjs->toregress == false)
                    {
                        NavObjs->Node_Add(frame);
                                
                                    
                    }
                                

    
                }
                if(sqrt((Target[0]-X)*(Target[0]-X) +((Target[1]-Y)*(Target[1]-Y)))<=0.1)
                {
                    ROS_INFO("End Goal Reached");
                    MoveObjs->Lin_Vel = 0.0f;
                    MoveObjs->Ang_Vel = 0.0f;
                    MoveObjs->Move();
                    std::cout<<"Optimising my Waypoints"<<std::endl;
                    NavObjs->Node_Opti();
                    std::cout<<"Waypoints adjusted"<<std::endl;
                    NavObjs->Node_WriteFormat();
                    NavObjs->Found_Target();
                    //if the program reaches here the user has decided to continue new parameters are updated
                    if(NavObjs->closing==true)
                    {
                        return;
                    }
                        else
                        {
                            START.clear();
                            START.push_back(StartX);
                            START.push_back(StartY);
                            LearnObjs->Start = START;
                            Target = NavObjs->Target;
                            count1 = 0;
                        }
                                


                }

                //after this passive obstacle avoidance and the move method is called
                MoveObjs->Obs_avoid();
                MoveObjs->Move();
            }
            if(LearnObjs->toregress == true)
            {
                if(sqrt((START[0]-X)*(START[0]-X) +((START[1]-Y)*(START[1]-Y)))<=0.1)
                {
                    std::cout<<"regrressed to start"<<std::endl;
                    MoveObjs->Lin_Vel = 0.0f;
                    MoveObjs->Ang_Vel = 0.0f;
                    MoveObjs->Move();
                    //setting the parameters       
                    LearnObjs->toregress = false;
                    MoveObjs->ObjDect = false;
                    LearnObjs->Target =Target;
                    MoveObjs->headingSet=false;
                    std::cout<<"target "<<LearnObjs->Target[0]<<" "<<LearnObjs->Target[1]<<std::endl; 
                    MoveObjs->WayPoint =Target;

                    //calling the node controller in re learn
                    LearnObjs->ControlNodes();
                    prevChild.clear();
                                
                                

                }
            }
                    
        }
        else
        {
            //as count is 0 it means that the traversal target should be set
            if(LearnObjs->toregress == false)
            {   
                std::cout<<"Conducting movement"<<std::endl;
                MoveObjs->EndPoint = Target;
                LearnObjs->Target =Target;
                MoveObjs->WayPoint =child;
                prevChild.push_back(child);
                            
                            
                            
            }
            else
            {
                std::cout<<"Conducting a regression"<<std::endl;
                MoveObjs->headingSet =false;
                LearnObjs->Target =START;
                MoveObjs->WayPoint = child;
                if(child[0]==START[0]&&child[1]==START[1])
                {
                    MoveObjs->ObjDect=true;
                }
        
            }
                        
            std::cout<<"child "<<child[0]<<" "<<child[1]<<std::endl; 
            count1++;
                        
        }

    }
    //kind of irrelevant really, but a rough guide to
    else
    {
        std::cout<<"No info has returned from scanning or learning...Waiting response"<<std::endl;
                    
    }
};
void TeaRobot::PathFollow()
{
    //This will be the control code for making the robot follow a path based on what "its' learnt on previous runs"

    //pulling the calculated path
    path = SearchObj->Path;
    //making the child a step of the path
    child = path[step];

    //setting up the robot head towards the target
    
};