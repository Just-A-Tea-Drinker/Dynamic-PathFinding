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
            START.push_back(StartX);
            START.push_back(StartY);
            //activating the A_search if there are nodes
            
            SearchObj = std::make_unique<A_search>(START,Target,Nodes);
            path = SearchObj->Path;
  
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
            if(sqrt((START[0]-X)*(START[0]-X) +((START[1]-Y)*(START[1]-Y)))<=0.1&&LearnObjs->toregress == false)
            {
                NavObjs->Node_Add(LearnObjs->Re_frame); 
            }
                       
                    
            if(MoveObjs->headingSet ==false)
            {
                MoveObjs->HeadingCalc();
                
                
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
                    //adding the last move to the recorded nodes to be stored
                    frame = LearnObjs->Re_frame;
                    if(LearnObjs->toregress == false)
                    {
                        NavObjs->Node_Add(frame);
            
                    }
                    std::cout<<"Optimising my Waypoints"<<std::endl;
                    //NavObjs->Node_Opti();
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
                        MoveObjs->EndPoint = Target;
                        LearnObjs->Target = Target;
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
                    if(LearnObjs->count3 ==3)
                    {
                        LearnObjs->count3 = 0;
                    }
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
            //notifying the user of the next expected waypoint          
            std::cout<<"child "<<child[0]<<" "<<child[1]<<std::endl; 
            count1++;
                        
        }

    }
    //kind of irrelevant really, but a rough guides the user on the terminal to the current execution
    else
    {
        std::cout<<"No info has returned from scanning or learning...Waiting response"<<std::endl;
                    
    }
};
void TeaRobot::PathFollow()
{
    //This will be the control code for making the robot follow a path based on what "its' learnt on previous runs"
    
    
    MoveObjs->ranges=ranges;
    //making sure the path has something in it
    if(path.size()>1)
    {
        
        //checking whether or not the heading is set
        //count2 is used to control the execution
        if(count2>0)
        {
            //traveral and main target has been set now to start moving
            MoveObjs->X=X;
            MoveObjs->Y=Y;
            MoveObjs->roll=roll;
            MoveObjs->pitch=pitch;
            MoveObjs->yaw=yaw;
            MoveObjs->w=w;
            //turning off traditional controls used for the exploration as the movement controll will be high level
            MoveObjs->pathDect = true;
                    
            //if the heading is false then the heading is calculated      
            if(MoveObjs->headingSet ==false)
            {
                MoveObjs->HeadingCalc();
                
            }
            if(MoveObjs->headingSet ==true)
            {
                
                

                //function similar to the exploratory one except this one is used to detect whether the robot is off course          
                if(sqrt(((MoveObjs->WayPoint[0]-X)*(MoveObjs->WayPoint[0]-X)) +((MoveObjs->WayPoint[1]-Y)*(MoveObjs->WayPoint[1]-Y)))>=way_Dist+0.1 )
                {
                    ROS_INFO("Robot off course attempting to correct");
                    //calcualting if its closer to a previous or in the future point
                    if(Step<path.size()-1)
                    {
                        Step++;
                    }
                    else
                    {
                        //re-making start to begin the algorithm
                        START.clear();
                        START.push_back(X);
                        START.push_back(Y);
                        //setting up the search algorithm
                        SearchObj->Start=START;
                        SearchObj->PathOption = false;
                        //calling the  path calculator
                        SearchObj->A_Node_Search();
                        path = SearchObj->Path;
                    }
                    
                    count2=0;

                    
                    
                    
                   
            
                }
                //checking if the target has been reached or not
                if(sqrt((Target[0]-X)*(Target[0]-X) +((Target[1]-Y)*(Target[1]-Y)))<=0.1)
                {
                    ROS_INFO("End Goal Reached");
                    MoveObjs->Lin_Vel = 0.0f;
                    MoveObjs->Ang_Vel = 0.0f;
                    MoveObjs->Move();
                    
                    //end has been reached asking the user whether or not they want to move to a new position
                    NavObjs->Found_Target();
                    //making sure the user wants to continue (fail-safe)
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
                        MoveObjs->EndPoint = Target;
                        LearnObjs->Target = Target;
                        count2 = 0;
                    }
                          


                }
                //checking whether or not the robot could have hit a check point
                if(sqrt(((MoveObjs->WayPoint[0]-X)*(MoveObjs->WayPoint[0]-X)) +((MoveObjs->WayPoint[1]-Y)*(MoveObjs->WayPoint[1]-Y)))<=0.1)
                {
                    ROS_INFO("Path node has been reached");
                    
                    //incrementing the waypoint child
                    if(Step==path.size()-1)
                    {
                        SearchObj->usePath = false;
                    }
                    else
                    {
                        Step++;
                        count2 = 0;
                    }
                    


                }
                
                
                //after this passive obstacle avoidance and the move method is called
                MoveObjs->Obs_avoid();
                MoveObjs->Move();
            }
            
                    
        }
        else
        {
            //as count2 is 0 it means that the traversal target should be set
            std::cout<<"Navigating to next step"<<std::endl;
            MoveObjs->EndPoint = Target;
            
            MoveObjs->WayPoint =path[Step];
            way_Dist = sqrt(((MoveObjs->WayPoint[0]-X)*(MoveObjs->WayPoint[0]-X)) +((MoveObjs->WayPoint[1]-Y)*(MoveObjs->WayPoint[1]-Y)));
            MoveObjs->way_dist = way_Dist;
                    
            std::cout<<"Way point "<<path[Step][0]<<" "<<path[Step][1]<<std::endl; 
            count2++;
                        
        }
    }
    

    
    
};