#include "Robo_control.h"

TeaRobot::TeaRobot()
{
    //Setting up the publishers and Subscribers
    ROS_INFO("Subscribbing to Scanning,Odemetry");
    ros::AsyncSpinner spinner(0);
   
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
        if(FirstPos ==true)
        {
            FirstPos = false;
            //MoveObjs[0]->ObjDect =true;
            
        }
        else
        {   
            //as this is not the first call the learning algorithm should of found the best child
            //this will give us a general direction of travel
            child = LearnObjs[0]->BestChild;
            
            //testing if an object has been detected
            
            
                //obj has been dectected so a new direction need to decided
                //MoveObjs[0]->ObjDect =false;
                //Setting up the movement controller to calculate headings and move
                MoveObjs[0]->ranges=ranges;
                //storing  the children to make sure a code lockup doesnt occur
                
                //only start the movement if both the best child and the children have been found
                if(child.size()>0&&LearnObjs[0]->Re_Learn_Frame.size()>1)
                {
                    
                    
                    //count1 is used to again control the order of execution due to dependancies
                    if(count1>0)
                    {
                        //traveral and main target has been set now to start moving
                        MoveObjs[0]->X=X;
                        MoveObjs[0]->Y=Y;
                        MoveObjs[0]->roll=roll;
                        MoveObjs[0]->pitch=pitch;
                        MoveObjs[0]->yaw=yaw;
                        MoveObjs[0]->w=w;
                        
                        if(MoveObjs[0]->headingSet ==false)
                        {
                            //MoveObjs[0]->WayPoint =child;
                            MoveObjs[0]->HeadingCalc();
                            head = MoveObjs[0]->angle;
                        }
                        if(MoveObjs[0]->headingSet ==true)
                        {
                            
                            if(sqrt(((MoveObjs[0]->WayPoint[0]-X)*(MoveObjs[0]->WayPoint[0]-X)) +((MoveObjs[0]->WayPoint[1]-Y)*(MoveObjs[0]->WayPoint[1]-Y)))>=0.85 )
                            {
                                ROS_INFO("Check Point reached");
                                //calling the node add feature to add the parent and children to the file of nodes
                                
                                count1 =0;
  
                            }
                            if(sqrt((Target[0]-X)*(Target[0]-X) +((Target[1]-Y)*(Target[1]-Y)))<=0.1)
                            {
                                ROS_INFO("End Goal Reached");
                                    
                            }

                            //after this passive obstacle avoidance and the move method is called
                            MoveObjs[0]->Obs_avoid();
                            MoveObjs[0]->Move();
                        }
                
                    }
                    else
                    {
                        //as count is 0 it means that the traversal target should be set
                        MoveObjs[0]->WayPoint =child;
                        MoveObjs[0]->EndPoint = Target;
                        std::cout<<"child "<<child[0]<<" "<<child[1]<<std::endl; 
                        count1++;
                    }

                }
                else
                {
                    std::cout<<"no valid children"<<std::endl;

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
        Re_learn *Learning = new Re_learn(Target);
        LearnObjs.push_back(Learning);
        LearnObjs[0]->FindChildren(StartX,StartY,ranges,yaw);
        count++;
    }
    else
    {
        LearnObjs[0]->FindChildren(X,Y,ranges,yaw);
    }
};
