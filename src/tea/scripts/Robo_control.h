#include "Headers.h"
#include "NAV_manager.h"
#include "Re_learn.h"
#include "Move.h"
#include "A_search.h"



class TeaRobot
{
    public:
    //members 
    std::vector<float>Target;
    //con/de/structor
    TeaRobot();
    ~TeaRobot();

    //call backs for the odom and LIDAR
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void PosCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    //other methods
    void learnStart();
    

    private:
    //ros declarations
    ros::NodeHandle nh;
    ros::Subscriber ScanSub;
    ros::Subscriber Position;
    //pointers to classes
    std::vector<Re_learn*>LearnObjs;
    std::vector<NAV_manager*>NavObjs;
    std::vector<Movement*>MoveObjs;
    std::vector<A_search*>SearchObj;

    std::vector<std::vector<float>>nodes,prevChild;
    //members
    int count = 0;
    int count1 = 0;
    int count2 = 0;
    bool FirstPos = true;
    std::vector<float>START;
    //Nodes
    std::vector<std::vector<float>>Nodes;
    std::vector<std::vector<float>>frame;
    //Scan Data
    std::vector<float>ranges,child;
    //Odom data
    float x,y,z,w,head;
    double roll,yaw,pitch;
    float StartX,StartY;
    float X,Y;
    
    

};