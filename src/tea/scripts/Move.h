#include "Headers.h"

class Movement
{
    public:
    //member
    std::vector<float>ranges,WayPoint,EndPoint;
    float heading,X,Y,angle;
    bool headingSet;
    double x,y,z,w;
    double roll,pitch,yaw;
    float Ang_Vel,Lin_Vel;
    //con/de/structors
    Movement();
    ~Movement();

    //methods
    void Move();
    void Obs_avoid();
    void HeadingCalc();
    float ModCalc(float a,float b);

    private:
    //members
    
    
    //ros
    ros::NodeHandle nh;
    ros::Publisher velPub;
    
};