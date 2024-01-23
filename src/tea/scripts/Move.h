#include "Headers.h"

class Movement
{
    public:
    //member
    std::vector<float>ranges,WayPoint,EndPoint;
    float heading,X,Y,angle;
    bool headingSet,ObjDect,pathDect;
    double x,y,z,w;
    double roll,pitch,yaw;
    float Ang_Vel,Lin_Vel,way_dist;
    //con/de/structors
    Movement();
    ~Movement();

    //methods
    void Move();
    void Obs_avoid();
    void HeadingCalc();
    float ModCalc(float angle,float scale);

    private:
    //ros
    ros::NodeHandle nh;
    ros::Publisher velPub;
    //variables
    int count;
    
};