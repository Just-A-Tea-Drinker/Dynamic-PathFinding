#include "Headers.h"


class Re_learn
{
    public:
    //members
    std::vector<std::vector<float>>Re_Learn_Frame;
    std::vector<float> BestChild,prevChild;
    std::vector<std::vector<float>>Visited,forbidden;
    bool toregress = false;
    std::vector<float>Target;
    //con/de/structor
    Re_learn();
    Re_learn(std::vector<float>Target,std::vector<float>begin);
    ~Re_learn();

    //methods
    void FindChildren(float x,float y,std::vector<float>ranges,double yaw);
    void GetBestChild();
    
    private:
    //members
    std::vector<float>Smallest_Ranges,Start;
    //std::vector<float>Target;
    std::vector<std::vector<float>>Relevant_Laser;
    //members used for several functios 
    int VecIn;
    float X,Y;
    double Yaw;
    float distance,AvgRange;
    int count2 = 0;
    int count3= 0;
    int index = -1;
    float Max_Dist;


    //methods
    float GetMetric(float estX,float estY);
    std::vector<float> CordTransform();
    
    
    




};