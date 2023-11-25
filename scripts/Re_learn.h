#include "Headers.h"


class Re_learn
{
    public:
    //members
    std::vector<std::vector<float>>Re_Learn_Frame;
    std::vector<float> BestChild;
    //con/de/structor
    Re_learn();
    Re_learn(std::vector<float>Target);
    ~Re_learn();

    //methods
    void FindChildren(float x,float y,std::vector<float>ranges,double yaw);
    void GetBestChild();
    
    private:
    std::vector<float>Smallest_Ranges;
    std::vector<float>Target;
    std::vector<std::vector<float>>Relevant_Laser;

    //methods
    float GetMetric(float estX,float estY);
    bool WillItFit(float range,int index,float Curx,float Cury,float estX,float estY,float yaw);
    void GetValidLaser();
    std::vector<float> CordTransform(float distance,int index,float X,float Y,float yaw);
    
    




};