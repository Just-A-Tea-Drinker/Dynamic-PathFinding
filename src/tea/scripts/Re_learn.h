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

    //methods
    float GetMetric(float estX,float estY);
    
    




};