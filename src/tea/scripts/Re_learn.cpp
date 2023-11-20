#include "Re_learn.h"
#include <typeinfo>
Re_learn::Re_learn()
{
    
};
Re_learn::~Re_learn()
{
    ROS_INFO("RE Learning controller closed");
};
Re_learn::Re_learn(std::vector<float>target)
{
    ROS_INFO("RE Learning controller active");
    Target = target;
    
    //FindChildren();
};

void Re_learn::FindChildren(float x,float y,std::vector<float>ranges,double yaw)
{
    float conyaw = yaw*(180/3.1415);
    //std::cout<<conyaw<<std::endl;
    Smallest_Ranges = ranges;
    //testing whether each 20 increment is free at a certain range to get a valid child
    Re_Learn_Frame.clear();
    
    Re_Learn_Frame.push_back({x,y});
    for(int index = 0; index<Smallest_Ranges.size();index++)
    {
       //std::cout<<Smallest_Ranges[index]<<" at "<<(index)<<std::endl;
       if(Smallest_Ranges[index]>1.0f)
        {
             
            //calculating a estimate cordinate using trigonometry 
            float estiX = x + (0.75*cos((index*0.349066)-yaw));
            float estiY = y + (0.75*sin((index*0.349066)-yaw));
            //ROS_INFO("esti x: %f esti y: %f",estiX,estiY);
            Re_Learn_Frame.push_back({estiX,estiY,GetMetric(estiX,estiY)});
            

        }
    }
    GetBestChild();
    
};
float Re_learn::GetMetric(float estX,float estY)
{
    //creating rewards to be added to valid children
    //manhatten distance
    float man_dist;
    if(estX > Target[0])
    {
        if(estY> Target[1])
        {
            man_dist = (estX-Target[0])+(estY-Target[1]);
        }
        else
        {
            man_dist = (estX-Target[0])+(Target[1]-estY);
        }
    }
    else
    {
        if(estY> Target[1])
        {
            man_dist = (Target[0]-estX)+(estY-Target[1]);
        }
        else
        {
            man_dist = (Target[0]-estX)+(Target[1]-estY);
        }
    }

    return man_dist;
};
void Re_learn::GetBestChild()
{
    //searching through the children for the smallest cost
    
    for(int child = 1;child<Re_Learn_Frame.size();child++)
    {
        if(child == 1)
        {
            BestChild = Re_Learn_Frame[child];
        }
        else
        {
            if(Re_Learn_Frame[child][2]<BestChild[2])
            {
                BestChild = Re_Learn_Frame[child];
            }
        }
    }
    
    

}
