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
    
    //std::cout<<conyaw<<std::endl;
    
    //finding the best child in the robots surroundings 
    Re_Learn_Frame.clear();
    
    //Re_Learn_Frame.push_back({x,y});
    Smallest_Ranges = ranges;
    
    for(int index = 0; index<Smallest_Ranges.size();index++)
    {
        if(isgreaterequal(Smallest_Ranges[index],1.0f))
        {

            //adding the co-oridnates of the laser beam as well as the children with metrics
            Relevant_Laser.push_back(CordTransform(Smallest_Ranges[index],index,x,y,yaw));
            //calculating the estimate co-ordinates using trig
            std::vector<float>Estimates = CordTransform(0.5,index,x,y,yaw);
            Re_Learn_Frame.push_back({Estimates[0],Estimates[1],GetMetric(Estimates[0],Estimates[1])});
            
        }
        
        
      
    }
    //calculating what the best child is and whether or not its valid
    //iterating over all the laser values and if one is 
    for(int i = 0;i <Relevant_Laser.size();i++)
    {
        for(int j = 0;j<Re_Learn_Frame.size();j++)
        {
            if(sqrt(((Relevant_Laser[i][0]-Re_Learn_Frame[j][0])*(Relevant_Laser[i][0]-Re_Learn_Frame[j][0]))+((Relevant_Laser[i][1]-Re_Learn_Frame[j][1])*(Relevant_Laser[i][1]-Re_Learn_Frame[j][1])))<=0.4f)
            {
                Re_Learn_Frame.erase(Re_Learn_Frame.begin()+j);
                i=0;
                break;
            }
        }
        
    }
    //std::cout<<Re_Learn_Frame.size()<<std::endl;
    GetBestChild();
    //std::cout<<"child check complete"<<std::endl;
    
    
};
bool Re_learn::WillItFit(float range,int index,float Curx,float Cury,float estX,float estY,float yaw)
{
    //testing whether or not the robot will fit even if the range is adquate

    //testing if the euclidean distance between the laser point converted into cartesian space is more than 0.25 away in straight line
    std::vector<float> laserPoint = CordTransform(range,index,Curx,Cury,yaw);
    //calculating the euclidean distance between them and returnng true or false based on it (dist> 0.25)
    if(isgreaterequal(sqrt(((laserPoint[0]-estX)*(laserPoint[0]-estX))+((laserPoint[1]-estY)*(laserPoint[1]-estY))),1.0f))
    {
        std::cout<<sqrt(((laserPoint[0]-estX)*(laserPoint[0]-estX))+((laserPoint[1]-estY)*(laserPoint[1]-estY)))<<std::endl;
        return true;

    }
    else
    {
        return false;
    }

    

};
void Re_learn::GetValidLaser()
{
    //calculating whether or not the lasers around the child will make a difference to fitting

};
std::vector<float> Re_learn::CordTransform(float distance,int index,float x,float y,float yaw)
{
    if(index<90&&index>=0||index>=269&&index<360)
    {
        float estiX = x - (distance*cos((index*0.01750139333307743)+yaw));
        float estiY = y - (distance*sin((index*0.01750139333307743)+yaw));
        return {estiX,estiY};  
        //testing whether or not the new point is a certain distance from the obstacle so the robot fits
        
                
    }
    else
    {
        float estiX = x + (-distance*cos((index*0.01750139333307743)+yaw));
        float estiY = y + (-distance*sin((index*0.01750139333307743)+yaw));
        return {estiX,estiY};      
    }
};
float Re_learn::GetMetric(float estX,float estY)
{
    //creating rewards to be added to valid children
    //manhatten distance
    float man_dist;
    //man_dist = (Target[0]-estX)+(Target[1]-estY);
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
                //std::cout<<child<<std::endl;
            }
        }
    }

}
