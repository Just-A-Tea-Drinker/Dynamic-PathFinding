#include "Re_learn.h"

Re_learn::Re_learn(){};
Re_learn::~Re_learn()
{
    ROS_INFO("RE Learning controller closed");
};
Re_learn::Re_learn(std::vector<float>target,std::vector<float>begin)
{
    ROS_INFO("RE Learning controller active");
    Target = target;
    Start = begin;
};

void Re_learn::FindChildren(float x,float y,std::vector<float>ranges,double yaw)
{
    
    //calcualting the maximum theoretical distance using manhatten distance
    if(count2<1)
    {
        if(x > Target[0])
        {
            if(y> Target[1])
            {
                Max_Dist = (x-Target[0])+(y-Target[1]);
            }
            else
            {
                Max_Dist = (x-Target[0])+(Target[1]-y);
            }
        }
        else
        {
            if(y> Target[1])
            {
                Max_Dist = (Target[0]-x)+(y-Target[1]);
            }
            else
            {
                Max_Dist = (Target[0]-x)+(Target[1]-y);
            }
        }
    count2++;
    }
    
    //finding the best child in the robots surroundings 
    Re_Learn_Frame.clear();
    Re_Learn_Frame.push_back({x,y});
    Smallest_Ranges = ranges;
    Relevant_Laser.clear();
    //resetting values
    VecIn = 0;
    X = x;
    Y = y;
    Yaw = yaw;
    for(int index = 0; index<Smallest_Ranges.size();index++)
    {   
        VecIn = index;
        distance = Smallest_Ranges[index];
        Relevant_Laser.push_back(CordTransform());
    }
    
    for(int index = 0; index<Smallest_Ranges.size();index++)
    {
        if(isgreaterequal(Smallest_Ranges[index],1.0f))
        {
            //setting some values for the other methods
            VecIn = index;
            distance = 0.75f;
            //calculating the estimate co-ordinates using trig
            std::vector<float>Estimates = CordTransform();

            //child is added to a list to be processed later
            Re_Learn_Frame.push_back({Estimates[0],Estimates[1],GetMetric(Estimates[0],Estimates[1])});
        }
    }
    //calculating what the best child is and whether or not its valid
    //iterating over all the laser values
    for(int i = 0;i <Relevant_Laser.size();i++)
    {
        //iterating over all the children
        for(int j = 0;j<Re_Learn_Frame.size();j++)
        {
            //testing the the euclidean distance to be <0.4 to delete invalid children
            if(sqrt(((Relevant_Laser[i][0]-Re_Learn_Frame[j][0])*(Relevant_Laser[i][0]-Re_Learn_Frame[j][0]))+((Relevant_Laser[i][1]-Re_Learn_Frame[j][1])*(Relevant_Laser[i][1]-Re_Learn_Frame[j][1])))<=0.4f)
            {
                Re_Learn_Frame.erase(Re_Learn_Frame.begin()+j);
                
            }
        }  
    }
    if(toregress == false)
    {
        //cehcking all the children to see whether or not they are in the forbidden
        for(std::vector<float>node: Visited)
        {
            for(int child = 1;child<Re_Learn_Frame.size();child++)
            {
                if(sqrt(((Re_Learn_Frame[child][0]-node[0])*(Re_Learn_Frame[child][0]-node[0]))+((Re_Learn_Frame[child][1]-node[1])*(Re_Learn_Frame[child][1]-node[1])))<2.5)
                {
                    Re_Learn_Frame.erase(Re_Learn_Frame.begin()+child);
                   
                  
                   
                }
            }
        }
         for(std::vector<float>node: forbidden)
        {
            for(int child = 1;child<Re_Learn_Frame.size();child++)
            {
                if(sqrt(((Re_Learn_Frame[child][0]-node[0])*(Re_Learn_Frame[child][0]-node[0]))+((Re_Learn_Frame[child][1]-node[1])*(Re_Learn_Frame[child][1]-node[1])))<1)
                {
                    Re_Learn_Frame.erase(Re_Learn_Frame.begin()+child);
                  
                   
                }
            }
        }
    }
    
    GetBestChild();
    Re_frame = Re_Learn_Frame;
 
};

std::vector<float> Re_learn::CordTransform()
{
    //based on the rotation/index the estimated co ordinates are calculated in 2 ways based  on index of the laser scan
    if(VecIn<90&&VecIn>=0||VecIn>=269&&VecIn<360)
    {
        float estiX = X - (distance*cos((VecIn*0.01750139333307743)+Yaw));
        float estiY = Y - (distance*sin((VecIn*0.01750139333307743)+Yaw));
        return {estiX,estiY};    
    }
    else
    {
        float estiX = X + (-distance*cos((VecIn*0.01750139333307743)+Yaw));
        float estiY = Y + (-distance*sin((VecIn*0.01750139333307743)+Yaw));
        return {estiX,estiY};      
    }
};
float Re_learn::GetMetric(float estX,float estY)
{
    //creating rewards to be added to valid children
    //manhatten distance
    float man_dist;
    //making sure the smallest number is subtracted from the larger one
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
    float Dist_Needed = Max_Dist-man_dist;

    
    float ChildToTarget = sqrt(((Target[0]-estX)*(Target[0]-estX))+((Target[1]-estY)+(Target[1]-estY)));

    //metric currently used in a average of the manhatten distance and the euclidean distance found to provide a balance between shortest route and a realistic approach
    
    //using a swtiching metric system that dynamically switches
    if(count3 == 3)
    {
        return ChildToTarget;
    }
    else
    {
        return (man_dist+ChildToTarget)/2;
    }
    
};
void Re_learn::GetBestChild()
{
    //searching through the children for the smallest cost
    for(int child = 1;child<Re_Learn_Frame.size();child++)
    {
       
       if(sqrt(((Target[0]-X)*(Target[0]-X))+((Target[1]-Y)*(Target[1]-Y)))<0.85f)
        {
            BestChild=Target;
            break;
        }
 
            
        //IMPLEMENT FRONT AND BACK ALGORITHM FOR A BIT MORE SPEED
        //defualt setting is the first child
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
    std::vector<float> target_temp = Target;
    Target = Start;
    float start_dist = GetMetric(BestChild[0],BestChild[1]);
    Target = target_temp;
   
    for(std::vector<float>node:forbidden)
    {
        if((sqrt(((node[0]-BestChild[0])*(node[0]-BestChild[0]))+((node[1]-BestChild[1])*(node[1]-BestChild[1])))<0.5f)&&(BestChild[2]>start_dist))
        {
                
            toregress = true;
            count3++;
            
        }  
    }
        
    if(forbidden.size()>1)
    {
        if((BestChild[2]>forbidden[forbidden.size()-1][2])&&(BestChild[2]>start_dist))
        {
            toregress = true;
            count3++;
        }
       

    }
    else
    {
        toregress = false;
    }

};
void Re_learn::ControlNodes()
{
    //a small method used for dealing with the forbidden list after the regression has been completed
    for(std::vector<float>node:forbidden)
    {
        Visited.push_back(node);
    }
    //clearing forbidden for the next run
    forbidden.clear();

};
