#include "A_search.h"


//con/destructor
A_search::A_search(){};
A_search::A_search(std::vector<float>start,std::vector<float>Target,std::vector<std::vector<float>> Nodes)
{
    ROS_INFO("Node traversal activated");
    //defining the start and end of the desired path as well as feeding the class all known nodes
    Start = start;
    End = Target;
    KnownPoints=Nodes;
    usePath = false;
    PathExplore = false;
    Current = start;
    count =0;
    if(Nodes.size()>1)
    {
        Node_trav();
        if(usePath==true)
        {
            A_Node_Search();
        }
        
    }
};
A_search::~A_search()
{
    ROS_INFO("Node traversal terminated");
};
void A_search::Node_trav()
{
    //asking the user whether or not they want to use a preprogrammed path to a target
    bool targetAv = false;
    char ans;
    std::vector<float> temp;
    //testing whether or not a completed path straight to the destination is even possible with the information known
    for(std::vector<float>Node:KnownPoints)
    {
        //testing if the parents or children are the destination
        for(int x = 2;x<Node.size()-2;x+=3)
        {
            //checking if the 'parent' and the first child is the target
            
            if(x==2)
            {
                temp ={Node[0],Node[1]};
                if(temp==End)
                {
                    targetAv = true;
                    break;
                }
                temp ={Node[x],Node[x+1]};
                if(temp==End)
                {
                    targetAv = true;
                    break;
                } 
            }
            //checking all of the children
            else
            {
                temp ={Node[x],Node[x+1]};
                if(temp==End)
                {
                    targetAv = true;
                    break;
                } 
            }
        }
    }
    //checking whether or not the target has been found within the traversed nodes or not
    if(targetAv ==true)
    {
        //asking the user whether or not they would like to traverse along a preprogrammed path
        while(true)
        {
            try
            {
                std::cout<<"The target has been found within the learnt data would you like to use a calculated path? Y/N"<<std::endl;
                std::cin>>ans;
                ans = std::tolower(ans);
                if(ans =='y')
                {
                    
                    usePath=true;
                    PathOption = true;
                    
                    break;;
                }
                else if (ans =='n')
                {
                    
                    usePath=false;
                    break;
                    
                }
                else
                {
                    throw(ans);
                }

            }
            catch(char ans)
            {
                std::cout<<"You have not entered a valid response, please try again."<<std::endl;
            }
        }
        return;
    }
    //case when a path could be parrtially calculated but a true resolve isnt found, has both worst case scenarios as well as best
    //Worst: known points are nowhere near the intended target meaning exploring would be better
    // Best: known points have the target so this route has been traversed before, straight route to destination
    else
    {
        //asking the user if they want to use a hybrid method of traversal using known points and exploration
      while(true)
        {
            try
            {
                std::cout<<"There isnt an already explored path, would you like to try to use data to get as close as possible? Y/N"<<std::endl;
                std::cin>>ans;
                ans = std::tolower(ans);
                if(ans =='y')
                {
                    
                    usePath=true;
                    PathOption = false;
                    //bool used to check that the hybrid method has been activated
                    PathExplore = true;
                    break;
                }
                else if (ans =='n')
                {
                    
                    usePath=false;
                    break;
                    
                }
                else
                {
                    throw(ans);
                }

            }
            catch(char ans)
            {
                std::cout<<"You have not entered a valid response, please try again."<<std::endl;
            }
        }
        return;  
    }
    
}




//Method used for finding a path through the  nodes that have been discovered a recursive algorithm as this functions similarly to a tree traversal
void A_search::A_Node_Search()
{
    
    std::vector<float> temp;
    if(PathOption == true)
    {
        
        //building a complete path from start to goal
        Candidates.clear();
        //recording all the nodes that are 0.75 away
        
        for(std::vector<float>Node:KnownPoints)
        {
            
            //as the first 2 are a parent the next three comprise of x,y and cost so this needs to be appropriately handled as cost isnt needed and would mess up the calculations
            for(int x = 2;x<Node.size()-2;x+=3)
            {
                
                //Adding all the potential candidates
                //handling the the parent
                if(x==2)
                {
                    //hbandling the parent as well as the child directly next to the parent
                    temp ={Node[0],Node[1]};
                    
                    if(sqrt(((temp[0]-Current[0])*(temp[0]-Current[0])) +((temp[1]-Current[1])*(temp[1]-Current[1])))<=1.5)
                    {
                        Candidates.push_back(temp);
                        
                    }
                    temp ={Node[x],Node[x+1]};
                    if(sqrt(((temp[0]-Current[0])*(temp[0]-Current[0])) +((temp[1]-Current[1])*(temp[1]-Current[1])))<=1.5)
                    {
                        Candidates.push_back(temp);
                        
                    }
                    
                }
                else
                {
                    //handling subsequent children after the parent and adjacent child
                    temp ={Node[x],Node[x+1]};
                    if(sqrt(((temp[0]-Current[0])*(temp[0]-Current[0])) +((temp[1]-Current[1])*(temp[1]-Current[1])))<=1.5)
                    {
                        Candidates.push_back(temp);
                       
                    }
                }
            }
            
        }
        
        //going through the candidates and checking which one is the best to add to the path
        
        
        for(std::vector<float>candi:Candidates)
        {
            
            //getting a cost as its needed
            GetCost(candi[0],candi[1]);
            //checking whether not the next move is the end or not and defaulting to it
            if(candi == End)
            {
                smallest = candi;
                break;
            }
            //if the algorithm is being initialised this is called
            if(smallCost==-1)
            {
                smallCost=cost;
                smallest = candi;
            }
            //else a standard procedure is called which simply replaced the smallest candidate if the cost is smaller than the smallest one
            else
            {
                if(cost<smallCost)
                {
                    smallCost = cost;
                    smallest = candi;
                }

            }
        }
        smallCost = -1;
        
        //adding this to the path and making it the next point of interest to find the next available move like the looking at the neighbours in A*
        Path.push_back(smallest);

        //testing whether or not this code is stuck/cant find a solution to break it
        if(Current == smallest)
        {
            if(count == 1)
            {
               std::cout<<"Creating solution failed resorting to exploration"<<std::endl;
               usePath = false;
               count = 0;
               return; 
            }
            count++;
        }

        Current = smallest;
        //if the path is eqivalent to the end then the function is allowed to return, otherwise it calls itself once again
        if(Current!=End)
        {
            A_Node_Search();
        }
        else
        {
            std::cout<<"A path has been created"<<std::endl;
            std::string path;
            for(std::vector<float>point:Path)
            {
                path+="["+std::to_string(point[0])+","+std::to_string(point[1])+"]";
            }
            std::cout<<path<<std::endl;
        }
    }
    else
    {
        
        //building a partial path using the closest known position
        
        //finding the closest point known to the goal point to travel towards
        Candidates.clear();
        //recording all the nodes to locate to closest one using avg if manhatten and euclidean dist
        //this only needs to happen once to establish a target to aim for 
        for(std::vector<float>Node:KnownPoints)
        {
            for(int x = 2;x<Node.size()-2;x+=3)
            {
                //Adding all the potential candidates
                if(x==2)
                {
                    temp ={Node[0],Node[1]};
                    Candidates.push_back(temp);
                    temp ={Node[x],Node[x+1]};
                    Candidates.push_back(temp);
                }
                else
                {
                    temp ={Node[x],Node[x+1]};
                    Candidates.push_back(temp);
                        
                }
            }
        }
        
        //iterating through the candidates to find the smallest one for the rough target to head towards
        for(std::vector<float>candi:Candidates)
        {
            GetCost(candi[0],candi[1]);
            if(smallCost==-1)
            {
                smallCost=cost;
                smallest = candi;
            }
            else
            {
                if(cost<smallCost)
                {
                    smallCost = cost;
                    smallest = candi;
                }

            } 
        }
        
        //utilising the recursive code above by changing the end
        //as there isnt  really an end
        smallCost = -1;
        End = smallest;
        PathOption = true;

        A_Node_Search();
    }
};
void A_search::GetCost(float x, float y)
{
    //getting the cost for the node calling this method is rather simple as its same hueristic as the previous approach

    //inputting the node you want to test, it will calculate the distance between them and pick the lowest one
    //calculating the manhattten distance
    float Max_Dist;
    if(x > End[0])
    {
        if(y> End[1])
        {
            Max_Dist = (x-End[0])+(y-End[1]);
        }
        else
        {
            Max_Dist = (x-End[0])+(End[1]-y);
        }
    }    
    else
    {
        if(y> End[1])
        {
            Max_Dist = (End[0]-x)+(y-End[1]);
        }
        else
        {
            Max_Dist = (End[0]-x)+(End[1]-y);
        }
    
    }
    //calculating the euclidean distance to the end point
    float eu_Dist = sqrt(((End[0]-x)*(End[0]-x))+((End[1]-y)*(End[1]-y)));
    //simply using the same cost metric as before
    cost = (Max_Dist+eu_Dist)/2;

};