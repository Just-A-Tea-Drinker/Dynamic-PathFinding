#include "NAV_manager.h"



//customised constructor
NAV_manager::NAV_manager()
{
    //insert the file reading of a  target on a given line
    //due to the sourcing in the ROS targets is found outside of source in the project WS

    std::ifstream TargetFile("targets.txt");
    std::ifstream NodeFile("Nodes.txt");

    std::string temp;
    
    std::vector<float> line;

    //will iterate ln x ln and remove the delimiter ','
    while(getline(TargetFile,temp))
    {
        line.clear();
        std::stringstream ss(temp);
        for(float i;ss >> i;)
        {
            line.push_back(i);
            if(ss.peek()== ',')
            {
                ss.ignore();
            }

        }
        Target_contents.push_back(line);
    }
    TargetFile.close();

    //will iterate ln x ln and remove the delimiter ','
    while(getline(NodeFile,temp))
    {
        line.clear();
        std::stringstream ss(temp);
        for(float i;ss >> i;)
        {
            line.push_back(i);
            if(ss.peek()== ',')
            {
                ss.ignore();
            }

        }
        Target_contents.push_back(line);
    }
    NodeFile.close();
    // The code was adapted from previous work
    //link: https://github.com/Beniial/TSE-TIAGo-Software-2023/blob/release/move_tiago/src/config_manage.cpp
    if(Node_contents.size()>1)
    {
        Target_Handler();
    }

};
NAV_manager::~NAV_manager()
{

};

//method for obtaining a user chosen location
void NAV_manager::Target_Handler()
{
    int index;
    //pulling a certain vector from the vector list
    std::cout<<"The targets content has:"<<Target_contents.size()<<" targets\nPlease select one between 0 & "<< Target_contents.size()<<std::endl;
    std::cin>>index;
    while(true)
    {
        try
        {
            if(index<0 || index>Target_contents.size())
            {
                throw(index);
            }
            else
            {
                Target= Target_contents[index];
                break;
            }
        }
        catch(int index)
        {
            std::cout<<"Index must be between 0 & "<<Target_contents.size()<<std::endl;
        }
    }
    
};


//method for pulling nodes from the vector of vectors to check what its learnt
void NAV_manager::Node_Ret()
{
    //Method for accessing and retrieving inormation from vec x vec from constr

};



//Method for adding new nodes into the 'Long term memory'
void NAV_manager::Node_Add(float x,float y,std::vector<std::vector<float>>Parent,std::vector<std::vector<float>>Children)
{

};