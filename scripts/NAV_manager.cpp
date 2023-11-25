#include "NAV_manager.h"
#include "Headers.h"


//customised constructor
NAV_manager::NAV_manager()
{
    //insert the file reading of a  target on a given line
    //due to the sourcing in the ROS targets is found outside of source in the project WS
    ROS_INFO("File management active");
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
        //std::cout<<line[0]<<std::endl;
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
        Node_contents.push_back(line);
    }
    NodeFile.close();
    // The code was adapted from previous work
    //link: https://github.com/Beniial/TSE-TIAGo-Software-2023/blob/release/move_tiago/src/config_manage.cpp

    if(Target_contents.size()>1)
    {
        Target_Handler();
    }
    else
    {
        Target = Target_contents[0];
    }
    if(Node_contents.size()>0)
    {
        Node_Handler();
    }
    else
    {
        ROS_INFO("No Nodes detected");
    }
   

};
NAV_manager::~NAV_manager()
{

};
void NAV_manager::Node_Handler()
{
    char ans;
    while(true)
    {
        try
        {
            std::cout<<"This program has detected recorded nodes in storage would you like to use them? Y/N"<<std::endl;
            std::cin>>ans;
            ans = std::tolower(ans);
            if(ans =='y')
            {
                ROS_INFO("Nodes loaded");
                break;
            }
            else if (ans =='n')
            {
                std::ofstream file("Nodes.txt");
                ROS_INFO("Nodes file contents deleted");
                file.close();
                break;
                
            }
            else
            {
                throw(ans);
            }

        }
        catch(std::string ans)
        {
            std::cout<<"You have not entered a valid response, please try again."<<std::endl;
        }
    }
};
//method for obtaining a user chosen location
void NAV_manager::Target_Handler()
{
    int index;
    //pulling a certain vector from the vector list
    
    while(true)
    {
        try
        {
            std::cout<<"The targets content has:"<<Target_contents.size()<<" targets\nPlease select one between 0 & "<< Target_contents.size()-1<<std::endl;
            std::cin>>index;
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
            std::cout<<"Index must be between 0 & "<<Target_contents.size()-1 <<std::endl;
            
        }
    }
    
};


//method for pulling nodes from the vector of vectors to check what its learnt
void NAV_manager::Node_Ret()
{
    //Method for accessing and retrieving inormation from vec x vec from constr

};


void NAV_manager::Node_ReadFormat()
{

};
//Method for adding new nodes into the 'Long term memory'
void NAV_manager::Node_Add(std::vector<std::vector<float>>Node)
{
    //adding the node the txt with parent, and children with their costs
    std::ofstream of;
    std::fstream f;
    //opening the nodes file in append mode
    of.open("Nodes.txt",std::ios::app);
    std::string Node_To_Write = "Parent:"+std::to_string(Node[0][0])+","+std::to_string(Node[0][1]);
    for(int child = 1;child < Node.size();child++)
    {
        Node_To_Write+=" Child:";
        for(float val :Node[child])
        {
            Node_To_Write+= std::to_string(val)+", ";
        }
        
    }
    Node_To_Write+="\n";
    of<< Node_To_Write;
    of.close();
};