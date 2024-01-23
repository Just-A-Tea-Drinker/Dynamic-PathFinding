#include "NAV_manager.h"



//customised constructor
NAV_manager::NAV_manager()
{
    //insert the file reading of a  target on a given line
    //due to the sourcing in the ROS targets is found outside of source in the project WS
    ROS_INFO("File management active");
    std::ifstream TargetFile("targets.txt");
    std::ifstream NodeFile("Nodes.txt");
    closing =false;
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
    ROS_INFO("File management deactivated");
};
void NAV_manager::Node_Handler()
{
    char ans;
    while(true)
    {
        try
        {
            std::cout<<"This program has detected recorded nodes in storage, would you like to use them? Y/N"<<std::endl;
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
void NAV_manager::Found_Target()
{
    char ans;
    while(true)
    {

      try
        {
            std::cout<<"The designated target has been reached \nWould you like to navigate to a new one? Y/N"<<std::endl;
            std::cout<<"Entering 'N' will close the program"<<std::endl;
            std::cin>>ans;
            ans = std::tolower(ans);
            if(ans =='y')
            {
                Target_Handler();
                break;
            }
            else if (ans =='n')
            {
                ROS_INFO("Closing the program");
                ros::shutdown();
                closing = true;
                return;
                
                
                
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
    
}


//methods handling the importing and exporting of the nodes
void NAV_manager::Node_ReadFormat()
{
    //method for pulling the data stored on a to be processed
    //sorting through node contents as it will be all together
    int count = 0;
    std::vector<float> New_node;
    std::vector<std::vector<float>>storage;
    //resetting the vectors

    for(int x = 0;x<Node_contents.size();x++)
    {
        //adding parent
        New_Nodes.push_back({Node_contents[x][0],Node_contents[x][1]});
        //adding the children
        for(int i =2;i<Node_contents[x].size();i++)
        {
            if(count==3)
            {
                New_Nodes.push_back(New_node);
                New_node.clear();
                count = 0;
            }
            else
            {
                New_node.push_back(Node_contents[x][i]);
                count++;
            }
        }
        All_Nodes.push_back(New_Nodes);
        //New_Nodes.clear();
       

    }
};

//Method for adding new nodes into the 'Long term memory'
void NAV_manager::Node_WriteFormat()
{
    //adding the node the txt with parent, and children with their costs
    std::ofstream of;
    std::fstream f;
    //opening the nodes file in append mode
    of.open("Nodes.txt",std::ios::app);
    for(std::vector<std::vector<float>>Node:Opti_Nodes)
    {
        std::string Node_To_Write = std::to_string(Node[0][0])+","+std::to_string(Node[0][1])+" ";
        for(int child = 1;child < Node.size();child++)
        {
            for(float val :Node[child])
            {
                Node_To_Write+= std::to_string(val)+", ";
            }
        
        }
        Node_To_Write+="\n";
        of<< Node_To_Write;
    }
    
    of.close();
   
};


void NAV_manager::Node_Add(std::vector<std::vector<float>>Node)
{   
    //storing the nodes in the all nodes as this allows me to regress if need be for example if an obstacle is there the robot shouldnt be blinded by rewards as it knows those nodes are forbidden
    All_Nodes.push_back(Node);


    //all of the nodes will be optimised as a complete batch in post processing allowing the robot to simply gather information untill it reaches the target
    Opti_Nodes.push_back(Node);

};

void NAV_manager::Node_Opti()
{
    std::vector<float> Fpar,Spar;
    //method used for trimming and tailoring the parent-child relations based on distance
    for(int node = 0;node<Opti_Nodes.size()-2;node++)
    {
        //FIRST PARENT
        Fpar =Opti_Nodes[node][0];
        //SECOND PARENT
        Spar= Opti_Nodes[node+2][0];

        //getting the distance
        float parent_distance = sqrt(((Spar[0]-Fpar[0])*(Spar[0]-Fpar[0]))+((Spar[1]-Fpar[1])*(Spar[1]-Fpar[1])));
        if(parent_distance<1.0f)
        {
            Opti_Nodes[node+2][0]=Opti_Nodes[node][0];

        }
    }
   
    

};