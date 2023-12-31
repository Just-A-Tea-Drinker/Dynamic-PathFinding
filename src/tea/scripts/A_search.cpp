#include "A_search.h"


//con/destructor
A_search::A_search(){};
A_search::A_search(float x,float y,std::vector<float>Target)
{
    ROS_INFO("Node traversal activated");
    //defining the start and end of the desired path
    Start = {x,y};
    End = Target; 
};
A_search::~A_search()
{
    ROS_INFO("Node traversal terminated");
};





//Method used for findinG a path through the  nodes that have been discovered
void A_search::A_Node_Search()
{





};
void A_search::GetCost()
{
    //getting the cost for the node calling this method is rather simple as its same hueristic as the previous approach

    //inputting the node you want to test, it will calculate the distance between them and pick the lowest one



};