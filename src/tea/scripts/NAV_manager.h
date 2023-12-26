#include "Headers.h"


class NAV_manager
{
    public:
    //variables
    //dealing with the file contents
    std::vector<std::vector<float>> Target_contents;
    std::vector<std::vector<float>> Node_contents;
    //just a target from the files
    std::vector<float> Target;
    //storing nodes when routing to be stored and optimised
    std::vector<std::vector<float>>Nodes_traversed;



    //constructor/destructor setup
    NAV_manager();
    ~NAV_manager();

    //terminal based UI methods
    void Target_Handler();
    void Node_Handler();

    //reads and writes to the long memory
    void Node_ReadFormat();
    void Node_WriteFormat();

    //Methods for backend features such as node optimisation, adding nodes to be optimised and returning the next waypoint
    void Node_Ret();
    void Node_Add(std::vector<std::vector<float>>Node);
    void Node_Opti();
};