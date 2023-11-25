#include "Headers.h"


class NAV_manager
{
    public:
    //variables
    std::vector<std::vector<float>> Target_contents;
    std::vector<std::vector<float>> Node_contents;
    std::vector<float> Target;



    //constructor/destructor setup
    NAV_manager();
    ~NAV_manager();

    void Target_Handler();
    void Node_Handler();
    void Node_Ret();
    void Node_ReadFormat();
    void Node_Add(std::vector<std::vector<float>>Node);
};