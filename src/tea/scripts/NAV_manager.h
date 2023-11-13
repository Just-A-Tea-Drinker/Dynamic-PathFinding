#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <filesystem>
#include <limits.h>
#include <stdio.h>




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
    void Node_Ret();
    void Node_Add(float x,float y,std::vector<std::vector<float>>Parent,std::vector<std::vector<float>>Children);
};