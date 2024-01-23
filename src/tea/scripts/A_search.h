#include "Headers.h"


class A_search
{
    public:
    //variables
    std::vector<float>Start,End,Nearest_to_target;
    std::vector<std::vector<float>>Path,KnownPoints;
    bool usePath,GetTarget,PathExplore,PathOption;


    //con/destructor
    A_search();
    A_search(std::vector<float>start,std::vector<float>Target,std::vector<std::vector<float>> Nodes);
    ~A_search();
    //methods
    void A_Node_Search();


    private:
    //variables
    std::vector<std::vector<float>>Candidates;
    std::vector<float>Current,smallest;
    int count;
    float cost;
    float smallCost = -1;
    //methods
    void Node_trav();
    void GetCost(float x, float y);

};
