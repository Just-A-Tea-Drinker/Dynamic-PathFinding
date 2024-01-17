#include "Headers.h"


class A_search
{
public:
//variables
std::vector<float>Start,End,Nearest_to_target;
std::vector<std::vector<float>>Path,KnownPoints;
bool usePath,GetTarget,PathExplore;


//con/destructor
A_search();
A_search(std::vector<float>start,std::vector<float>Target,std::vector<std::vector<float>> Nodes);
~A_search();
//methods







private:
//variables
bool PathOption;
std::vector<std::vector<float>>Candidates;
std::vector<float>Current;
int count;
float cost;
//methods
void Node_trav();
void A_Node_Search();
void GetCost(float x, float y);










};
