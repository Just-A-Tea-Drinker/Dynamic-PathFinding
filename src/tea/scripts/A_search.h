#include "Headers.h"


class A_search
{
public:
//variables
std::vector<float>Start,End,Nearest_to_target;
std::vector<std::vector<float>>Path,KnownPoints;


//con/destructor
A_search();
A_search(float x,float y,std::vector<float>Target);
~A_search();
//methods

void A_Node_Search();
void GetCost();





private:
//variables



//methods











};
