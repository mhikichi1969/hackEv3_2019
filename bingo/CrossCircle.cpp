#include "CrossCircle.h"
#include "BingoEnum.h"

CrossCircle::CrossCircle(int node,int id,COLOR col)
        :BlockPlace(node),
        circleid(id),
        color(col)
{
    kind=BPKIND::CROSS;

}

int CrossCircle::getId(){
    return circleid;    
}

COLOR CrossCircle::getColor(){
    return color;    
}