#include "BlockCircle.h"

BlockCircle::BlockCircle(int node,int id,COLOR col)
        :BlockPlace(node),
        circleid(id),
        color(col)
{
    kind=BPKIND::BLOCK;

}

COLOR BlockCircle::getColor()
{
    return color;
}

int BlockCircle::getid()
{
    return circleid;
}