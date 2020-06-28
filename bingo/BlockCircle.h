#ifndef __BLOCK_CIRCLE_H__
#define __BLOCK_CIRCLE_H__

#include "BlockPlace.h"

class BlockCircle : public BlockPlace
{
    public:
        BlockCircle(int node,int id, COLOR color);
        COLOR getColor();
        int getid();

    private:
        int circleid;
        COLOR color;

       
    
};


#endif