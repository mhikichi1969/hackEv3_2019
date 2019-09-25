#ifndef __CROSS_CIRCLE_H__
#define __CROSS_CIRCLE_H__

#include "BlockPlace.h"
#include "BingoEnum.h"

class CrossCircle : public BlockPlace
{
    public:
        CrossCircle(int node,int id, COLOR color);
        int getId();
        COLOR getColor();

    private:
        int circleid;
        COLOR color;
    
};


#endif