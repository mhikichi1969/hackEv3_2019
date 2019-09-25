#ifndef __BLOCK_H__
#define __BLOCK_H__

#include "BingoEnum.h"

class Block
{
    public:
        Block(COLOR col);
        COLOR getColor();
        void setColor(COLOR col);

    private:
        COLOR color;


};

#endif