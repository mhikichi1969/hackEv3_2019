#include "Block.h"


Block::Block(COLOR col)
{
    color = col;

}

COLOR Block::getColor()
{
    return color;
}

void Block::setColor(COLOR col)
{
    color = col;
}