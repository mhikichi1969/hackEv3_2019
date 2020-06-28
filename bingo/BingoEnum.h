#ifndef __BINGO_ENUM_H__
#define __BINGO_ENUM_H__
    enum class COLOR : int{
        RED,
        GREEN,
        BLUE,
        YELLOW,
        BLACK,
        NONE
    };

    enum DIR{
        N,
        E,
        S,
        W
    };

    enum BPKIND {
        CROSS,
        BLOCK,
        LINE,
        OTHER
    };

#endif