#include "Section.h"

Section::Section(Judge *judge,
        SimpleWalker *walker,
        //LineTracer *walker,
        StraightWalker *straight,
        Turn *turn,
        VirtualTracer *vt
        ):
        mJudge(judge),
        mSimpleWalker(walker),
        mStraightWalker(straight),
        mTurn(turn),
        mVirtualTracer(vt),
        mNext(NULL)
{

}

/*
bool Section::run()
{
    ev3_speaker_play_tone(NOTE_C4,30);

    return false;
}*/

void Section::setNext(Section *next)
{
    mNext = next;
}

Section *Section::next()
{
    return mNext;
}

void Section::recordCount()
{
    //ev3_speaker_play_tone(NOTE_E4,100);
    mJudge->recordCount();
}
