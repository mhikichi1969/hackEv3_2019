#ifndef _Section_H_
#define _Section_H_

#include "Judge.h"
#include "SimpleWalker.h"
#include "Turn.h"
#include "StraightWalker.h"
//#include "LineTracer.h"

class Section {
    public:
        Section(Judge *judge,
                SimpleWalker *waller,
                //LineTracer *waller,
                StraightWalker *straight,
                Turn *turn);

        virtual bool run() = 0;
        void setNext(Section *next);
        Section *next();
        void recordCount();

    protected:

        Judge *mJudge;
        SimpleWalker *mSimpleWalker;
        Turn *mTurn;
        StraightWalker *mStraightWalker;


        Section *mNext;

};
#endif