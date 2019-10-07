#ifndef _SECTIONJUDGE_H_
#define _SECTIONJUDGE_H_

#include "Judge.h"
#include "HPolling.h"
#include "Odometry.h"
#include "SpeedSectionEnum.h"

class SectionJudge : public Judge {
    public:
        SectionJudge(HPolling *polling,
                Odometry *odometry);
        void setValue(int cmd,double val);
        void setAngleParam(Flag::End endFlag);
        bool angleCheck();
        void resetLength();


    private:

};
#endif
