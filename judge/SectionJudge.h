#ifndef _SECTIONJUDGE_H_
#define _SECTIONJUDGE_H_

#include "Judge.h"
#include "HPolling.h"
#include "Odometry.h"
//#include "SpeedSectionEnum.h"

class SectionJudge : public Judge {
    public:
        SectionJudge(HPolling *polling,
                Odometry *odometry);

         
        void setAngleParam(End endFlag); 

        void setValue(int cmd,double val);
        bool angleCheck();
        void resetLength();
        


    private:

};
#endif
