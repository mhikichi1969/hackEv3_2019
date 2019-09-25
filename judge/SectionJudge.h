#ifndef _SECTIONJUDGE_H_
#define _SECTIONJUDGE_H_

#include "Judge.h"
#include "HPolling.h"
#include "Odometry.h"

class SectionJudge : public Judge {
    public:
        SectionJudge(HPolling *polling,
                Odometry *odometry);
        //bool isOK();

    private:

};
#endif
