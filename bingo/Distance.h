#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#include "Area.h"
#include "BingoEnum.h"

class Distance
{
    public:
        Distance(Area *area);
        void initcost();
        void initturncost();
        bool checkRoute(int pt[]);
        void calcDistance();
        void setStart(int node);
        int calcTurncost(int goal[], DIR dir);
        int checkmincostdir(int row,int col);

        void calcTurncostAll(DIR start_dir);
        double getLenAndTurnCost(int node);
        DIR getGoalDirection(int goal);
        void getRoute(int st,int ed,int ret[]);
        int setBlockinGoalCost(int nodeid);
        void calcTurncostAllPostProcess(int start_node,DIR start_dir);
        void setCheckBlock(bool check);
        void debugPrint();
        void debugPrint2();


    private:
        Area *mArea;
		double initialcost[7][7] = 
                            {{100,100,100,100,100,100,100}, 
							 {100,100,100,100,100,100,100} , 
							 {100,100,100,100,100,100,100} , 
							 {100,100,100,100,100,100,100} , 
							 {100,100,100,100,100,100,100} , 
							 {100,100,100,100,100,100,100} , 
							 {100,100,100,100,100,100,100} };
        double resultcost[7][7];
        double turncost[7][7];

        int st[2];
        bool blockcheck = true;

};

#endif

