#include "Distance.h"
#include "Runner.h"
#include "math.h"
#include "CrossCircle.h"

#include "util.h"

Distance::Distance(Area *area):
    mArea(area)
{

	 blockcheck = true;
	
}

void Distance::initcost()
{
    for(int i=0;i<7;i++) {
        for(int j=0;j<7;j++) {
            resultcost[i][j]=initialcost[i][j];
        }
    }
}

void Distance::initturncost()
{
    for(int i=0;i<7;i++) {
        for(int j=0;j<7;j++) {
            turncost[i][j]=initialcost[i][j];
        }
    }
}


void Distance::setStart(int node)
{
    st[0] = node/7; //row
    st[1] = node%7; //col


}

/* 進入不可チェック
 pt は座標(row,col)
*/
bool Distance::checkRoute(int pt[])
{
    int idx = pt[0]*7+pt[1];
    if(mArea->getBlockPlace(idx)==nullptr) //置き場が無い（真ん中）
        return false;
    if(mArea->getBlockPlace(idx)->getKind()==BPKIND::BLOCK) // ブロックサークル
        return false;
    if (blockcheck && mArea->getBlockPlace(idx)->getBlock()!=nullptr) // ブロックが置いている
        return false;
    return true;
}

void Distance::calcDistance() {
    initcost();
    resultcost[st[0]][st[1]]=0;
    
    for(int cnt=0;cnt<10;cnt++) {
    int change=true;
    while (change) {
        change = false;
        for (int row=0;row<7;row++){
            for (int col=0;col<7;col++) {
                int n[] = {row-1, col};
                int e[] = {row,col+1 };
                int s[] = {row+1,col };
                int w[] = {row,col-1 };
                
                if (n[0]>=0){
                    if (checkRoute(n) ) {
                        if (resultcost[n[0]][n[1]] > resultcost[row][col]+1) {
                                resultcost[n[0]][n[1]] = resultcost[row][col]+1;
                                change=true;
                        }
                    }
                }
                if (e[1]<=6){
                    if (checkRoute(e)) {
                        if (resultcost[e[0]][e[1]] > resultcost[row][col]+1){
                                resultcost[e[0]][e[1]] = resultcost[row][col]+1;
                                change=true;
                        }
                    }
                }
                if (s[0]<=6){
                    if (checkRoute(s)){
                        if (resultcost[s[0]][s[1]] > resultcost[row][col]+1){
                                resultcost[s[0]][s[1]] = resultcost[row][col]+1;
                                change=true;
                        }
                    }
                }
                if (w[1]>=0){
                    if (checkRoute(w)){
                        if (resultcost[w[0]][w[1]] > resultcost[row][col]+1) {
                                resultcost[w[0]][w[1]] = resultcost[row][col]+1;
                                change=true;
                        }
                    }
                }
            }
        }
    }
    }                   
}

/*
int Distance::calcTurncost(int goal[], DIR dir){
    int start_dir = ((int)dir+2)%4;
    int cost;

    int row = goal[0];
    int col = goal[1];	
    if (resultcost[row][col]==100)
        return 100;
    if (resultcost[row][col]==0)
        return 0;

    int no = resultcost[row][col];

    Runner *runner=mArea->getRunner()->makeClone();

    cost = 0;
    int cnt=0;
    int newcost;
    while (no!=0) { 
        int n[] = {row-1, col};
        int e[] = {row,col+1};
        int s[] = {row+1,col};
        int w[] = {row,col-1};
        if (0<=n[0] && n[0]<=6 && 0<=n[1] && n[1]<=6 && resultcost[n[0]][n[1]]==resultcost[row][col]-1) {
            no = resultcost[n[0]][n[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,n);
            if (cnt!=0) cost = cost +newcost;
            row = n[0];
            col = n[1];
        } else if(0<=e[0] && e[0]<=6 && 0<=e[1] && e[1]<=6  && resultcost[e[0]][e[1]]==resultcost[row][col]-1) {
            no = resultcost[e[0]][e[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,e);
            if (cnt!=0) cost = cost +newcost;
            row = e[0];
            col = e[1];
        } else if( 0<=s[0] && s[0]<=6 && 0<=s[1] && s[1]<=6 && resultcost[s[0]][s[1]]==resultcost[row][col]-1) {
            no = resultcost[s[0]][s[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,s);
            if (cnt!=0) cost = cost +newcost;
            row = s[0];
            col = s[1];
        } else if(0<=w[0] && w[0]<=6 && 0<=w[1] && w[1]<=6  && resultcost[w[0]][w[1]]==resultcost[row][col]-1) {
            no = resultcost[w[0]][w[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,w);
            if (cnt!=0) cost = cost +newcost;
            row = w[0];
            col = w[1];
        } else {
            delete runner;
            return 100;
        }
        cnt++;
    }
    cost = cost+fabs((int)runner->getDir()-start_dir);
    delete runner;

    return cost;		
}		
*/

int Distance::calcTurncost(int goal[], DIR st_dir){
    int start_dir = ((int)st_dir+2)%4;
    int cost;

    int row = goal[0];
    int col = goal[1];	
    if (resultcost[row][col]==100)
        return 100;
    if (resultcost[row][col]==0)
        return 0;

    int no = resultcost[row][col];

    Runner *runner=mArea->getRunner()->makeClone();

    cost = 0;
    int cnt=0;
    int newcost;
    while (no!=0) { 
        int n[] = {row-1, col};
        int e[] = {row,col+1};
        int s[] = {row+1,col};
        int w[] = {row,col-1};
        int dir = checkmincostdir(row,col);

        if (dir==0) {
            no = resultcost[n[0]][n[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,n);
            if (cnt!=0) cost = cost +newcost;
            row = n[0];
            col = n[1];
        } else if(dir==1) {
            no = resultcost[e[0]][e[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,e);
            if (cnt!=0) cost = cost +newcost;
            row = e[0];
            col = e[1];
        } else if(dir==2) {
            no = resultcost[s[0]][s[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,s);
            if (cnt!=0) cost = cost +newcost;
            row = s[0];
            col = s[1];
        } else if(dir==3) {
            no = resultcost[w[0]][w[1]];
            int pt[] = {row,col};
            newcost = runner->checkDirection(pt,w);
            if (cnt!=0) cost = cost +newcost;
            row = w[0];
            col = w[1];
        } else {
            delete runner;
            return 100;
        }
        cnt++;
    }

    int st_turn = fabs((int)runner->getDir()-start_dir);
    if (st_turn>2) {
        st_turn = fabs((int)runner->getDir()-start_dir)-2;
    }

    cost = cost+st_turn;

    delete runner;

    return cost;		
}		

int Distance::checkmincostdir(int row,int col) 
{
    int n[] = {row-1, col};
    int e[] = {row,col+1};
    int s[] = {row+1,col};
    int w[] = {row,col-1};
    int diff = 100;
    int maxdiff = 100;
    int maxcost=100;
    int select = -1;
    int cost;
    if (0<=n[0] && n[0]<=6 && 0<=n[1] && n[1]<=6 && resultcost[n[0]][n[1]]<resultcost[row][col]) {
        maxcost = resultcost[n[0]][n[1]]+turncost[n[0]][n[1]];
        maxdiff = resultcost[row][col]-resultcost[n[0]][n[1]];
        select = 0;
    }
    if (0<=e[0] && e[0]<=6 && 0<=e[1] && e[1]<=6 && resultcost[e[0]][e[1]]<resultcost[row][col]) {
        cost = resultcost[e[0]][e[1]]+turncost[e[0]][e[1]];
        diff = resultcost[row][col]-resultcost[e[0]][e[1]];
        if(maxcost>cost){
            maxcost = cost;
            maxdiff = diff;
            select=1;
        }

    }

    if (0<=s[0] && s[0]<=6 && 0<=s[1] && s[1]<=6 && resultcost[s[0]][s[1]]<resultcost[row][col]) {
        cost = resultcost[s[0]][s[1]]+turncost[s[0]][s[1]];
        diff = resultcost[row][col]-resultcost[s[0]][s[1]];
        if(maxcost>cost){
            maxcost = cost;
            maxdiff = diff;
            select=2;
        }
    }
    if (0<=w[0] && w[0]<=6 && 0<=w[1] && w[1]<=6 && resultcost[w[0]][w[1]]<resultcost[row][col]) {
        cost = resultcost[w[0]][w[1]]+turncost[w[0]][w[1]];
        diff = resultcost[row][col]-resultcost[w[0]][w[1]];
        if(maxcost>cost){
            maxcost = cost;
            maxdiff = diff;
            select=3;
        }
    }
    return select;			
}

void Distance::calcTurncostAll(DIR start_dir) {
        initturncost();
        for(int cnt=0;cnt<30;cnt++) {
            for (int row =0; row<7;row++) {
                for (int col =0; col<7;col++) {
                    int pt[] = {row,col};
                    turncost[row][col] = calcTurncost(pt,start_dir);
                }
            }
        }
		//debugPrint2();
}

double Distance::getLenAndTurnCost(int node)
{
    double len_cost=0.6;
    double turn_cost=1.7; 
    int pt[]={node/7,node%7};
    return resultcost[pt[0]][pt[1]]*len_cost+turncost[pt[0]][pt[1]]*turn_cost;
}

/* コスト表から走行体の進入方法を推定*/
DIR Distance::getGoalDirection(int goal){
    int row = goal/7;
    int col = goal%7;	

    int n[] = {row-1, col};
    int e[] = {row,col+1};
    int s[] = {row+1,col};
    int w[] = {row,col-1};
    int min=100;
    DIR result=(DIR)-1;

 //   if(goal==12) debugPrint2();

    if (0<=n[0] && n[0]<=6 && 0<=n[1] && n[1]<=6 && resultcost[n[0]][n[1]]+turncost[n[0]][n[1]]<min) {
        min = resultcost[n[0]][n[1]]+turncost[n[0]][n[1]];
        result = DIR::S;
    }
    if ( 0<=e[0] && e[0]<=6 && 0<=e[1] && e[1]<=6 && resultcost[e[0]][e[1]]+turncost[e[0]][e[1]]<min) {
        min = resultcost[e[0]][e[1]]+turncost[e[0]][e[1]];
        result = DIR::W;
    }
    if  (0<=s[0] && s[0]<=6 && 0<=s[1] && s[1]<=6 && resultcost[s[0]][s[1]]+turncost[s[0]][s[1]]<min) {
        min = resultcost[s[0]][s[1]]+turncost[s[0]][s[1]];
        result = DIR::N;
    }
    if  (0<=w[0] && w[0]<=6 && 0<=w[1] && w[1]<=6 && resultcost[w[0]][w[1]]+turncost[w[0]][w[1]]<min) {
        min = resultcost[w[0]][w[1]]+turncost[w[0]][w[1]];
        result = DIR::E;
    }

    return result;
}

void Distance::getRoute(int st_node,int ed_node,int ret[])
{
    int st[] = {st_node/7, st_node%7};
    int ed[] = {ed_node/7, ed_node%7};
    int sp=0;
    int stack[50];

    int row = ed[0];
    int col = ed[1];
    stack[sp++]=row*7+col;

    char buf[256];
    Runner *runner=mArea->getRunner()->makeClone();  // 9/21
    bool first=true;

    while(resultcost[row][col]!=0 ) {

        int n[] = {row-1, col};
        int e[] = {row,col+1};
        int s[] = {row+1,col};
        int w[] = {row,col-1};

        DIR dir = getGoalDirection(row*7+col);
        if (first) {
            runner->setDir(dir);
            first=false;
        }

        if(dir==DIR::N) {
            row = s[0];
            col = s[1];
        }        
        else if(dir==DIR::E) {
            row = w[0];
            col = w[1];
        }        
        else if(dir==DIR::S) {
            row = n[0];
            col = n[1];
        }        
        else if(dir==DIR::W) {
            row = e[0];
            col = e[1];
        } else {
            return;
        }       
        stack[sp++]=row*7+col;

        /*
        if (0<=n[0] && n[0]<=6 && 0<=n[1] && n[1]<=6 && resultcost[row][col]-1==resultcost[n[0]][n[1]]) {
            stack[sp++]=n[0]*7+n[1];
            row = n[0];
            if (first) {
                runner->setDir(DIR::S);
                first=false;
            }
        }
        else if ( 0<=e[0] && e[0]<=6 && 0<=e[1] && e[1]<=6 && resultcost[row][col]-1==resultcost[e[0]][e[1]]) {
            stack[sp++]=e[0]*7+e[1];
            row = e[0];
            col = e[1];
             if (first) {
                runner->setDir(DIR::W);
                first=false;
            }
      }
        else if  (0<=s[0] && s[0]<=6 && 0<=s[1] && s[1]<=6 && resultcost[row][col]-1==resultcost[s[0]][s[1]]) {
            stack[sp++]=s[0]*7+s[1];
            row = s[0];
            col = s[1];
            if (first) {
                runner->setDir(DIR::N);
                first=false;
            }

        }
        else if  (0<=w[0] && w[0]<=6 && 0<=w[1] && w[1]<=6 && resultcost[row][col]-1==resultcost[w[0]][w[1]]) {
            stack[sp++]=w[0]*7+w[1];
            row = w[0];
            col = w[1];
            if (first) {
                runner->setDir(DIR::E);
                first=false;
            }

        } else {
            return;
        }*/
       // sprintf(buf,"%d,%d,%d",row,col,resultcost[row][col]);
       // msg_f(buf,0);
    }
   // msg_f("route2",0);
    int cnt=0;
    while(sp>0) {
        ret[cnt++]=stack[--sp];
    }
    ret[cnt]=999;

    delete runner;

}

/*
    ブロックが置いてあるところがゴールの場合に最終的なコストを計算する
*/
int Distance::setBlockinGoalCost(int nodeid)
{
    char buf[256];

    //ブロックサークルのブロックを取る場合は周辺の交点サークルがすべて空いていることを条件とする
         //msg_f("dist 0",11);
   BlockPlace *bp = mArea->getBlockPlace(nodeid);
        //sprintf(buf,"dist %d,%d",bp,nodeid);
        //msg_f(buf,11);

    if(bp->getKind()==BPKIND::BLOCK) {

        CrossCircle **cc = (CrossCircle **)bp->getSlashPlaces();
        for(int i=0;i<4;i++) {
            if(cc[i]->getBlock()!=nullptr) {
                COLOR col = cc[i]->getBlock()->getColor();
               // sprintf(buf,"dist %d,%d",cc[i]->getNodeid(),col);
               // msg_f(buf,8+i);
                return 100;
            }
        }
    }

    int row = nodeid/7;
    int col = nodeid%7;
    int n[] = {row-1,col};
    int e[] = {row, col+1};
    int s[] = {row+1, col};
    int w[] = {row, col-1};
    int min = 100;
    int turn =100;
    //msg_f("dist 1",11);
    if (n[0]>=0 && min>resultcost[n[0]][n[1]]) {
        min = resultcost[n[0]][n[1]];
        turn = turncost[n[0]][n[1]];
    }
    //    msg_f("dist 2",11);

    if (e[1]<=6 && min>resultcost[e[0]][e[1]]) {
        min =resultcost[e[0]][e[1]];
        turn = turncost[e[0]][e[1]];
    }
     //   msg_f("dist 3",11);

    if (s[0]<=6 && min>resultcost[s[0]][s[1]]) {
        min = resultcost[s[0]][s[1]];  
        turn = turncost[s[0]][s[1]];
    }
     //   msg_f("dist 4",11);

    if (w[1]>=0 && min>resultcost[w[0]][w[1]]) {
        min = resultcost[w[0]][w[1]];  
        turn = turncost[w[0]][w[1]];
    }
   // msg_f("dist 5",11);

    if (min<100) { 
        resultcost[row][col]=min+1;
        turncost[row][col] = turn;
    }

    return min+turn;
}

/*
    ブロックサークルを始点とした場合の旋回コストの更新
    ブロックサークル内での旋回不可
*/
void Distance::calcTurncostAllPostProcess(int start_node,DIR start_dir)
{
    int row = start_node/7;
    int col = start_node%7;

    if (start_dir==DIR::N){
        turncost[row][col+1] =100;
        turncost[row+1][col] =100;
        turncost[row][col-1] =100;
    }
    if (start_dir==DIR::E){
        turncost[row-1][col] =100;
        turncost[row+1][col] =100;
        turncost[row][col-1] =100;
    }
    if (start_dir==DIR::S){
        turncost[row-1][col] =100;
        turncost[row][col+1] =100;
        turncost[row][col-1] =100;
    }
    if (start_dir==DIR::W){
        turncost[row-1][col] =100;
        turncost[row][col+1] =100;
        turncost[row+1][col] =100;
    }
}

void Distance::setCheckBlock(bool check)
{
    blockcheck = check;
}
void Distance::debugPrint()
{
    char buf[256];
    char tmp[8];
    int line=2;
    for(int j=0;j<7;j++,line++) {
        for(int i=0;i<7;i++) {
            if(resultcost[j][i]>=10) 
                tmp[i] = '*';
            else 
                tmp[i] = '0'+resultcost[j][i];
        }
        sprintf(buf,"%c %c %c %c %c %c %c",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6]  );
        msg_f(buf,line);
    }

}

void Distance::debugPrint2()
{
    char buf[256];
    char tmp[8];
    int line=2;
    for(int j=0;j<7;j++,line++) {
        for(int i=0;i<7;i++) {
            if(turncost[j][i]>=10) 
                tmp[i] = '*';
            else 
                tmp[i] = '0'+turncost[j][i];
        }
        sprintf(buf,"%c %c %c %c %c %c %c",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6]  );
        msg_f(buf,line);
    }

}

