#include "SectionCreate.h"
#include "BlockPlace.h"
#include "Area.h"
#include "util.h"
#include "ParamStruct.h"
#include "Flag.h"
#include "SectionJudge.h"
#include "BlockCircle.h"

extern HPolling *gHPolling;
extern Odometry *gOdo;
extern LineTracer *gLineTracer;
extern SDFile * gSDFile;
extern SectionJudge *gJudge;
extern Turn *gTurn;
extern StraightWalker *gStraightWalker;
extern ArmControl *gArmControl;



SectionCreate::SectionCreate()
{
    //param_idx=0;
    setRouteClear();
    mState = UNDEF; 
    mTNodeId = -5;
}

SectionCreate::~SectionCreate()
{
    delete mArea;
    //delete mCSection;

}

bool SectionCreate::run()
{
    //msg_f("SectionCreate::run",1);
    switch(mState){
        case UNDEF:
            execUndefined();
            break;
        case WAIT:
            execWait();
            break;
        case INIT:
            execInit();
            break;
        case POST:
            execPost();
            break;
        case GARAGE:
            execGarageIn();
            break;
        case SEARCH:
            execSearchColor();
            break;
        case ACTION:
            execAction();
            break;
        case END:
            //execEnd();
            return true;
            break;
        default:
            break;
    }
    return false;
}

void SectionCreate::execUndefined()
{
    //msg_f("SC:UDF",3);

    //setRoutes(listDummy,36);   //デバッグ用
    //setRoutes(listDummy,-1);   //デバッグ用

    mState = WAIT;
    //mState = ACTION;
}
void SectionCreate::execWait()
{
    //msg_f("SC:WAIT",3);
    
    if(extractionRoutes()){
        if(isBlockCarry()){ 
            //ブロック運搬中
            mState = POST;
            //msg_f("SC:POST",11);
        }else if(isGarageIn()){
            //ガレージイン
            mState = GARAGE;
        }else if(isSearchColor()){
            //ブロック色チェック
            mState = SEARCH;
        }else{
            //区間生成終了
            mState = END;
            //msg_f("SC:END",12);
        }
    }else{
        //区間生成中
        setRouteClear();
        param_idx = 0;
        mState = INIT;
    }
}

void SectionCreate::execInit()
{
    msg_f("SC:INIT",3);
    
    calcRoute();    //走行体の方向遷移計算
    calcAction();   //走行区間設定
    mParam[param_idx] = mParamEnd[0]; //終了処理区間設定
    setSection();   //走行区間出力
    mState = ACTION;
}

//ポスト動作実行
void SectionCreate::execPost()
{
    //msg_f("SC:POST",3);

    param_idx = 0;
    if(isBlockCarry()){ //ブロック運搬中
        calcThrow();    //スローイン区間設定
    }
    mParam[param_idx] = mParamEnd[0]; //終了処理区間設定
    setSection();   //走行区間出力
    mTNodeId = -5;  //初期化して次はスローインしないように変更
    mState = ACTION;
}

//ガレージ動作実行
void SectionCreate::execGarageIn()
{
    msg_f("SC:GARAGE",10);

    param_idx = 0;
    int i=0;    //ガレージインのために向く方向
    switch(mCourse){
        case 0: //L
            i = (int)DIR::E;
            break;
        case 1: //R
            i = (int)DIR::W;
            break;
        default:
            break;
    }
    int n = i - (int)mRunner->getDir();  //目標方向ー走行体方向から回転方向と角度を計算
    if(n<-1) n+=4;   //旋回方向範囲外になった場合は正規化
    if(n> 2) n-=4;   //旋回方向範囲外になった場合は正規化

    mCalcRoute[1] = n;  //旋回方向
    mCalcRoute[2] = -2; //ガレージイン指定

    //char buf[256];
    //sprintf(buf,"SC:G:N:%d,1:%d,2:%d",n,mCalcRoute[1],mCalcRoute[2]);
    //msg_f(buf,12);

    calcAction();   //区間決定
    mParam[param_idx] = mParamEnd[0]; //終了処理区間設定
    setSection();   //走行区間出力
    mTNodeId = -5;  //初期化して次はガレージインしないように変更
    mState = ACTION;
}


//色検出動作実行
void SectionCreate::execSearchColor()
{
    msg_f("SC:SEARCH",10);
    param_idx = 0;
    setParam(mParamEC);
    mParam[param_idx] = mParamEnd[0]; //終了処理区間設定
    setSection();   //走行区間出力
    mTNodeId = -5;  //初期化して次はガレージインしないように変更
    mState = ACTION;
}


//区間実行
void SectionCreate::execAction()
{
    msg_f("SC:ACTION",3);

    if(mCSection->run()) {
        mState = WAIT;
        //mState = END; //デバッグ用
    }
}

void SectionCreate::execEnd()
{
    //msg_f("SC:END",3);
    //char buf[256];
    //sprintf(buf,"SectionCreate : END : %d,%d,%d,%d,%d",mCalcRoute[0], mCalcRoute[1], mCalcRoute[2], mCalcRoute[3], mCalcRoute[4]);
    //msg_f(buf,11);

    //mState = UNDEF;
}

//全経路から一区間分の経路を抽出
bool SectionCreate::extractionRoutes()
{
    char buf[256];
    int i=0;
    while(mRoutes[route_idx]!=ROUTE_END && i<ONE_SECTION){  //一区間分の経路または経路終了までを抽出
        mDirections[i] = mRoutes[route_idx];    //一区間分の経路を格納
        i++;
        route_idx++;
    }
    //route_idx--;  
   // sprintf(buf,"i %d,KIND:%d",i,mArea->getBlockPlace(mDirections[0])->getKind());
   // msg_f(buf,6);

    if(i==2 && mArea->getBlockPlace(mDirections[0])->getKind() == BPKIND::BLOCK) {
        mRunner->setNext(mDirections[1]);   //次に向かうノードを仮設定
        i=1; 
    }
    if(i==3 && mArea->getBlockPlace(mDirections[0])->getKind() == BPKIND::LINE) {
        i--;
        route_idx--;
    }
    mDirections[i] = ROUTE_END; //最後尾に終了経路設定
    
    return mRoutes[route_idx--]==ROUTE_END && i==1; //経路から全区間生成終了済？
}

/*
 * 
*/
void SectionCreate::calcRoute()
{
        char buf[256];
    msg_f("SC:ACTION_CALCROUTE",3);
    int i=0;
    int l=3;    //一区間の経路数
    mCalcRoute[i] = mRunner->getDir(); //走行体の現在の方向を保存

    sprintf(buf,"SC:R:%d,%d,%d,%d",mDirections[0],mDirections[1],mDirections[2],mDirections[3]);
    msg_f(buf,11);

    if(mArea->getBlockPlace(mDirections[i])->getKind() == BPKIND::LINE){    //黒線上スタートの場合
        l = 2;
        mCalcRoute[2] = -3; 
    }

    //while(i<2){
    while(i<l-1){
        i++;
        mRunner->setNext(mDirections[i]);   //次に向かうノードを仮設定
        mCalcRoute[i] = mRunner->calcRoute(); //走行体の方向の変更を保存
        sprintf(buf,"SC:R:%d:%d,%d,%d,%d,%d",i,mCalcRoute[0],mCalcRoute[1],mCalcRoute[2],mDirections[0],mDirections[1]);
        msg_f(buf,11);
    }
    //sprintf(buf,"SC:R:%d,%d,%d,%d",mCalcRoute[0],mCalcRoute[1],mCalcRoute[2],mCalcRoute[3]);
   // msg_f(buf,11);
    msg_f("SC:ACTION_CALCROUTE_END",3);
}

/*
 * 
*/
void SectionCreate::calcAction()
{
    msg_f("SC:ACTION_CALCACTION",3);
    int i = 0;
    COLOR c = COLOR::NONE;
    int fwd;

    //プレ動作
    switch (mCalcRoute[1])
    {
    case 0: //直進
        msg_f("SC:CREATE1: S",10);
        if(mRunner->isCircleBefore()){
            //交点サークル前ー＞交点サークル後
            setParam(mParamPS);
        }else{
            //交点サークル後
        }
        break;    
    case 1: //右旋回
    case -1: //左旋回
        fwd = (gLineTracer->getEdgeMode()&&mCalcRoute[1]==1 || (!gLineTracer->getEdgeMode())&&mCalcRoute[1]==-1) ?1.5:0.0;
        mParamPT[5].len = fwd;

        if(mRunner->isCircleBefore()){
            msg_f("SC:CREATE1: T90B",10);
            //交点サークル前ー＞交点サークル後
            mParamPT[1].endFlag = Flag::END_LEN;
             mParamPT[5].endFlag = Flag::END_ALL;
       }else{
            msg_f("SC:CREATE1: T90A",10);
            //交点サークル後ー＞交点サークル後
            mParamPT[1].endFlag = Flag::END_ALL;
            mParamPT[5].endFlag = Flag::END_LEN;
        }

        // 黒線から旋回の場合
        if(mRunner->isOnLine()) {
            mParamPT[1].endFlag = Flag::END_ALL;
            mParamPT[5].endFlag = Flag::END_ALL;
        }


        if(isBlockCarry()){
            mParamPT[2].fwd = 2.0d;
            mParamPT[3].fwd = 2.0d;
        }else{
            mParamPT[2].fwd = 2.0d;
            mParamPT[3].fwd = 2.0d;
        }
        mParamPT[2].target = mCalcRoute[1] * 75.0d;
        mParamPT[3].target = mCalcRoute[1] * 89.0d;
        mParamPT[6].target = -mCalcRoute[1];
        setParam(mParamPT);
        break;
    case 2: //反転
        msg_f("SC:CREATE1: B",10);
        if(mRunner->isCircleBefore()){
            //交点サークル前
            mParamPB[1].endFlag = Flag::END_LEN;
        }else{
            //交点サークル後
            mParamPB[1].endFlag = Flag::END_ALL;
        }

        if(isBlockCarry()) {
            mParamPT[2].fwd = 3.0d;
            mParamPT[3].fwd = 3.0d;

            if( gLineTracer->getEdgeMode()){
                //左エッジ
                i = Turn::RIGHT;
            }else{
                //右エッジ
                i = Turn::LEFT;
            }
        } else {
            mParamPT[2].fwd = 2.0d;
            mParamPT[3].fwd = 2.0d;

            if( gLineTracer->getEdgeMode()){
                //左エッジ
                i = Turn::LEFT;
            }else{
                //右エッジ
                i = Turn::RIGHT;
            }

        }

        if(isBlockCarry()){
            //ブロック運搬中
            mParamPB[2].turn = 10.0d;
            mParamPB[3].turn = 7.0d;
        }else{
            //ブロックなし
            mParamPB[2].turn = 25.0d;
            mParamPB[3].turn = 10.0d;
        }
        mParamPB[2].target = i * 165.0d;
        mParamPB[3].target = i * 178.0d;
        mParamPB[4].target = -i;

        setParam(mParamPB);
        break;
    default:
        msg_f("SC:CREATE1: ERR",10);
        break;
    }

    //主動作
    switch (mCalcRoute[2])
    {
    case 0: //直進  //交点から交点
        msg_f("SC:CREATE2: S",8);
        mParamMS[3].target = mRunner->getNextColor(); 
        setParam(mParamMS);
        mRunner->setCircleBefore(true);
        break;
    case 1: //右旋回、黒ブロック回収
    case -1: //左旋回、黒ブロック回収
        i = mDirections[1]*2-mDirections[0];    //仮移動ノードID
        c = ((BlockCircle*)(mArea->getBlockPlace(i)))->getColor();    //仮移動ノード色
        char buf[256];
        sprintf(buf,"SC*CR2:%d,%d",i,c);
        msg_f(buf,9);
        mParamMT[3].target = c; 
               // mParamMT[3].target = 1; 

        //mParamMT[6].target = c; 
        mParamMT[6].target = mCalcRoute[2] * 75.0d; //8
        mParamMT[7].target = mCalcRoute[2] * 90.0d; //9
        
         mRunner->setCircleBefore(false);
         mRunner->setOnLine(true);

        setParam(mParamMT);
        break;
    case 2: //後退、ブロック退避
        i = mRunner->getDir() + 2;
        if(i>3)i=i-4;
        mRunner->setDir((DIR)i);
        setParam(mParamMB);
        mRunner->setCircleBefore(false);
        break;
    case -2: //ガレージイン
        msg_f("SC:ACTION:GARAGE:",11);
        switch(mCourse){
            case 0: //L
                msg_f("SC:ACTION:GARAGE:L",11);
                setParam(mParamEL);
                break;
            case 1: //R
                msg_f("SC:ACTION:GARAGE:R",11);
                setParam(mParamER);
                break;
            default:
                break;
        }
        break;
    case -3: //黒線スタートの前進
        if(mCalcRoute[1]==0){
            //msg_f("taihi bk",6);
            //退避ブロック回収
            mParamEB[2].endFlag = Flag::END_ALL;
            //mParamEB[3].endFlag = Flag::END_ALL;
            mParamEB[4].len = 5;
        }else{
            // msg_f("bonus bk ",6);
           //黒ブロック回収後
            mParamEB[2].endFlag = Flag::END_LEN;
            //mParamEB[3].endFlag = Flag::END_LEN;
           // mParamEB[4].endFlag = Flag::END_LEN;
            mParamEB[4].len = 13; 
        }
        mParamEB[6].target = mRunner->getNextColor();

        setParam(mParamEB);
        mRunner->setCircleBefore(true);
        break;
    default://行動無し
        break;
    }
}

/*
 * 
*/
void SectionCreate::calcThrow()
{
    msg_f("SC:ACTION_CALCTHROW",3);
    char buf[256];
    //sprintf(buf,"SC:TI:NID:%d,SID:%d",mRunner->getNextNodeId(),mRunner->getStart()->getNodeid());
    //msg_f(buf,6);
    BlockPlace **bp = mRunner->getNextBlockPlace()->getSlashPlaces();   //交点ノード周囲のブロックサークル確認
    int i=0;
    //sprintf(buf,"SC:T:ID:%d,1:%d,2:%d",mTNodeId,bp[1]->getNodeid(),bp[2]->getNodeid());
    //msg_f(buf,6);
    
    while(i<4){
        if(bp[i] != nullptr && bp[i]->getNodeid() == mTNodeId){ //スローイン先のノードIDが周囲のブロックサークルに存在するか
            break;
        }
        i++;
    }

    if(i<4){   //存在する場合はスローイン
        int n = i - (int)mRunner->getDir();  //ノード位置ー走行体方向から回転方向と角度を計算
        if(n<0) n+=4;   //負数になった場合は正数化
        int sign = 1;
        int angle = 0;
        mParamET[7+4].fwd = 0.0d; //スローイン後の前進値をブロックなしの値に設定
        mParamET[8+4].fwd = 0.0d; //スローイン後の前進値をブロックなしの値に設定
        mParamET[11+4].endFlag = Flag::END_ALL;

        bool edge = gLineTracer->getEdgeMode();

        switch (n){ //旋回角度決定
            case 1: //右135°旋回
                mParamET[7+4].fwd = 3.0d; //スローイン後の前進値を修正
                mParamET[8+4].fwd = 3.0d; //スローイン後の前進値を修正
                mRunner->turnRunner(1);
                angle = 90;
                mParamET[11+4].endFlag = Flag::END_LEN;
            case 0: //右45°旋回
                mRunner->turnRunner(1);
                mParamET[9+4].target = Turn::LEFT;
                mParamET[10].len = (!edge)?-1.5:-1.0;

                break;
            case 2: //左135°旋回
                mParamET[7+4].fwd = 3.0d; //スローイン後の前進値を修正
                mParamET[8+4].fwd = 3.0d; //スローイン後の前進値を修正
                mRunner->turnRunner(-1);
                angle = -90;
                mParamET[11+4].endFlag = Flag::END_LEN;

            case 3: //左45°旋回
                mRunner->turnRunner(-1);
                sign = -1;
                mParamET[9+4].target = Turn::RIGHT;
                mParamET[10].len = (edge)?-1.5:-1.0;

                break;            
            default:
                break;
        } 
        mParamET[2].target = 30 * sign + angle;
        mParamET[3].target = 45 * sign + angle;  // 44から変更 9/10
        mParamET[7+4].target = 75 * sign + angle;
        mParamET[8+4].target = 90 * sign + angle;  //89から変更 9/10


        setParam(mParamET);
        mRunner->setCircleBefore(false);

    }else{
        msg_f("SC:ACTION_CALCTHROW_ERR",12);
    }
}

void SectionCreate::setRoutes(int r[],int tb)
{
    //msg_f("SC:SET_ROUTES",3);
    char buf[256];
    sprintf(buf,"SC:S_R: R : %d,%d,%d,%d,%d,%d",r[0],r[1],r[2],r[3],r[4],r[5]);
    msg_f(buf,12);

    int i=0;
    while(r[i]!=ROUTE_END){
        mRoutes[i] = r[i];
        i++;
    }
    mRoutes[i] = ROUTE_END;

    mTNodeId = tb;
    route_idx = 0;
    //sprintf(buf,"SC:SR:i%d:R:%d,%d,%d,%d,%d,%d",i,mRoutes[0],mRoutes[1],mRoutes[2],mRoutes[3],mRoutes[4],mRoutes[5]);
    //msg_f(buf,9);
    mState = UNDEF;   //非デバッグ用
}

void SectionCreate::setRouteClear()
{
    mCalcRoute[0] = -5;
    mCalcRoute[1] = -5;
    mCalcRoute[2] = -5;
    mCalcRoute[3] = -5;
}

/*
 * 
*/
void SectionCreate::setParam(PParam* p)
{
    //msg_f("SC:C_SP:",8);
    char buf[256];
    //sprintf(buf,"SC:P:%d,%d,%d,%d,%d,%d,%d,%d",(int)p[0].runFlag,(int)p[1].runFlag,(int)p[2].runFlag,(int)p[3].runFlag,(int)p[4].runFlag,(int)p[5].runFlag,(int)p[6].runFlag,(int)p[7].runFlag);
    //msg_f(buf,6);
    //sprintf(buf,"SC:P_idx:%d",param_idx);
    //msg_f(buf,5);
    int i=0;
    while(p[i].runFlag != Flag::RUN_END){
        //sprintf(buf,"SC:P_idx:%d,i%d",param_idx,i);
        //msg_f(buf,5);
        mParam[param_idx] = p[i];
        i++;
        param_idx++;
    }
}

void SectionCreate::setCompositeSection(Section* cs){
    //msg_f("SC:SET_CS",3);
    mCSection = cs;
}

void SectionCreate::setCourse(int course){
    //msg_f("SC:SET_COURSE",3);
    mCourse = course;

    mArea = new Area(mCourse);    //非デバッグ用
    //mArea = new Area(1);    //デバッグ用
    mRunner = mArea->getRunner();
    mRunner->setArea(mArea);
    
}

//区間出力
void SectionCreate::setSection()
{
    msg_f("SC:ACTION_SET",3);
    int i=0;
    while(mParam[i].runFlag != Flag::RUN_END){  //終了処理以外の区間を出力
        ((CompositeSection*)mCSection)->setSection(mParam[i],i);
        i++;
    }
    ((CompositeSection*)mCSection)->setSection(mParam[i],i);    //終了処理区間を出力
}

bool SectionCreate::isBlockCarry()
{
    return mTNodeId >= 0;
}

bool SectionCreate::isSearchColor()
{
    return mTNodeId == -3;
}

bool SectionCreate::isGarageIn()
{
    return mTNodeId == -2;
}

