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
    mRestore=-1;
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
    //msg_f("SC:INIT",3);
    
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
   // msg_f("SC:GARAGE",10);

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
  //  msg_f("SC:SEARCH",10);
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
    //msg_f("SC:ACTION",3);

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
  //  msg_f("SC:ACTION_CALCROUTE",3);
    int i=0;
    int l=3;    //一区間の経路数
    mCalcRoute[i] = mRunner->getDir(); //走行体の現在の方向を保存

   // sprintf(buf,"SC:R:%d,%d,%d,%d",mDirections[0],mDirections[1],mDirections[2],mDirections[3]);
  //  msg_f(buf,11);

    if(mArea->getBlockPlace(mDirections[i])->getKind() == BPKIND::LINE){    //黒線上スタートの場合
        l = 2;
        mCalcRoute[2] = -3; 
    }

    //while(i<2){
    while(i<l-1){
        i++;
        mRunner->setNext(mDirections[i]);   //次に向かうノードを仮設定
        mCalcRoute[i] = mRunner->calcRoute(); //走行体の方向の変更を保存
       // sprintf(buf,"SC:R:%d:%d,%d,%d,%d,%d",i,mCalcRoute[0],mCalcRoute[1],mCalcRoute[2],mDirections[0],mDirections[1]);
       // msg_f(buf,11);
    }
    //sprintf(buf,"SC:R:%d,%d,%d,%d",mCalcRoute[0],mCalcRoute[1],mCalcRoute[2],mCalcRoute[3]);
   // msg_f(buf,11);
    //msg_f("SC:ACTION_CALCROUTE_END",3);
}

/*
 * 
*/
void SectionCreate::calcAction()
{
   // msg_f("SC:ACTION_CALCACTION",3);
    int i = 0;
    COLOR c = COLOR::NONE;
    double fwd;

    bool restore=false;
    bool fast_turn=false;
    //プレ動作
    switch (mCalcRoute[1])
    {
    case 0: //直進
       // msg_f("SC:CREATE1: S",10);
        if(mRunner->isCircleBefore()){
            //交点サークル前ー＞交点サークル後
            setParam(mParamPS);
        }else{
            //交点サークル後
        }
        break;    
    case 1: //右旋回
    case -1: //左旋回
        restore = (mRestore!=-1 && mRestore==mDirections[1])?true:false;

        // 旋回方向確定
        mParamPT[3].target = mCalcRoute[1] * 65.0d;
        mParamPT[4].target = mCalcRoute[1] * 89.0d;
        mParamPT[7].target = -mCalcRoute[1];

         // 旋回後に進む距離　エッジと逆方向の場合は進む
        fwd = (gLineTracer->getEdgeMode()&&mCalcRoute[1]==1 || (!gLineTracer->getEdgeMode())&&mCalcRoute[1]==-1) ?3.0:0.0;

        mParamPT[6].len = fwd;
        mParamPT[1].endFlag = Flag::END_LEN;
        //プレ処理の設定
        if(mRunner->isCircleBefore()){
          //  msg_f("SC:CREATE1: T90B",10);
            //交点サークル前ー＞交点サークル後
            //mParamPT[1].endFlag = Flag::END_LEN;
            mParamPT[1].fwd = S_POW*0.35;
            //mParamPT[1].len = 6.0;

            mParamPT[1].endFlag = Flag::END_LEN;
            mParamPT[6].endFlag = Flag::END_ALL;
            
            if(isBlockCarry()){
                mParamPT[1].len = 3.5d; // ブロック保持の場合に最初の前進距離
            }else{
                mParamPT[1].len = 7.5d; // ブロック無しの場合に最初の前進距離
            }
            
       }else{ // サークル後ろからのプレ処理
           // msg_f("SC:CREATE1: T90A",10);
            //交点サークル後ー＞交点サークル後
            //mParamPT[1].endFlag = Flag::END_ALL;
            mParamPT[1].fwd = -10.0d;
            mParamPT[1].len = -0.5;

            mParamPT[6].endFlag = Flag::END_LEN;
        }

        // 交点サークル手前、黒線上ではない、黒ブロック回収ではない,退避ではない、ガレージインではない場合に高速旋回
        if( mRunner->isCircleBefore() && !mRunner->isOnLine() && 
            mCalcRoute[2]!=1 && mCalcRoute[2]!=-1 && 
            mCalcRoute[2]!=2 && mCalcRoute[2]!=-2 &&
             !restore) {     
      #if 1

            if(isBlockCarry()){
                mParamPVT[1].fwd=14;
                mParamPVT[1].target = mCalcRoute[1] * 84.0d; 
                mParamPVT[1].len = -8; // 旋回のオフセット値に使用
            }else {
                mParamPVT[1].fwd=14;
                mParamPVT[1].target = mCalcRoute[1] * 83.0d; // 軽い状態で旋回速度が速いので早めに止める
                mParamPVT[1].len = 1; // 旋回のオフセット値に使用

            }
            
            if(mCalcRoute[2]!=2 && mCalcRoute[2]!=-2)
                mParamPVT[0].endFlag =Flag::END_ALL; // 退避ならブレーキ
            else
                mParamPVT[0].endFlag =Flag::END_LEN;
                


            mParamPVT[1].turn = mCalcRoute[1]*8.3; // 旋回半径 6.5 走行体により変える？
            mParamPVT[2].target = -mCalcRoute[1]; // エッジの変更
            setParam(mParamPVT);
            fast_turn=true;
            break;
        #else
            mParamPT[1].endFlag = Flag::END_ALL;

            if(isBlockCarry()){
                mParamPT[3].fwd = 16.0d;
                mParamPT[4].fwd = 10.0d;
                mParamPT[3].turn = 16.0d; // ブロックが重いので強めに
                mParamPT[4].turn = 10.0d;
            } else {
                mParamPT[3].fwd = 15.0d;
                mParamPT[4].fwd = 10.0d;
                mParamPT[3].turn = 14.0d; 
                mParamPT[4].turn = 10.0d;
            }
        #endif
        } else {           
            //ブロック有旋回と、ブロック無し旋回のパワー設定
            if(isBlockCarry()){
                mParamPT[3].fwd = 7.0d;
                mParamPT[4].fwd = 3.0d;
                mParamPT[3].turn = 11.0d;
                mParamPT[4].turn = 6.0d;
            }else{
                mParamPT[3].fwd = 2.0d;
                mParamPT[4].fwd = 1.0d;
                mParamPT[3].turn = 14.0d;
                mParamPT[4].turn = 6.0d;
            }    
        }


        // 黒線から旋回の場合 交点サークル前後の場合はfalse
        if(mRunner->isOnLine()) {
            mParamPT[1].endFlag = Flag::END_ALL;
            mParamPT[6].endFlag = Flag::END_ALL;

            // 線上ではすこし早めに
            mParamPT[3].fwd = 4.0d;
            mParamPT[4].fwd = 3.0d;
            mParamPT[3].turn = 13.0d;
            mParamPT[4].turn = 6.0d;

            // 線上ではしっかり旋回
            mParamPT[3].target = mCalcRoute[1] * 65.0d;
            mParamPT[4].target = mCalcRoute[1] * 89.0d;

        }

        setParam(mParamPT);

        break;
    case 2: //反転
      //  msg_f("SC:CREATE1: B",10);
        if(mRunner->isCircleBefore()){
            //交点サークル前
            mParamPB[1].endFlag = Flag::END_LEN;
        }else{
            //交点サークル後
            mParamPB[1].endFlag = Flag::END_ALL;
        }

        if(isBlockCarry()) {
            mParamPB[2].fwd = 6.0d;
            mParamPB[3].fwd = 6.0d;
            mParamPB[4].fwd = 6.0d;

            if( gLineTracer->getEdgeMode()){
                //左エッジ
                i = Turn::RIGHT;
            }else{
                //右エッジ
                i = Turn::LEFT;
            }
        } else {
            mParamPB[2].fwd = 0.0d;
            mParamPB[3].fwd = 0.0d;
            mParamPB[4].fwd = 0.0d;

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

            mParamPB[2].target = i * -15.0d;
            mParamPB[2].turn = 11.0d;
            mParamPB[2].endFlag = Flag::END_ANG;
            mParamPB[3].turn = 13.0d;
            mParamPB[3].target = i * 180.0d;
            mParamPB[3].endFlag = Flag::END_ANG2;

            mParamPB[4].turn = 10.0d;
            mParamPB[4].target = i *220.0d;
            mParamPB[4].endFlag = Flag::END_ANG2;
            mParamPB[5].target = i * 195.0d;
            mParamPB[5].turn = 8.0d;

            mParamPB[5].endFlag = Flag::END_ANG2;
            mParamPB[8].len = 6;

        }else{
            //ブロックなし
            mParamPB[2].turn = 0.0d;
            mParamPB[2].endFlag = Flag::END_ALL;
            mParamPB[3].endFlag = Flag::END_ANG;
            mParamPB[3].turn = 25.0d;
            mParamPB[3].target = i * 130.0d;
            mParamPB[4].turn = 6.0d;
            mParamPB[4].target = i * 178.0d; 
            mParamPB[4].endFlag = Flag::END_ANG2;
            mParamPB[5].turn = 0.0d;
            mParamPB[5].endFlag = Flag::END_ALL;
            mParamPB[8].len = 1.5;

        }
        //mParamPB[2].target = i * -5.0d;
        //mParamPB[4].target = i * 185.0d;
        //mParamPB[5].target = i * -180.0d;
        mParamPB[6].target = -i; //　エッジ変更

        setParam(mParamPB);
        break;
    default:
        msg_f("SC:CREATE1: ERR",10);
        break;
    }

    //主動作

    double  adj;
    switch (mCalcRoute[2])
    {
    case 0: //直進  //交点から交点
      //  msg_f("SC:CREATE2: S",8);
        mParamMS[1].len = fast_turn?(13+fwd):18; // 高速旋回の後は距離が短い?

        mParamMS[3].target = mRunner->getNextColor(); 
        setParam(mParamMS);
        mRunner->setCircleBefore(true);
        break;
    case 1: //右旋回、黒ブロック回収
    case -1: //左旋回、黒ブロック回収
        i = mDirections[1]*2-mDirections[0];    //仮移動ノードID
        c = ((BlockCircle*)(mArea->getBlockPlace(i)))->getColor();    //仮移動ノード色
        char buf[256];
       // sprintf(buf,"SC*CR2:%d,%d",i,c);
       // msg_f(buf,9);
        mParamMT[3].target = c; 
               // mParamMT[3].target = 1; 

        //mParamMT[6].target = c; 
        mParamMT[8].target = mCalcRoute[2] * 75.0d; //8
        mParamMT[9].target = mCalcRoute[2] * 89.0d; //9
        
         mRunner->setCircleBefore(false);
         mRunner->setOnLine(true);

        setParam(mParamMT);
        break;
    case 2: //後退、ブロック退避
        i = mRunner->getDir() + 2;
        if(i>3)i=i-4;
        mRunner->setDir((DIR)i);

        mParamMB[2].len=fast_turn?3.5:8.0;   // あまり長くとると、Lコースの初手の退避が進みすぎる
        mParamMB[2].len=mCalcRoute[1]==0?3.5:mParamMB[2].len; // 直進後は短く

        setParam(mParamMB);
        mRunner->setCircleBefore(false);
        break;
    case -2: //ガレージイン
      //  msg_f("SC:ACTION:GARAGE:",11);
        switch(mCourse){
            case 0: //L
               // msg_f("SC:ACTION:GARAGE:L",11);
                setParam(mParamEL);
                break;
            case 1: //R
                //msg_f("SC:ACTION:GARAGE:R",11);
                adj = (mCalcRoute[1]==2)?2:0;  // １８０度反転時は距離調整
                mParamER[0].len = 29+adj; 
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
            mParamEB[1].endFlag = Flag::END_ALL;
            //mParamEB[3].endFlag = Flag::END_ALL;
            mParamEB[3].len = 5;
        }else{
            // msg_f("bonus bk ",6);
           //黒ブロック回収後
            mParamEB[1].endFlag = Flag::END_LEN;
            //mParamEB[3].endFlag = Flag::END_LEN;
           // mParamEB[4].endFlag = Flag::END_LEN;
            mParamEB[3].len = 11; 
        }
        mParamEB[5].target = mRunner->getNextColor();

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
   // msg_f("SC:ACTION_CALCTHROW",3);
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
        mParamET[7+5].fwd = 0.0d; //スローイン後の前進値をブロックなしの値に設定
        mParamET[8+5].fwd = 0.0d; //スローイン後の前進値をブロックなしの値に設定
        mParamET[11+5].endFlag = Flag::END_ALL;  // スロー後の旋回の後に前に進むか

        bool edge = gLineTracer->getEdgeMode();

        mParamET[2].turn = 15;
        mParamET[3].turn = 6;
        mParamET[2].fwd = 2;
        mParamET[3].fwd = 2;


        switch (n){ //旋回角度決定
            case 1: //右135°旋回
                mParamET[1].len = (!edge)?2.0:3.5; // 同一方向エッジからは少なめに前進
                mParamET[2].turn = 14; 
                mParamET[3].turn = 6;
                mParamET[2].fwd = 6;
                mParamET[3].fwd = 4;

                mParamET[6].len = (!edge)?3.5:4.5; // 左エッジからのスローは距離を多め
               // mParamET[10].len = (!edge)?-4.5:-3.0; // 同一方向のエッジからは多めに下がる
                mParamET[10].len = (!edge)?-2.0:-2.0; // 共通

                mParamET[7+5].fwd = -1.0d; //スローイン後の前進値を修正
                mParamET[8+5].fwd = -0.0d; //スローイン後の前進値を修正
                mRunner->turnRunner(1);
                angle = 90;
                mParamET[11+5].endFlag = Flag::END_LEN; // 135度スローの後の前進
                mParamET[11+5].fwd = -S_POW*0.5;
                mParamET[11+5].len = (!edge)?-2.5:-1.0; ; // 135度スローの後の前進

               // mRunner->turnRunner(1);
                mParamET[9+5].target = Turn::RIGHT;
                break;
            case 0: //右45°旋回
                //mParamET[1].len = (!edge)?4.5:5.0; // 同一方向エッジからは少なめに前進
                mParamET[1].len = (!edge)?5.5:3.5; // 逆エッジ(左）からは少なめに前進
                mRunner->turnRunner(1);
                mParamET[9+5].target = Turn::LEFT;
                //mParamET[6].len = (!edge)?1.5:3.5; // 左エッジからのスローは距離を多め
                mParamET[6].len = (!edge)?3.0:6.0; // 左エッジからのスローは距離を多め

                mParamET[10].len = (!edge)?-1.7:-1.3; //共通
                mParamET[11+5].endFlag = Flag::END_ALL;
              //  mParamET[11+5].fwd = S_POW*0.5;
               // mParamET[11+5].len = (!edge)?1.5:0; // 

                break;
            case 2: //左135°旋回
                mParamET[1].len = (edge)?2.0:3.5; // 同一方向エッジからは少なめに前進
                mParamET[2].turn = 14;
                mParamET[3].turn = 6;
                mParamET[2].fwd = 6;
                mParamET[3].fwd = 4;
                mParamET[6].len = (edge)?3.5:4.5; // 左エッジからのスローは距離を多め

                //mParamET[10].len = (edge)?-4.5:-3.0; // 同一方向のエッジからは下がる
                mParamET[10].len = (edge)?-2.0:-2.0; // 共通


                mParamET[7+5].fwd = -1.0d; //スローイン後の前進値を修正
                mParamET[8+5].fwd = -0.0d; //スローイン後の前進値を修正
                mRunner->turnRunner(-1);
                angle = -90;
                mParamET[11+5].endFlag = Flag::END_LEN;                
                mParamET[11+5].fwd = -S_POW*0.5;
                mParamET[11+5].len =  (edge)?-2.5:-1.0; // 135度スローの後の前進
               // mRunner->turnRunner(-1);
                sign = -1;
                mParamET[9+5].target = Turn::LEFT;
                //mParamET[6].len = (edge)?1.5:3.0; // 右エッジからのスローは距離を多め
               // mParamET[10].len = (edge)?-1.0:-1.0;

                break;
            case 3: //左45°旋回
                mParamET[1].len = (edge)?5.5:3.5; // 逆エッジからは少なめに前進
                mRunner->turnRunner(-1);
                sign = -1;
                mParamET[9+5].target = Turn::RIGHT;
                mParamET[6].len = (edge)?3.0:6.0; 
                mParamET[10].len = (edge)?-1.7:-1.3; // 戻りは共通
                mParamET[11+5].endFlag = Flag::END_ALL;
                //mParamET[11+5].fwd = S_POW*0.5;
                //mParamET[11+5].len = (edge)?1.5:0; // 

                break;            
            default:
                break;
        } 
        if(n==0 || n==3) { //45度スロー
            mParamET[2].target = 30 * sign + angle;
            mParamET[3].target = 44 * sign + angle;  // 44から変更 9/10
            mParamET[7+5].target = 70 * sign + angle;
            mParamET[8+5].target = 89 * sign + angle;  //89から変更 9/10
        } else {
            mParamET[2].target = 30 * sign + angle;
            mParamET[3].target = 45 * sign + angle;  // 44から変更 9/10
            mParamET[7+5].target = 30 * sign + angle;
            mParamET[8+5].target = 1*sign + angle;  //89から変更 9/10
        }


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
   // sprintf(buf,"SC:S_R: R : %d,%d,%d,%d,%d,%d",r[0],r[1],r[2],r[3],r[4],r[5]);
   // msg_f(buf,12);

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
   // msg_f("SC:ACTION_SET",3);
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

void SectionCreate::setGetResotoredBlockMode(int restore)
{
    mRestore = restore;
}
