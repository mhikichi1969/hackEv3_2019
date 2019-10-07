#ifndef __SECTION_CREATE_H__
#define __SECTION_CREATE_H__


#include "BlockPlace.h"
#include "Area.h"
#include "Runner.h"
#include "ParamStruct.h"
#include "Flag.h"
#include "CompositeSection.h"
#include "Const.h"


class SectionCreate
{
    public:
        SectionCreate();
        ~SectionCreate();
        bool run();

        void execUndefined();
        void execWait();
        void execInit();
        void execPost();
        void execGarageIn();
        void execSearchColor();

        void execAction();
        void execEnd();

        bool extractionRoutes();
        void calcRoute();
        void calcAction();
        void calcThrow();

        void setCourse(int course);
        void setRoutes(int r[],int tb);
        void setParam(PParam* p);
        void setCompositeSection(Section* cs);
        void setSection();
        void setRouteClear();

        bool isBlockCarry();  //ブロック運搬中？
        bool isSearchColor(); //ブロック色不明？
        bool isGarageIn();    //ガレージインする？

    protected:
        Section *mCSection; //CompositeSection
        static const int ROUTE_END = 999;     //区間完了の定数
        static const int ONE_SECTION = 3;     //一区間あたりに想定されている経路数の上限
        //static const double S_POW = 23.0d;    //直進、ライントレース時にPID制御を行う場合に適切なfwd値

    private:
        enum State {
            UNDEF,
            WAIT,
            INIT,
            POST,
            GARAGE,
            SEARCH,
            ACTION,
            END
        };  

        Area *mArea;
        Runner *mRunner;

        int mRoutes[100];       //受け取った経路を保持
        State mState;
        int mCalcRoute[4];  //走行体の方向の遷移を保持
        int mDirections[10];//受け取った経路のうち実行単位の区間を保持
        int param_idx=0;    //１区間の処理状況を管理するインデックス
        int route_idx=0;    //１区間から実行単位での抽出状況を管理するインデックス
        int mCourse;        //0ならLコース、1ならRコース
        bool mTFlag;        //スローイン実施フラグ、直前がスローイン区間の場合にTRUE、それ以外の場合にFALSE
        int mTNodeId;       //+：スローイン先のブロックサークルのノードID、-5：初期値、未定義状態、-2：ガレージイン、-3：ブロック色チェック

        int listDummy[20] = {//受け取る経路の想定、デバッグ用
            //34,33,32,ROUTE_END  //前進用、交点ノード、前方黒線、前方交点ノード、END
            //34,26,ROUTE_END //スローイン用、交点ノード、ブロックサークル、END
            //34,ROUTE_END //スローイン用、交点ノード、END
            //34,33,32,31,30,ROUTE_END 
            //30,29,28,21,14,ROUTE_END 
            //34,27,20,13,6,5,4,3,2,9,16,ROUTE_END 
            34,27,20,ROUTE_END 
            //34,33,32,25,18,19,20,27,34,ROUTE_END
            //34,27,20,ROUTE_END
            //32,31,30,ROUTE_END
            //34,41,48,ROUTE_END
            //34,33,40,ROUTE_END
            //34,33,40,47,46,ROUTE_END
        };

        
        PParam  mParam[100];   //double fwd ,double target ,double len ,double turn ,Method runFlag ,End endFlag
        PParam  mParamEnd[1] = {
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }    //終了処理
        };

        //プレ動作
        PParam  mParamPS[5] = {    //交点サークル前から交点サークル後への直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.4  ,0.0d                   ,8     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{ 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamPT[9] = {    //交点サークル前から交点サークル後への90°旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.4  ,0.0d                   ,7      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進 
          { 1.0d  , 75.0d                 ,0      ,11 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°右旋回⓵ /15
          { 1.0d  , 88.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°右旋回⓶
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,0.0d                   ,1.0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進

          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          //{0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamPB[9] = {    //交点サークル前から交点サークル後まで前進後に180°旋回
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.4  ,0.0d                   ,8      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 2.0d  ,165.0d                 ,0      ,8 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し180°右旋回⓵
          { 2.0d  ,179.0d                 ,0      ,7  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し180°右旋回⓶
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,0.0d                   ,1.5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        //主動作
        PParam  mParamMS[7] = {    //直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,0.0d                   ,18     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース 21から変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,(double)COLOR::NONE    ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
         // { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamMT[16] = {    //黒ブロック回収
          //前進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.8  ,0.0d                   ,21     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.8  ,(double)COLOR::NONE    ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          //{15.0d  ,0.0d                   ,5      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //一定距離まで直進
          //{10.0d  ,(double)COLOR::NONE    ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //既定の色まで直進
          //後退、旋回、黒ブロック回収
          {-S_POW*1.2 ,0.0d                   ,-6    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 1.0d  , 75.0d                 ,0      ,12 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°右旋回⓵
          { 1.0d  , 89.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°右旋回⓶
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.8  ,0.0d                   ,25     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          {S_POW*0.5  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_BLK },  //黒線を検知するまで直進
          {10.0d  ,0.0d                   ,2.0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamMB[8] = {    //ブロック退避
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
        //  {0.0d   ,15.0d                  ,0      ,0  ,Flag::RUN_TURN    ,Flag::END_CNT },  //一定時間(target*4ms)
          {0  ,0.0d                   ,0     ,0  ,Flag::RUN_STRAIGHT     ,Flag::END_ALL },  //停止
          {S_POW*0.5  ,0.0d                   ,5     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {-S_POW ,0.0d                   ,-4    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          {0.0d   ,15.0d                  ,0      ,0  ,Flag::RUN_TURN    ,Flag::END_CNT },  //一定時間色取得(target*4ms)
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          { 0.0d  ,0                 ,0      ,0       ,Flag::RUN_TURN     ,Flag::END_ALL },  //停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        //ポスト動作
        PParam  mParamET[18] = {    //スローイン
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*0.4  ,0.0d                   ,7.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進   (5.0から7.0に変更9/10)
          { 2.0d  , 30.0d                 ,0      ,12 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック保持90°右旋回⓵
          { 2.0d  , 44.0d                 ,0      ,7  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック保持90°右旋回⓶
          { 0.0d  ,  0.0d                 ,0      ,0  ,Flag::RUN_TURN     ,Flag::END_ALL },  //旋回停止
          //前進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,0.0d                   ,1.5    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 0.0d  ,  0.0d                 ,0      ,0  ,Flag::RUN_TURN     ,Flag::END_ALL },  //ダミー

          { 80.0d ,80.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ARM },  //ブロックをスローイン
          { 0.0d  , 0.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //スローイン後にアームを既定の位置に戻す
          
          //後退
          {-S_POW  ,0.0d                   ,-1.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進


          { 1.0d  , 75.0d                 ,0      ,17 ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°右旋回⓷
          { 1.0d  , 88.0d                 ,0      ,7  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°右旋回⓸
          {0      ,Turn::LEFT             ,0      ,0  ,Flag::RUN_EDGE     ,Flag::END_ALL },  //ライントレース走行用エッジを左に変更
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,0.0d                   ,1.0    ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進   (5.0から7.0に変更9/10)


          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamER[16] = {    //1-4間からガレージイン（R)
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {120.0d ,80.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //アームを最大まで持ち上げる
          {S_POW*0.9  ,0.0d                   ,43     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
           {0.0d   ,100.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT   ,Flag::END_CNT }, 
          { 1.0d  ,-70.0d                 ,0      ,25 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
          { 1.0d  ,-85.0d                 ,0      ,8 ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°左旋回⓶
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW*3  ,0.0d                   ,38     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamEL[16] = {    //5-8間からガレージイン（L)
          { 1.0d  ,-10.0d                 ,0      ,15 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {120.0d ,80.0d                  ,0      ,0  ,Flag::RUN_THROW    ,Flag::END_ALL },  //アームを最大まで持ち上げる
          {S_POW*1.2  ,0.0d                   ,39     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで直進
          { 1.0d  ,-55.0d                 ,0      ,20 ,Flag::RUN_TURN     ,Flag::END_ANG },  //ブロック無し90°左旋回⓵
          { 1.0d  ,-67.0d                 ,0      ,5  ,Flag::RUN_TURN     ,Flag::END_ANG2},  //ブロック無し90°左旋回⓶
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };

        PParam  mParamEB[10] = {    //黒線スタートからの前進
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {-S_POW*0.6 ,0.0d                   ,-10     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {S_POW*0.8  ,0.0d                   ,13      ,0  ,Flag::RUN_LINE   ,Flag::END_LEN },  //
    //      {S_POW*0.8  ,0.0d                   ,5     ,0  ,Flag::RUN_LINE     ,Flag::END_LEN },  //一定距離までライントレース
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {S_POW  ,(double)COLOR::NONE    ,0     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_COL },  //既定の色まで直進
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
         // { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };



        PParam  mParamEC[14] = {    //ブロック色確認
        //  {0.0d   ,-1.0d                  ,0      , 1  ,Flag::RUN_TURN      ,Flag::END_ANG },  //アーム操作
          {0.0d   ,15.0d                  ,0      ,0  ,Flag::RUN_TURN    ,Flag::END_CNT },  //一定時間色取得(target*4ms)

          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          {-S_POW*0.5 ,0.0d                   ,-2     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで後退
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止

          {0.0d   ,60.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作
          {0.0d   ,15.0d                  ,0      ,0  ,Flag::RUN_COLOR    ,Flag::END_CNT },  //一定時間色取得(target*4ms)
          {0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_SAVE     ,Flag::END_ALL },  //ブロック色保存
          {0.0d   , 0.0d                  ,0      ,0  ,Flag::RUN_ARM      ,Flag::END_ARM },  //アーム操作解除

          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施
          { 0.0d  ,  0.0d                 ,0      , 0 ,Flag::RUN_TURN     ,Flag::END_ALL },  //スピードコントロール解除
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_RECORD   ,Flag::END_ALL },  //レコードカウント実施

          {S_POW*0.3  ,0.0d                   , 2     ,0  ,Flag::RUN_STRAIGHT ,Flag::END_LEN },  //一定距離まで前進
          { 0.0d  ,0.0d                   ,0      ,0  ,Flag::RUN_STRAIGHT ,Flag::END_ALL },  //直進停止
          {0.0d   ,0.0d                   ,0      ,0  ,Flag::RUN_END      ,Flag::END_UDF }   //終了処理
        };
};
#endif
