#ifndef _SCENE_WALKER_H_
#define _SCENE_WALKER_H_
#include "Tracer.h"
#include "SpeedSection.h"
#include "HCalibrator.h"
#include "Starter.h"
#include "LineTracer.h"
#include "CompositeSection.h"
#include "SectionCreate.h"
#include "BlockBingo.h"
#include "Timer.h"


class SceneWalker {
    public:
        SceneWalker(HCalibrator *cal,
                    Starter *start
                    );

        void run();

        void execUndefined();
        void execInit();
        void execCalibration();
        void execTracer();
        void execStarter();
        void execToBlock();
        void initToBingo();
        void execToBingo();
        void execWait();
        void execSearchColor();
        void execCarryBlock();
        void execExit();
        void execComposite();
        void execDebug();
        void execEnd();

        enum State {
            UNDEFINED,
            INIT,
            CALIBRATION,
            START,
            TRACE,
            INITTOBINGO,
            BINGO,
            WAIT,
            TOBLOCK,
            EXEC_TOBLOCK,
            SEARCHCOL,
            CARRYBLOCK,
            EXEC_CARRYBLOCK,
            TOPARKING,
            COMPOSITE,
            DEBUG,
            END
        };


        int getNextState();


    protected:
        HCalibrator *mCal;
        Tracer *mTracer;
        SpeedSection *mSpeedSection;

        Starter *mStarter;
        State mState;
        State mBState;

        Section *mSSection; //SpeedSection
        Section *mCSection; //CompositeSection
        SectionCreate *mSC;
        BlockBingo *mBlockBingo;
        Timer *mTimer;

        

    private:
    /*
        PParam  mParamSS[9] = {    //直進
            {25.0f  , 0    , 14      ,0  ,Method::RUN_LINE ,End::END_LEN },  //ライントレース
            {0.0d   , 0.0d          ,0      ,0  ,Method::RUN_RECORD   ,End::END_ALL },  //レコードカウント実施
            {15.0f  ,(double)COLOR::YELLOW  ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_COL },  //既定の色まで直進
          //  { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
           // { 15.0d  ,0.0d                  , 5.0      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
           // { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
           // { -12.0d  ,0.0d                   ,-3.0      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
           // { 12.0d  ,(double)COLOR::YELLOW  ,0.0      ,0  ,Method::RUN_STRAIGHT ,End::END_COL },  //直進停止
            { 15.0d  ,0.0d                   ,2.0      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
            { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
            { -15.0d  ,0.0d                   ,-0.5      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
            { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
            {0.0d   ,0.0d                   ,0      ,0  ,Method::RUN_END      ,End::END_UDF }   //終了処理
        };
        */

        PParam  mParamSS[20] = {    //直進
            {35.0f  , 0    , 20      ,0  ,Method::RUN_LINE ,End::END_LEN },  //ライントレース
            {0.0d   , 0.0d          ,0      ,0  ,Method::RUN_RECORD   ,End::END_ALL },  //レコードカウント実施
            {30.0f  ,(double)COLOR::YELLOW  ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_COL },  //既定の色まで直進
            { 30.0d  ,0.0d                   ,7.5      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
            { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
            { 0.0d  , 60.0d                 ,0      ,30 ,Method::RUN_TURN     ,End::END_ANG }, 
            { 0.0d  , 88.0d                 ,0      ,10 ,Method::RUN_TURN     ,End::END_ANG2 }, 
        //  {40.0d  , 88.0d            ,0   ,10 ,Method::RUN_VIRTUAL     ,End::END_ANG },  //ブロック無し90°右旋回⓵ /15
          {0      ,Turn::LEFT             ,0      ,0  ,Method::RUN_EDGE     ,End::END_ALL },  //ライントレース走行用エッジを左に変更
         {0.0d   , 0.0d          ,0      ,0  ,Method::RUN_RECORD   ,End::END_ALL },  //レコードカウント実施
             { -5.0d  ,0.0d                   ,-0.1      ,0  ,Method::RUN_STRAIGHT ,End::END_LEN },  //直進停止
            { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
            { 20.0d  ,0.0d                   ,13.0      ,0  ,Method::RUN_LINE ,End::END_LEN },  //直進停止
         {0.0d   , 0.0d          ,0      ,0  ,Method::RUN_RECORD   ,End::END_ALL },  //レコードカウント実施
            {20.0f  ,(double)COLOR::YELLOW  ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_COL },  //既定の色まで直進
            { 0.0d  ,0.0d                   ,0      ,0  ,Method::RUN_STRAIGHT ,End::END_ALL },  //直進停止
            {0.0d   ,0.0d                   ,0      ,0  ,Method::RUN_END      ,End::END_UDF }   //終了処理
        };


        
        int mDebA[30][50] = {
           // {34,33,32,39,46,999}
           // {34,33,32,25,18,17,16,23,30,29,28,21,14,999},   //4色経由ガレージイン
        // {34,33,32,25,18, 11,4,3,2,1,0,7,14,999},   //赤テスト
            // ロングテスト
            {34,33,32, 25,18,11,4, 3,2,1,0, 7,14,21,28,35,42, 43,44,45,46,47,48, 41,34, 33,32,25,18, 19,20,13,6, 5,4,3,2, 9,16,23,30,37,44, 43,42, 35,28,21,14 ,999},   //赤テスト
         //  {34,27,20,13,6, 5,4,3,2,1,0,7,14,999},   //上回り　駐車
       
          //              {28,29,30,23,16,9,2,1,0,7,14,999},   //赤サークル旋回テスト(L)
            {999}, // 単体テスト終端

            {34,33,32,999},   //即スロー右右45
            {34,33,32,999},   //即スロー右左45
            {34,33,32,999},   //即スロー右左135


            {34,27,20,13,6,5,4,3,2,1,0,999},   //
            {0,7,0,999},   //
            {0,1,8,15,999},   //
            {15,14,999},   //
            {14,999},   //
            {34,999},          //動作なし



        };

        int mDebN[20] = { -2,24,38,40,-1,-1,-1,22,-2,-1, 
                        };
        int deb_idx = 0;
};
#endif
