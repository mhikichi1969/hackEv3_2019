#ifndef __CONST_H__
#define __CONST_H__

#define RUNNER_NO 0 // 走行体番号 0:MS-08 1:MS-18

//Bluetoothのデータを走行体から送信するか
#define BTSEND false
//角度判定にジャイロを使うか
#define GYRO true

/* 交点サークルの色のHの中心 */
#define H_RED_C 358.0
#define H_GREEN_C 105.0   //128
#define H_BLUE_C 225.0
#define H_YELLOW_C 35.0
// RED 0.6
#define S_RED_C 0.7*0.4
#define S_GREEN_C 0.6*0.3
#define S_BLUE_C 0.91*0.4
#define S_YELLOW_C 0.8*0.4

/* ブロック色のHの中心 */
#define H_RED_B 3.0
#define H_GREEN_B 160.0
#define H_BLUE_B 220.0
#define H_YELLOW_B 33.0
#define S_RED_B 0.15
#define S_GREEN_B 0.15
#define S_BLUE_B 0.15
#define S_YELLOW_B 0.15


#define LOWPASS 0.6

/* ブロック運搬用PID */
/*
#define CARRY_KP 8.5
#define CARRY_KI 2.0
#define CARRY_KD 2.1
*/
/*
#define CARRY_KP 7.3
#define CARRY_KI 2.05
#define CARRY_KD 1.31
*/
// 8100volt 地区大会
/*
#define CARRY_KP 7.4*1.013*0.96
#define CARRY_KI 2.1*1.013*0.96
#define CARRY_KD 1.32*1.013*0.96
*/
// ブロックビンゴ基準前進値
//#define S_POW 22.0d
#define S_POW 32.0d
#define S_POW_THROW 28.0d


// RGB floatで再調整 
#define SLOW_SPEED 42
/*#define SPEED_KD_SLOW 3.5*0.75 
#define SPEED_KP_SLOW 30.5*1.013*0.4
#define SPEED_KI_SLOW 6.5*1.013*0.4*/
#define SPEED_KD_SLOW 2.625*0.9 
#define SPEED_KP_SLOW 12.3586
#define SPEED_KI_SLOW 2.6338


//RGB float で再調整 ベーススピード50 地区大会final
/*
#define SPEED_KD 3.5*1.03*0.8
//31
#define SPEED_KP 30.0*0.45
// *1.05
#define SPEED_KI 6.5*1.06*0.7
*/
/*#define SPEED_KD 2.884*1.06*1.0812
#define SPEED_KP 13.5*1.06*1.0812
#define SPEED_KI 4.823*1.0812*/
#if RUNNER_NO==0
#define SPEED_KP 13.5*1.13
#define SPEED_KI 4.8*1.1
#define SPEED_KD 2.7*0.923  //0.925

/* 10/21
#define CARRY_KP 7.4*1.013*0.88   //0.96
#define CARRY_KI 2.1*1.013*0.25   //0.6 
#define CARRY_KD 1.32*1.013*1.163 //1.163
*/
#define CARRY_KP 7.4*1.013*0.974   //0.97 0.98 0.96
#define CARRY_KI 2.1*1.013*0.05   //0.001 0.36 0.6 
#define CARRY_KD 1.32*1.013*1.1345 //1.13 1.07 1.163

#elif RUNNER_NO==1
#define SPEED_KD 2.7*1.06
#define SPEED_KP 13.5*1.16
#define SPEED_KI 4.8*1.16

#define CARRY_KP 7.4*1.013*1.1
#define CARRY_KI 2.1*1.013*1.0
#define CARRY_KD 1.32*1.013*1.07


#endif


#define SPEED_KD2 3.5*0.96*0.9 
#define SPEED_KP2 30.5*1.013*0.96*0.9
#define SPEED_KI2 6.5*1.013*0.96*0.9


//#define BASE_VOLT 8200
//#define BASE_VOLT 8300
#define BASE_VOLT 9000
//SimpleWalker
#define ADJUST_BATTERY true
//LineTracer
#define ADJUST_BATTERY2 true
#define ADJUST_PARAM 0.875




//ショートカットコース用パラメータ
// Lコース ロゴ往復バックパターン用
#define BACK_ANGLE 18-180
#define BACK_GATE2_ANNGEL 208-180
#define BACK_LAST_ANGLE -181

// Rコース
#if RUNNER_NO==0
#define S1_ANGLE 
#define S2_ANGLE
#define SOUT_ANGLE -78
#define BACKSTRAIGHT_ANGLE -177
#define BACKSTRAIT_OUT_ANGLE
#define LAST_CURVEIN_ANGLE -180-118
#define LAST_CURVEOUT_ANGLE -180+90
#elif RUNNER_NO==1
#define SOUT_ANGLE -80
#define BACKSTRAIGHT_ANGLE -170
#define LAST_CURVEIN_ANGLE -180-115
#define LAST_CURVEOUT_ANGLE -180+90
#endif

//タイムアウト関連
//#define TIMEOUT 120000
#define TIMEOUT 240000

#define R_GARAGE 5500
#define L_GARAGE 3500
#endif