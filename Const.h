#ifndef __CONST_H__
#define __CONST_H__

/* 交点サークルの色のHの中心 */
#define H_RED_C 358.0
#define H_GREEN_C 128.0
#define H_BLUE_C 225.0
#define H_YELLOW_C 35.0
// RED 0.6
#define S_RED_C 0.5
#define S_GREEN_C 0.4
#define S_BLUE_C 0.5
#define S_YELLOW_C 0.3

/* ブロック色のHの中心 */
#define H_RED_B 3.0
#define H_GREEN_B 160.0
#define H_BLUE_B 220.0
#define H_YELLOW_B 33.0
#define S_RED_B 0.5
#define S_GREEN_B 0.1
#define S_BLUE_B 0.1
#define S_YELLOW_B 0.5


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
// 8100volt
#define CARRY_KP 7.4*1.013*0.96
#define CARRY_KI 2.1*1.013*0.96
#define CARRY_KD 1.32*1.013*0.96

// ブロックビンゴ基準前進値
//#define S_POW 22.0d
#define S_POW 25.0d

//充電池 8200用確定 
/*
#define SPEED_KD 3.5 
#define SPEED_KP 30.5*1.013
#define SPEED_KI 6.5*1.013
*/
/*
#define SPEED_KD 3.5 
#define SPEED_KP 30.5*1.013
#define SPEED_KI 6.5*1.013
*/

/*
#define SLOW_SPEED 42
#define SPEED_KD_SLOW 3.5*0.8 
#define SPEED_KP_SLOW 30.5*1.013*0.8
#define SPEED_KI_SLOW 6.5*1.013*0.5
*/

// RGB floatで再調整 
#define SLOW_SPEED 42
/*#define SPEED_KD_SLOW 3.5*0.75 
#define SPEED_KP_SLOW 30.5*1.013*0.4
#define SPEED_KI_SLOW 6.5*1.013*0.4*/
#define SPEED_KD_SLOW 2.625*0.9 
#define SPEED_KP_SLOW 12.3586
#define SPEED_KI_SLOW 2.6338

// 9000
/*
#define SPEED_KD 2.6 
#define SPEED_KP 20.5
#define SPEED_KI 6.5
*/
      // 8400
     /* double kd=3.8; 
      double kp=31.0;
      double ki=6.4;*/

      // ハイボルテージ 9000
     /*double kd2=3.2; 
      double kp2=30.0;
      double ki2=5.0;*/
      // 8500
/*
#define SPEED_KD2 1.9 
#define SPEED_KP2 19.0
#define SPEED_KD2 5.0
*/
/*
#define SPEED_KD2 3.0 
#define SPEED_KP2 30
#define SPEED_KI2 3.5
*/
/*
#define SPEED_KD2 1.5 
#define SPEED_KP2 15.0
#define SPEED_KI2 3.0
*/

//RGB float で再調整 ベーススピード50
#define SPEED_KD 3.5*1.03*0.8
//31
#define SPEED_KP 30.0*0.45
// *1.05
#define SPEED_KI 6.5*1.06*0.7


#define SPEED_KD2 3.5*0.96*0.9 
#define SPEED_KP2 30.5*1.013*0.96*0.9
#define SPEED_KI2 6.5*1.013*0.96*0.9


//#define BASE_VOLT 8200
//#define BASE_VOLT 8300
#define BASE_VOLT 9000
//SimpleWalker
#define ADJUST_BATTERY true
//LineTracer
#define ADJUST_BATTERY2 false

#endif