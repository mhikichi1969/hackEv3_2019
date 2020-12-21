#include "app.h"


// using宣言

#include "ColorSensor.h"
#include "GyroSensor.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"

#include "Tracer.h"
#include "ArmControl.h"

#include "HColorSensor.h"

#include "HBTtask.h"
#include "HPolling.h"
#include "Odometry.h"
#include "HCalibrator.h"
#include "SceneWalker.h"
#include "Starter.h"
#include "SDFile.h"
#include "LineTracer.h"
#include "SimpleWalker.h"
//#include "SpeedSection.h"
#include "SectionJudge.h"

#include "Turn.h"
#include "StraightWalker.h"
#include "SpeedControl.h"
#include "DeviceError.h"
#include "VirtualTracer.h"

//#include "util.h"
using namespace ev3api;

//ColorSensor gColorSensor(PORT_2);
HColorSensor gColorSensor(PORT_2);

//GyroSensor  gGyroSensor(PORT_4);
GyroSensor  *gGyroSensor=nullptr;
TouchSensor gTouchSensor(PORT_1);
SonarSensor gSonarSensor(PORT_3);
Motor       gLeftWheel(PORT_C,false,LARGE_MOTOR);
Motor       gRightWheel(PORT_B,false,LARGE_MOTOR);
Motor       gArm(PORT_A,true,LARGE_MOTOR);
Clock       gClock;

//gGyroSensor = new gGyroSensor(PORT_4);
//Tracer tracer;
HBTtask *gBTcomm;
HPolling *gHPolling;
DeviceError *gDeviceError;
Odometry *gOdo;
SimpleWalker *gSimpleWalker;
Tracer *gTracer;
LineTracer *gLineTracer;

ArmControl *gArmControl;
HCalibrator *gCalibrator;
SceneWalker *gSceneWalker;
Starter *gStarter;
SDFile * gSDFile;
//SpeedSection *gSpeed;
Judge *gJudge;
SectionJudge *gSectionJudge;

Turn *gTurn;
StraightWalker *gStraightWalker;
SpeedControl *gSpeedControl;
VirtualTracer *gVirtualTracer;

static void user_system_create() {
  // [TODO] タッチセンサの初期化に2msのdelayがあるため、ここで待つ
  tslp_tsk(2);
  //tracer.init();
  
  gBTcomm = new HBTtask();
  gOdo = new Odometry(0,0,0.004);
  gHPolling = new HPolling(gBTcomm, gOdo, gColorSensor,gGyroSensor,gTouchSensor,gSonarSensor,gLeftWheel,gRightWheel,gArm,gClock);
  gHPolling->newGyro();

  gDeviceError = new DeviceError(gHPolling);
  gSDFile = new SDFile();
  gTracer = new Tracer(gBTcomm,gHPolling,gOdo,gSDFile);
 gArmControl = new ArmControl(gArm,gBTcomm);
  gCalibrator = new HCalibrator(gHPolling,gArmControl);
  gStarter = new Starter(gTouchSensor);
  /*gSpeed = new SpeedWalker(gHPolling, 
                        gOdo,
                        gLineTracer,
                        gSDFile);*/
  gSceneWalker = new SceneWalker(gCalibrator,gStarter);
  gJudge = new Judge(gHPolling,gOdo);
  gSectionJudge = new SectionJudge(gHPolling,gOdo);
  gSpeedControl = new SpeedControl(gOdo);

  gSimpleWalker = new SimpleWalker(gLeftWheel,gRightWheel,gOdo,gSpeedControl);
  gLineTracer = new LineTracer(gLeftWheel,gRightWheel,gHPolling,gOdo,gSpeedControl);

  gTurn = new Turn(gLeftWheel,gRightWheel,gOdo,gSpeedControl);
  gStraightWalker = new StraightWalker(gLeftWheel,gRightWheel,gOdo,gSpeedControl);
  gVirtualTracer = new VirtualTracer(gLeftWheel,gRightWheel,gHPolling,gOdo,gSpeedControl);

  ev3_led_set_color(LED_ORANGE);
}

/**
 * EV3システム破棄
 */
static void user_system_destroy() {
    gLeftWheel.reset();
    gRightWheel.reset();
    gArm.reset();
    
    delete gTracer;
    delete gHPolling;
    delete gOdo;   
    delete gBTcomm;
    delete gDeviceError;
    delete gArmControl;
    delete gSceneWalker;


}



void tracer_cyc(intptr_t exinf) {
    act_tsk(TRACER_TASK);
}

void tracer_task(intptr_t exinf) {
  if (ev3_button_is_pressed(BACK_BUTTON)) {
    wup_tsk(MAIN_TASK);  // 左ボタン押下でメインを起こす
  } else {
    gHPolling->run();

    gSceneWalker->run(); //走行
  }
  ext_tsk();
}

void ev3_cyc_btsend(intptr_t exinf) {
  act_tsk(BT_SEND_TASK);
}
void ev3_cyc_bt(intptr_t exinf) {
  act_tsk(BT_TASK);
}
void ev3_cyc_arm(intptr_t exinf) {
  act_tsk(ARM_TASK);
}

void ev3_cyc_device_error(intptr_t exinf) {
    act_tsk(DEVICE_ERR_TASK);
}

void bt_task(intptr_t unused) {
    gBTcomm->reciev();
    ext_tsk();
}

void bt_send_task(intptr_t unused) {
    gBTcomm->send();
    ext_tsk();
}

void arm_task(intptr_t unused) {
    gArmControl->run();
    ext_tsk();    
}
void  device_error_task(intptr_t unused) {
    gDeviceError->run();
    ext_tsk();    
}


void main_task(intptr_t unused) {
  user_system_create();  // センサやモータの初期化処理
  

  sta_cyc(TRACER_CYC);
  //ev3_sta_cyc(EV3_CYC_BTSEND);
  sta_cyc(EV3_CYC_ARM);
  act_tsk(BT_TASK);
  //ev3_sta_cyc(EV3_CYC_BT);
  //sta_cyc(EV3_CYC_DEVICE_ERROR);

  slp_tsk();  // 起きたら、走行をやめる
  stp_cyc(TRACER_CYC);
  //ev3_stp_cyc(EV3_CYC_BTSEND);
  stp_cyc(EV3_CYC_ARM);
  //stp_cyc(EV3_CYC_DEVICE_ERROR);
  ter_tsk(BT_TASK);

 // ev3_stp_cyc(EV3_CYC_BT);
 // tracer.terminate();
  user_system_destroy();

  ext_tsk();
}
