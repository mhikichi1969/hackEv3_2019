#ifndef _TRACER_H_
#define _TRACER_H_

#include "Motor.h"
#include "ColorSensor.h"
#include "util.h"

#include "HBTtask.h"
#include "HPolling.h"
#include "HPID.h"
#include "SDFile.h"
#include "Odometry.h"

using namespace ev3api;

class Tracer {
public:
  Tracer(HBTtask *bt, HPolling *poller,Odometry *odd,SDFile *sd);
  void run();
  void init();
  void terminate();

// tag::tracer_h_private[]
private:
  Motor leftWheel;
  Motor rightWheel;
  ColorSensor colorSensor;
  const int8_t mThreshold = 20;
  const int8_t pwm = (Motor::PWM_MAX) / 6;

  float calc_prop_value();      // <1>

  HBTtask *mBt;
  HPolling *mPoller;
  Odometry *mOdo;
  SDFile *mSDFile;
  HPID *mPID;
};
// end::tracer_h_private[]
#endif