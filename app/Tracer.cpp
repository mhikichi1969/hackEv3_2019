#include "Tracer.h"

Tracer::Tracer(HBTtask *bt,HPolling *poller,Odometry *odo,SDFile *sd):
  leftWheel(PORT_C), rightWheel(PORT_B),
  colorSensor(PORT_2),
  mBt(bt),
  mPoller(poller),
  mOdo(odo),
  mSDFile(sd) {

    mPID = new HPID(0.004);
    mSDFile->open("/ev3rt/apps/Lbase.csv");

}

void Tracer::init() {
  init_f("Tracer");
}

void Tracer::terminate() {
  msg_f("Stopped.", 1);
  leftWheel.stop();
  rightWheel.stop();
}

// tag::tracer_calc_prop[]
float Tracer::calc_prop_value() {
  //const float Kp = 1.4;        // <1>

  //int diff = colorSensor.getBrightness() - target; // <3>
  //int diff = mPoller->getBrightness() - target;

 double len= mOdo->getLength();
 mSDFile->current_read(len*10);
 mSDFile->next_read(len*10+100);

/// fwd=70用
/*
  mPID->setTarget(0);
  mPID->setKp(35*0.65);
  mPID->setKi(30);
  mPID->setKd(2.46);
*/
  mPID->setTarget(0);
  mPID->setKp(8.5);
  mPID->setKi(21);
  mPID->setKd(0.699);

// 0616 ベスト
/*
  mPID->setKp(30*0.99);
  mPID->setKi(80*0.2);
  mPID->setKd(2.07*0.95);
*/
  double diff = mPoller->getBrightnessRate();

  //return (Kp * diff + bias);                       // <4>

  return -mPID->getOperation(diff);
}
// end::tracer_calc_prop[]
// tag::tracer_run[]
void Tracer::run() {
 // msg_f("running...", 1);
  int turn = (int)calc_prop_value(); // <1>
  int fwd=70;
 //int fwd = mBt->fwd;
 //int turn = mBt->turn;

 
  
  double pwm_l = fwd + turn;      // <2>
  double pwm_r = fwd - turn;      // <2>


  //if(turn!=0) ev3_speaker_play_tone(NOTE_E4,100);
  pwm_l = pwm_l*8220.0/mPoller->getVoltage();
  pwm_r = pwm_r*8220.0/mPoller->getVoltage();
   static const int MAXPWM=80;

           if(pwm_l>MAXPWM) {
                pwm_r = (int)((float)MAXPWM*pwm_r/pwm_l);
                pwm_l=MAXPWM;
            }
            if(pwm_l<-MAXPWM) {
                pwm_r = (int)((float)-MAXPWM*pwm_r/pwm_l);
                pwm_l=-MAXPWM;
            }

            if(pwm_r>MAXPWM) {
                pwm_l = (int)((float)MAXPWM*pwm_l/pwm_r);
                pwm_r=MAXPWM;
            }
            if(pwm_r<-MAXPWM) {
                pwm_l = (int)((float)-MAXPWM*pwm_l/pwm_r);
                pwm_r=-MAXPWM;
            }

            if(pwm_r>100) pwm_r=100;
            if(pwm_l>100) pwm_l=100;
            if(pwm_r<-100) pwm_r=-100;
            if(pwm_l<-100) pwm_l=-100;

  leftWheel.setPWM((int)pwm_l);
  rightWheel.setPWM((int)pwm_r);
//leftWheel.setPWM((int)0);
//rightWheel.setPWM((int)0);

}
// end::tracer_run[]
