#ifndef ODOMETRY_
#define ODOMETRY_

#include <math.h>
#include "util.h"


#define VELOCITY_CNT 20
class Odometry
{
public:
	Odometry(int32_t rs1,int32_t rs2,float dt);
	void getOdometry(float& x,float& y,float& th);
	void calc(int32_t rs1,int32_t rs2);
	void reset();
	void resetLength();
	void resetAngle();

	float getLength();
	double getAngle();
	double getAngleDeg();

	double getX();
	double getY();
	double getGyroX();
	double getGyroY();
	
	void recordCount();
	float lengthFromRecordPoint(int32_t rs1,int32_t rs2);
	float angleFromRecordPoint(int32_t rs1,int32_t rs2);
	void resetWheelCount();
	float getVelocityDiff(float curve);
	float getWheelCountDiff(float curve);
	float getWheelCountDiffFromRecord(float curve);
	float getWheelCountDiffFromRecordReverse(float curve);
	float getWheelCountDiffFromVelocity(float curve);

	void setAngle(float angle);
	bool isStop();
	void setGyroAngle(double angle);
	double getGyroAngle();
	
	double getAccel();
	double getVelocity();

	void backlash_cancel( int32_t *lenc, int32_t *renc);
	void setPwm(int left,int right);

	static const double RATE_RtoL;


private:
	double x;
	double y;

	double x_g;
	double y_g;
	double th;
	double rot;
	double dt;

	double th_g;
	double th_g_prev;
	double deg;
	double deg_g;
	
	double sumx;
	double sumy;
	double sumth;
	double sumlen;

	double r1_v;
	double r2_v;
	double v;
	double prev_v;
	double a;
	
	int32_t prev_rs1;
	int32_t prev_rs2;

	double record_len1;
	double record_len2;
	

	double record_rs1;
	double record_rs2;

	int32_t rs1_array[VELOCITY_CNT];
	int32_t rs2_array[VELOCITY_CNT];
	double v_array_rs1[VELOCITY_CNT];
	double v_array_rs2[VELOCITY_CNT];
	double v_array[VELOCITY_CNT];
	double a_array[VELOCITY_CNT];

	int stop_cnt=0;

	int l_pwm=0;
	int r_pwm=0;

};

#endif
