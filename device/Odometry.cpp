#include "Odometry.h"

const double D_RIGHT=10.00;
const double D_LEFT=10.0;
// 13.26?
//const float TREAD=13.2;
const double TREAD=14.25;

const double HOSEI=1.0; //MS09

//const float HOSEI=1.0003f;

const double Odometry::RATE_RtoL=0.00f;   // ＋右に補正

Odometry::Odometry(int32_t rs1,int32_t rs2,float dt)
{
	x=y=th=0.0;
	x_g=y_g=th_g=0.0;
	sumlen=0;
	prev_rs1=rs1;
	prev_rs2=rs2;
	this->dt=dt;
}

void Odometry::getOdometry(float& x,float& y,float& th)
{
	x=this->x;
	y=this->y;
	th=this->th*180.0/M_PI;
}

float Odometry::getLength() 
{
	return sumlen;
}
double Odometry::getAngle()
{
	return th;
}
double Odometry::getAngleDeg()
{
	return th*180/M_PI;
}


double Odometry::getGyroAngle()
{
	return deg_g;
//	return th*180/M_PI;
}

double Odometry::getX()
{
	return x;
}
double Odometry::getY()
{
	return y;
}
double Odometry::getGyroX()
{
	return x_g;
}
double Odometry::getGyroY()
{
	return y_g;
}


void Odometry::reset() 
{
	resetLength();
	resetAngle();

}
void Odometry::resetLength()
{
	sumlen=0.0;	
}
void Odometry::resetAngle()
{
	th=th_g=0.0;
	x=y=0;
	x_g=y_g=0.0;
	deg = 0.0;
}

void Odometry::setGyroAngle(double angle)
{
//	static double prev_angle=0;
//	deg += (angle+prev_angle)/2.0*0.004;
//	prev_angle = angle;

	th_g_prev = th_g;
	th_g = angle*M_PI/180; // deg -> rad
	deg_g = angle;
}

void Odometry::recordCount()
{

	ev3_speaker_play_tone(NOTE_F4,20);

	record_rs1 = prev_rs1;
	record_rs2 = prev_rs2;

	record_len1=0;
	record_len2=0;
}

void Odometry::calc(int32_t rs1,int32_t rs2)
{

	char buf[256];

	static int count=0;

	double drs1=rs1-prev_rs1;
	double drs2=(rs2-prev_rs2)*HOSEI;
	double len_l = drs1*M_PI*D_LEFT/360.0;
	double len_r = drs2*M_PI*D_RIGHT/360.0;
	double dth=(len_l-len_r)/TREAD;
	
	x+= (len_r+len_l)/2.0*cos(th+dth/2.0); //進行方向
	y+= (len_r+len_l)/2.0*sin(th+dth/2.0); //横
	x_g+= (len_r+len_l)/2.0*cos(th_g);
	y_g+= (len_r+len_l)/2.0*sin(th_g);

	th+=dth;
	sumlen += (len_r+len_l)/2.0;

	if(prev_rs1==rs1 && prev_rs2==rs2) {
		stop_cnt++;
	}
	else 
		stop_cnt=0;

	//v = (len_l+len_r)/(2.0*0.004); // 瞬間
	//v = 0.5*v + prev_v*0.5;

	prev_rs1=rs1;
	prev_rs2=rs2;

	record_len1 += len_l;
	record_len2 += len_r;

	rs1_array[count]=rs1;
	rs2_array[count]=rs2;

	r1_v = rs1_array[count] - rs1_array[(count+1)%VELOCITY_CNT];
	r2_v = rs2_array[count] - rs2_array[(count+1)%VELOCITY_CNT];
	v_array_rs1[count] = M_PI*D_LEFT*r1_v/(VELOCITY_CNT*0.004*360);
	v_array_rs2[count] = M_PI*D_RIGHT*r2_v/(VELOCITY_CNT*0.004*360);
	v=(v_array_rs1[count]+v_array_rs2[count])/2.0f;
	
	prev_v = v;


	v_array[count] = v;

	static float tmp=0;
	/*if( count>0)
		a_array[count] = (v_array[count-1] - v_array[count])/0.004;
	else 
		a_array[count] = (v_array[VELOCITY_CNT-1] - v_array[0])/0.004;*/
	a_array[count] = (v_array[count] - v_array[(count+1)%VELOCITY_CNT])/(VELOCITY_CNT*0.004);
	a = a_array[count] ;

	if(tmp<a_array[count])
		tmp = a_array[count];

//	sprintf(buf,"v:%4.2f,a:%4.2f,%5d,%5d",v_array_rs1[count],a_array[count],rs1_array[count],rs1_array[(count+1)%VELOCITY_CNT]);
//	msg_f(buf,8);
	//sprintf(buf,"v:%4.2f,a:%4.2f",v,a_array[count]);
	//msg_f(buf,9);

	if( ++count ==VELOCITY_CNT) {
		count=0;
	}
	

/*	
	sprintf(buf,"lCnt:%.2f,rCnt:%.2f,diff:%.1f",prev_rs1,prev_rs2,prev_rs1-prev_rs2);
	msg_f(buf,6);
	sprintf(buf,"l_len:%.2f,r_len:%.2f,diff:%.2f",record_len1,record_len2,record_len1-record_len2);
	msg_f(buf,7);*/

//	sprintf(buf,"x:%f,y:%f",this->x,this->y);
//	msg_f(buf,5);
	//sprintf(buf,"rs1:%d,rs2:%d",rs1,rs2);
/*	sprintf(buf,"len:%lf,th:%f",this->sumlen,this->th);
	msg_f(buf,6);
			
	sprintf(buf,"rs1:%ld,rs2:%ld,diff:%ld",rs1,rs2,rs1-rs2);
	msg_f(buf,7);
	*/
	//sprintf(buf,"r1_v:%100.0lf,r2_v:%100.0lf  v:%100.2lf, a:%100.2lf",r1_v,r2_v,v,a);
	//sprintf(buf,"v:%.2lf, a:%.2lf",v,a);
	//msg_f(buf,8);


	

}

float Odometry::getVelocityDiff(float curve)
{
	return r1_v-r2_v*(curve+RATE_RtoL);		
}

float Odometry::getWheelCountDiff(float curve)
{
	return prev_rs1-prev_rs2*(curve+RATE_RtoL);
}
float Odometry::getWheelCountDiffFromRecord(float curve)
{

	// rs1 左モータ　rs2 右モータ
	float ret = (prev_rs1-record_rs1)-(prev_rs2-record_rs2)*curve*HOSEI;
	if( ret>100 ) ret = 100;
	if( ret<-100 ) ret = -100;

	return ret;
//	return (float)(prev_rs1-record_rs1)/(prev_rs2-record_rs2);
	//return record_len1-record_len2*curve;
}

float Odometry::getWheelCountDiffFromRecordReverse(float curve)
{
	// rs1 左モータ　rs2 右モータ
	float ret = (prev_rs1-record_rs1)*curve*HOSEI-(prev_rs2-record_rs2);
	if( ret>100 ) ret = 100;
	if( ret<-100 ) ret = -100;

	return ret;
//	return (float)(prev_rs1-record_rs1)/(prev_rs2-record_rs2);
	//return record_len1-record_len2*curve;
}

float Odometry::getWheelCountDiffFromVelocity(float curve)
{
	// rs1 左モータ　rs2 右モータ
	float ret = r1_v*curve*HOSEI-r2_v;
	if( ret>100 ) ret = 100;
	if( ret<-100 ) ret = -100;

	return ret;

}


float Odometry::lengthFromRecordPoint(int32_t rs1,int32_t rs2)
{
		int32_t drs1=rs1-record_rs1;
		int32_t drs2=rs2-record_rs2;
		float len_r = drs1*M_PI*D_RIGHT/360;
		float len_l = drs2*M_PI*D_LEFT/360;

		return (len_r+len_l)/2.0f;
}

float Odometry::angleFromRecordPoint(int32_t rs1,int32_t rs2)
{
		int32_t drs1=rs1-record_rs1;
		int32_t drs2=rs2-record_rs2;
		float len_r = drs1*M_PI*D_RIGHT/360;
		float len_l = drs2*M_PI*D_LEFT/360;

		return (len_r-len_l)/TREAD;

}


void Odometry::resetWheelCount()
{
	reset();
	prev_rs1=0;
	prev_rs2=0;

}

void Odometry::setAngle(float angle)
{
	th = angle;
}

bool Odometry::isStop()
{
	return stop_cnt>500;
}

double Odometry::getVelocity()
{
	return v;
}
double Odometry::getAccel()
{
	return a;
}




//*****************************************************************************
// 関数名 : backlash_cancel
// 引数 : lpwm (左モーターPWM値 ※前回の出力値)
//        rpwm (右モーターPWM値 ※前回の出力値)
//        lenc (左モーターエンコーダー値)
//        renc (右モーターエンコーダー値)
// 返り値 : なし
// 概要 : 直近のPWM値に応じてエンコーダー値にバックラッシュ分の値を追加します。
//*****************************************************************************
void Odometry::backlash_cancel( int32_t *lenc, int32_t *renc)
{
    const int BACKLASHHALF = 4;   // バックラッシュの半分[deg]

    if(l_pwm < 0) *lenc += BACKLASHHALF;
    else if(l_pwm > 0) *lenc -= BACKLASHHALF;

    if(r_pwm < 0) *renc += BACKLASHHALF;
    else if(r_pwm > 0) *renc -= BACKLASHHALF;
}

void Odometry::setPwm(int left,int right)
{
	l_pwm = left;
	r_pwm=right;
}