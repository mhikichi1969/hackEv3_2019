#ifndef HBTtask_
#define HBTtask_

#include "ev3api.h"

struct T_SENDBUF{
	int32_t  cnt;
	int32_t  motor_l;
	int32_t  motor_r;
	int32_t  motor_arm;
	
	//colorid_t  color;
	//rgb_raw_t rgb;
	uint16_t  r;
	uint16_t  g;
	uint16_t  b;
	
	uint16_t  light;
	float light_rate;
	int16_t  gyro;
	float  angle;
	int16_t  sonar;
	int16_t volt;
	int16_t dummy;
	float x;
	float y;
	float th;
	float len;
	float v;
};

#define BUF_SIZE 1000

class HBTtask {
    public:
		HBTtask();
		~HBTtask();
//void reciev();
        void send();
		void reciev();
		int next(int index);
		void init_queue();
		void enqueue(T_SENDBUF x);
		T_SENDBUF dequeue();
		int empty();
		int getBTData();
		void resetBTData();
		int getRed();
		int getGreen();
		int getBlue();
		int getYellow();
		int getBlack();
		int getBonus();


		int fwd=0;
		int turn=0;
		int arm=0;

    private:
		FILE  *bt = NULL;
		T_SENDBUF SENDBUF[BUF_SIZE];
		int front;
		int rear;

		int buffer;

		int r_num=0;
		int g_num=0;
		int b_num=0;
		int y_num=0;
		int k_num=0;
		int bonus_num=0;

		bool send_stop=false;

};

#endif
