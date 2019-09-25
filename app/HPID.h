#ifndef HPID_
#define HPID_

class HPID {
    public:
        HPID();
        HPID(double delta);
        ~HPID();
        void setLimit(double);
        void setTarget(double);
        double getOperation(double value);
        double getOperation(double value,double limit);

        void setKp(double kp);
        void setKi(double ki);
        void setKd(double kd);

        double getDiff();
        double getIntegral();

        void resetParam();
        double getTarget();

        void setDeltaT(double delta);

        bool debug = false;
    private:
       double diff[20];
       double old[5000];
       int old_cnt=0;
       int old_max=1000;
       double integral;
       double target;
       double limit; 
       double delta;
       double Kp;
       double Ki;
       double Kd;
       double DELTAT;

       double tgt_limit;
};

#endif
