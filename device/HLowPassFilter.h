#ifndef H_LOW_PASS_FILTER_
#define H_LOW_PASS_FILTER_

class HLowPassFilter {
    public:
        HLowPassFilter(int);
        ~HLowPassFilter();
        int getAvgNum();
        void addValue(int);
        float getFillteredValue();
        void setRate(double rate);

    private:
        int avg_num;
        int current;
        int *log;  
        double prev; 

        double mRate;
};

#endif
