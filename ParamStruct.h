#ifndef _PARAM_STRUCT_
#define _PARAM_STRUCT_

#include "Flag.h"

//class ParamStruct   //
//{
    //public:
        typedef struct _PARAM_S {
            double fwd;
            double target;
            double kp;
            double ki;
            double kd;
            double curve;
            double ckp;

            double len;
        } SParam;

        typedef struct _PARAM_P {
            double fwd;
            double target;
            double len;
            double turn;
            Flag::Method runFlag;
            Flag::End endFlag;
        } PParam;


 //   private:

//};

#endif
