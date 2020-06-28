#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"


#define MAIN_PRIORITY    TMIN_APP_TPRI + 1
#define TRACER_PRIORITY  TMIN_APP_TPRI + 3
#define ARM_PRIORITY  TMIN_APP_TPRI + 2
#define BT_SEND_PRIORITY  TMIN_APP_TPRI + 7
#define BT_PRIORITY  TMIN_APP_TPRI + 6
#define DEVICE_ERROR_PRIORITY  TMIN_APP_TPRI + 4

#ifndef STACK_SIZE
#define STACK_SIZE      65536
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void tracer_cyc(intptr_t exinf);
//extern void bt_task(intptr_t unused);
//extern void bt_send_task(intptr_t unused);
extern void arm_task(intptr_t unused);
//extern void device_error_task(intptr_t unused);
//extern void ev3_cyc_btsend(intptr_t exinf);
//extern void ev3_cyc_bt(intptr_t exinf);
extern void ev3_cyc_arm(intptr_t exinf);
//extern void ev3_cyc_device_error(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
