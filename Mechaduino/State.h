#ifndef MECHADUINO_STATE_H
#define MECHADUINO_STATE_H

#include <USB/USBAPI.h>
#include "lookup/Parameters.h"


const float Ts = 0.0003333333;

// @TODO Put all the externs into a struct.. but not 100% on volatile semantics with that.

//----Current Parameters-----

extern volatile float pKp;
extern volatile float pKi;
extern volatile float pKd;

extern volatile float vKp;
extern volatile float vKi;
extern volatile float vKd;

//interrupt vars

enum Mode {
    Position,
    Velocity,
    Torque,
    Custom
};

extern volatile float ei;
extern volatile int U;  //control effort (abs)
extern volatile float r;  //setpoint
extern volatile float y;  // measured angle
extern volatile float yw;
extern volatile float yw_1;
extern volatile float e;  // e = r-y (error)
extern volatile float p;  // proportional effort
extern volatile float i;  // integral effort
extern volatile float PA;

extern volatile float u;  //real control effort (not abs)
extern volatile float u_1;
extern volatile float e_1;
extern volatile float u_2;
extern volatile float e_2;
extern volatile float u_3;
extern volatile float e_3;
extern volatile long counter;

extern volatile long wrap_count;
extern volatile float y_1;

extern volatile float ITerm;

extern volatile Mode mode;

extern int dir;		//initialize stepping mode variables
extern int stepNumber; // step index for cal routine

extern int a;  // raw encoder value in closed loop and print_angle routine (should fix the latter to include LUT)


#endif //MECHADUINO_STATE_H

