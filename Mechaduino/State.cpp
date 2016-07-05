#include "State.h"


// @TODO Put into a struct.. not 100% on volatile semantics with that.
volatile float pKp = 30.75;
volatile float pKi = 0.50;
volatile float pKd = 5.00;

volatile float vKp = 0.05;
volatile float vKi = 200.00 * Ts;
volatile float vKd = 0.00 / Ts;

//interrupt vars

volatile float ei = 0.0;
volatile int U = 0;  //control effort (abs)
volatile float r = 0.0;  //setpoint
volatile float y = 0.0;  // measured angle
volatile float yw = 0.0;
volatile float yw_1 = 0.0;
volatile float e = 0.0;  // e = r-y (error)
volatile float p = 0.0;  // proportional effort
volatile float i = 0.0;  // integral effort
volatile float PA = 1.8;  //

volatile float u = 0.0;  //real control effort (not abs)
volatile float u_1 = 0.0;
volatile float e_1 = 0.0;
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;

volatile long wrap_count = 0;
volatile float y_1 = 0;

volatile float ITerm;

volatile Mode mode;