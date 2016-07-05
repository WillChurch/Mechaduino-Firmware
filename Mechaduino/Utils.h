#ifndef MECHADUINO_UTILS_H
#define MECHADUINO_UTILS_H

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

float lookup_angle(int n);

float lookup_sine(int m);

void waitSerialUSB();

const int mod(int xMod, int mMod);

int readEncoder();

#endif //MECHADUINO_UTILS_H
