#include <USB/USBAPI.h>
#include "lookup/Parameters.h"
#include "lookup/Lookup.h"

float lookup_angle(int n) {
    return pgm_read_float_near(Parameters::lookup + n);
}

float lookup_sine(int m) {
    // @TODO Suspcicious
    m = (0.01 * (((m % 62832) + 62832) % 62832)) + 0.5; //+0.5 for rounding

    //SerialUSB.println(m);

    if (m > 314) {
        m = m - 314;
        return -pgm_read_float_near(Lookup::sine_lookup + m);
    } else {
        return pgm_read_float_near(Lookup::sine_lookup + m);
    }
}

void waitSerialUSB() {
    while (SerialUSB.available() == 0)  {
    }
}

const int mod(int xMod, int mMod) {
    return (xMod % mMod + mMod) % mMod;
}