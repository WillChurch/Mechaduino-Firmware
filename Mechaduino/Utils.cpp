#include <USB/USBAPI.h>
#include <wiring_digital.h>
#include <Arduino.h>
#include <SPI.h>
#include "constants/Parameters.h"
#include "constants/Lookup.h"
#include "constants/Pins.h"

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

int readEncoder() {
    long angleTemp;
    digitalWrite(CHIP_SELECT, LOW);

    //angle = SPI.transfer(0xFF);
    byte b1 = SPI.transfer(0xFF);
    byte b2 = SPI.transfer(0xFF);


    angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);
    //  SerialUSB.println((angle & 0B0011111111111111)*0.02197265625);

    digitalWrite(CHIP_SELECT, HIGH);
    return angleTemp;
}