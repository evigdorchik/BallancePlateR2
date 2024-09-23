#ifndef _TSC2007_h
#define _TSC2007_h

#include "Arduino.h"
#include <Wire.h>
#include <Servo.h>

#define DEV_ADDR 0x48

typedef enum {
   MEASURE_TEMP0 = 0,
   MEASURE_AUX = 2,
   MEASURE_TEMP1 = 4,
   ACTIVATE_X = 8,
   ACTIVATE_Y = 9,
   ACTIVATE_YPLUS_X = 10,
   SETUP_COMMAND = 11,
   MEASURE_X = 12,
   MEASURE_Y = 13,
   MEASURE_Z1 = 14,
   MEASURE_Z2 = 15
} tsc2007_function;

typedef enum {
   POWERDOWN_IRQON = 0,
   ADON_IRQOFF = 1,
   ADOFF_IRQON = 2,
} tsc2007_power;

typedef enum {
   ADC_12BIT = 0,
   ADC_8BIT = 1,
} tsc2007_resolution;

class TSC2007 {
   public:
      TSC2007();
      ~TSC2007();
      // byte MeasureTemp0();
      // byte MeasureAux();
      // byte MeasureTemp1();
      // byte ActivateX_drivers();
      // byte ActivateY_drivers();
      // byte ActivateYX_drivers();
      uint16_t command(tsc2007_function func, tsc2007_power pwr,
                        tsc2007_resolution res);
      // byte MeasureX();
      // byte MeasureY();
      // byte MeasureZ1();
      // byte MeasureZ2();
      // byte ping();
   private:
      // byte setup();
      // void initCall(byte reg);
      // void endCall();
};

#endif