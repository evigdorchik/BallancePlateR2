#include <Wire.h>
#include <Servo.h>
#include "TSC2007.h"

class TSC2007 {
         public:
            TSC2007();
            ~TSC2007();
            byte MeasureX();
            byte MeasureY();
            byte MeasureZ1();
            byte MeasureZ2();
            void reset();
         private:
            byte getReg(reg_ptr_t regPtr);
            byte getPin(pin_t pin, reg_ptr_t regPtr);
            void initCall(reg_ptr_t regPtr);
            void endCall();
    };