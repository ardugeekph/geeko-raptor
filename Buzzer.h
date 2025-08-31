#ifndef Buzzer_h
#define Buzzer_h

#include <Arduino.h>


class Buzzer {
    public:
        void begin();
        void beep(bool on);

    private:
};

#endif
