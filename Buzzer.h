#ifndef Buzzer_h
#define Buzzer_h

#include <Arduino.h>


class Buzzer {
    public:
        void begin();
        void beep(bool on);
        void pulse(unsigned long durationMs = 80);
        void tick();

    private:
        unsigned long pulseEndMs_ = 0;
};

#endif
