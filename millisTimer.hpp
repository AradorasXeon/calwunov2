//Implementing own timer function, because I suspect that I have freezes because i2c wants to communicate at some delay
#ifndef MILLIS_TIMER_H
#define MILLIS_TIMER_H

#include <Arduino.h>
class MillisTimer
{
    public:
    MillisTimer(unsigned long delayTime);
    void doDelay();
    static void delayThisMuch(unsigned long delayTime);

    private:
    unsigned long delayAmount = 0U;
    unsigned long startTime = 0U;
    unsigned long finishTime = 0U;
};

#endif // !MILLIS_TIMER_H