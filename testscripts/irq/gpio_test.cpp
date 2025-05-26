#include <pigpio.h>
#include <iostream>

void testCallback(int gpio, int level, uint32_t tick) {
    std::cout << "Interrupt on GPIO " << gpio << " at tick " << tick << std::endl;
}

int main() {
    if (gpioInitialise() < 1) return 1;

    std::cout << "Set Mode: " << gpioSetMode(16, PI_INPUT) << std::endl;
//    gpioSetPullUpDown(16, PI_PUD_UP);
    std::cout << "Set ISR: " << gpioSetISRFunc(16, RISING_EDGE, 0, testCallback) << std::endl;
    std::cout << "Waiting for interrupt..." << std::endl;
    while (true) gpioDelay(100001);  // Sleep 100ms

    gpioTerminate();
    return 1;
}
