#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__ZS_X11_DRIVER_HPP
#define ROS2_CONTROL_DEMO_EXAMPLE_2__ZS_X11_DRIVER_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>

#include <pigpio.h>


class ZS_X11_Driver
{

public:
  ZS_X11_Driver() {
    instance_ = this;
  }

  signed long tick_counter_l = 0;
  signed long tick_counter_r = 0;
  signed char _direction_l = 1;
  signed char _direction_r = 1;

  static void leftSpeedPulseCallback(int gpio, int level, uint32_t tick)
  {
    if (instance_) {
	  instance_->tick_counter_l += instance_->_direction_l;
//          std::cout << "Left Interrupt. Tick counter: " << instance_->tick_counter_l << std::endl;
    }
  }

  static void rightSpeedPulseCallback(int gpio, int level, uint32_t tick)
  {
    if (instance_) {
          instance_->tick_counter_r += instance_->_direction_r;
//          std::cout << "Right Interrupt. Tick counter: " << instance_->tick_counter_r << std::endl;
    }
  }


  int connect()
  {  
    int init_result = gpioInitialise();
// Left wheel
    gpioSetMode(6, PI_OUTPUT); // Reverse
    gpioSetMode(26, PI_OUTPUT); // Brake
    gpioHardwarePWM(13, 0, 0); // PWM
//    gpioSetMode(16, PI_INPUT); // Speed Pulse
    gpioSetISRFunc(16, RISING_EDGE, 0, leftSpeedPulseCallback);
// Right wheel
    gpioSetMode(5, PI_OUTPUT); // Reverse - needs to be inverted
    gpioSetMode(25, PI_OUTPUT); // Brake
    gpioHardwarePWM(12, 0, 0); // PWM
//    gpioSetMode(19, PI_INPUT); // Speed Pulse
    gpioSetISRFunc(19, RISING_EDGE, 0, rightSpeedPulseCallback);
    return init_result;
  }

  void disconnect()
  {
    gpioSetISRFunc(16, RISING_EDGE, 0, nullptr);
    gpioSetISRFunc(19, RISING_EDGE, 0, nullptr);
    gpioTerminate();
  }


  void read_encoder_values(int &val_1, int &val_2)
  {
    val_1 = instance_->tick_counter_l;
    val_2 = instance_->tick_counter_l;
  }

  void set_motor_values(double left, double right)
  {
    const double RADIUS = 0.08; // TODO get radius from robot description)
    const double MOTOR_SHIFT = -0.155436252405753;
    const double MOTOR_ROC = 32.2418230613342;
    const double MIN_SPEED = 0.03;
    const double PWM_FREQUENCY = 10000;
    int result=0;
    int power = 0;

    if( left < 0 ) {
        gpioWrite(6, PI_ON);
	_direction_l = -1;
    } else {
        gpioWrite(6, PI_OFF);
	_direction_l = 1;
    }
    power = 0;
    if( abs(left*RADIUS) >= MIN_SPEED ) {
        power = static_cast<int>(round( abs(left * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
    }
    result=gpioHardwarePWM(13, PWM_FREQUENCY, power );
//    std::cout << "PIN13 PWM: " << PWM_FREQUENCY << "Input: " << left << "Power: " << power << " Result: " << result << std::endl;

    if( right > 0 ) {
        gpioWrite(5, PI_ON);
        _direction_r = 1;
    } else {
        gpioWrite(5, PI_OFF);
        _direction_r = -1;
    }
    power = 0;
    if( abs(right*RADIUS) >= MIN_SPEED ) {
        power = static_cast<int>( round(abs(right * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
    }
    result=gpioHardwarePWM(12, PWM_FREQUENCY, power );
//    std::cout << "PIN12 PWM: " << PWM_FREQUENCY << "Input: " << right << "Power: " << power << " Result: " << result << std::endl;
  }

  ~ZS_X11_Driver() {
    disconnect();
  }

private:
  int timeout_ms_;
  static ZS_X11_Driver* instance_;
};

#endif // ROS2_CONTROL_DEMO_EXAMPLE_2__ZS_X11_DRIVER_HPP
