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

  ZS_X11_Driver() = default;

  void connect()
  {  
    gpioInitialise();
// Left wheel
    gpioSetMode(6, PI_OUTPUT); // Reverse
    gpioSetMode(26, PI_OUTPUT); // Brake
    gpioHardwarePWM(13, 0, 0); // PWM
    gpioSetMode(16, PI_INPUT); // Speed Pulse
// Right wheel
    gpioSetMode(5, PI_OUTPUT); // Reverse - needs to be inverted
    gpioSetMode(25, PI_OUTPUT); // Brake
    gpioHardwarePWM(12, 0, 0); // PWM
    gpioSetMode(19, PI_INPUT); // Speed Pulse

    timeout_ms_ = timeout_ms;
  }

  void disconnect()
  {
    gpioTerminate();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {

    std::string response = "";

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }
  void set_motor_values(double left, double right)
  {
    const double RADIUS = 0.08; // TODO get radius from robot description)
    const double MOTOR_SHIFT = -0.155436252405753;
    const double MOTOR_ROC = 32.2418230613342;
    const double MIN_SPEED = 0.03;
    const double PWM_FREQUENCY = 10000;

    int power = 0;

    if( left < 0 ) {
        gpioWrite(6, PI_ON);
    } else {
        gpioWrite(6, PI_OFF);
    }
    power = 0;
    if( abs(left*RADIUS) >= MIN_SPEED ) {
        power = static_cast<int>(round( abs(left * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
    }
    gpioHardwarePWM(13, PWM_FREQUENCY, power );

    if( right < 0 ) {
        gpioWrite(5, PI_ON);
    } else {
        gpioWrite(5, PI_OFF);
    }
    power = 0;
    if( abs(right*RADIUS) >= MIN_SPEED ) {
        power = static_cast<int>(round( abs(right * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
    }
    gpioHardwarePWM(12, PWM_FREQUENCY, power );
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

private:
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
