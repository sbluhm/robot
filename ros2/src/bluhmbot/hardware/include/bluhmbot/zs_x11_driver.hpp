#ifndef BLUHMBOT__ZS_X11_DRIVER_HPP
#define BLUHMBOT__ZS_X11_DRIVER_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>

#include <rpi_pwm.h>
#include <gpiod.h>
#include <thread>
#include <stop_token>

#define CHIP_NAME "gpiochip0"        // Adjust based on your GPIO chip (use `gpiodetect`)
#define LEFT_REVERSE 6              // GPIO6 as output
#define RIGHT_REVERSE 5             //
#define LEFT_SPEED_PULSE 14         // GPIO16 as input
#define RIGHT_SPEED_PULSE 4        // GPIO11 as input

#define PWM_FREQUENCY 10000


class ZS_X11_Driver
{

public:
  ZS_X11_Driver() {
    instance_ = this;
    stop_requested = false;
  }

  signed long tick_counter_l = 0;
  signed long tick_counter_r = 0;
  signed char _direction_l = 1;
  signed char _direction_r = 1;

  int connect()
  {
    chip = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip) {
	std::cerr << "[ERROR] Failed to open GPIO chip: " << CHIP_NAME << "\n";
        return 1;
    }


    // Configure output lines
    // LEFT REVERSE
    left_reverse = gpiod_chip_get_line(chip, LEFT_REVERSE);
    if (!left_reverse || gpiod_line_request_output(left_reverse, "gpio_output", 0) < 0) {
        std::cerr << "[ERROR] Failed to configure LEFT_REVERSE line\n";
	return 1;
    }

    // RIGHT SPEED PULSE
    right_reverse = gpiod_chip_get_line(chip, RIGHT_REVERSE);
    if (!right_reverse || gpiod_line_request_output(right_reverse, "gpio_output", 1) < 0) {
	std::cerr << "[ERROR] Failed to configure RIGHT_REVERSE line\n";
	return 1;
    }

    // Configure input line with rising edge detection

    // LEFT SPEED PULSE
    left_speed_pulse = gpiod_chip_get_line(chip, LEFT_SPEED_PULSE);
    if (!left_speed_pulse || gpiod_line_request_rising_edge_events(left_speed_pulse, "gpio_interrupt") < 0) {
	std::cerr << "[ERROR] Failed to configure LEFT_SPEED_PULSE line\n";
	return 1;
    }

    // RIGHT SPEED PULSE
    right_speed_pulse = gpiod_chip_get_line(chip, RIGHT_SPEED_PULSE);
    if (!right_speed_pulse || gpiod_line_request_rising_edge_events(right_speed_pulse, "gpio_interrupt") < 0) {
	std::cerr << "[ERROR] Failed to configure RIGHT_SPEED_PULSE line\n";
	return 1;
    }

// Left wheel
//    gpioSetMode(26, PI_OUTPUT); // Brake
    pwm_left.start(1, PWM_FREQUENCY, 0); // PWM on Channel 1 / GPIO13
// Right wheel
//    gpioSetMode(25, PI_OUTPUT); // Brake
    pwm_right.start(0, PWM_FREQUENCY, 0); // PWM on Channel 0 / GPIO12

// Start GPIO Monitoring
    
    tleft = std::thread(&ZS_X11_Driver::leftSpeedPulseMonitor, this);
    tright = std::thread(&ZS_X11_Driver::rightSpeedPulseMonitor, this);

    std::cout << "[INFO] ZS_X11_Driver successfully connected.\n";

    return 0;
  }

  inline void disconnect()
  {
    stop_requested = true;
    if (tleft.joinable()) tleft.join();
    if (tright.joinable()) tright.join();

    pwm_left.stop();
    pwm_right.stop();
    gpiod_line_release(left_reverse);
    gpiod_line_release(right_reverse);
    gpiod_line_release(left_speed_pulse);
    gpiod_line_release(right_speed_pulse);
    gpiod_chip_close(chip);
  }


  inline void read_encoder_values(signed long &val_1, signed long &val_2)
  {
    val_1 = instance_->tick_counter_l;
    val_2 = instance_->tick_counter_r;
  }

  inline void set_motor_values(double left, double right)
  {
    const float RADIUS = 0.08f; // TODO get radius from robot description)
    const float MIN_SPEED = 0.027f;
    const float A = 227.638572f;
    const float B = 15.904997f;
    const float C = 39.948085f;
    float power = 0.0f;

    // LEFT MOTOR
    if (left_reverse && gpiod_line_is_requested(left_reverse)) {
        if( left < 0 ) {
            gpiod_line_set_value(left_reverse, 1);
            _direction_l = -1;
        } else {
            gpiod_line_set_value(left_reverse, 0);
	    _direction_l = 1;
        }
    } else {
        std::cerr << "[ERROR] left_reverse line is not valid or not requested.\n";
    }

    float speed_l = std::abs(static_cast<float>(left)) * RADIUS;
    if( speed_l >= MIN_SPEED ) {
//        power = static_cast<int>(round( abs(left * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
	power = ( A * std::exp(-B * speed_l) + C ) * speed_l;
//	power = static_cast<int>( abs(left*RADIUS) / 5 * 1000000   );
    }
    pwm_left.setDutyCycle(power);
//    std::cout << "PIN13 PWM: " << PWM_FREQUENCY << " - Input: " << left << " - Power: " << power << " - Result: " << result << std::endl;

    // RIGHT MOTOR
    if (right_reverse && gpiod_line_is_requested(right_reverse)) {
        if( right > 0 ) {
            gpiod_line_set_value(right_reverse, 1);
            _direction_r = 1;
        } else {
            gpiod_line_set_value(right_reverse, 0);
            _direction_r = -1;
        }
    } else {
        std::cerr << "[ERROR] right_reverse line is not valid or not requested.\n";
    }

    power = 0;
    float speed_r = std::abs(static_cast<float>(right)) * RADIUS;
    if( speed_r >= MIN_SPEED ) {
//        power = static_cast<int>( round(abs(right * RADIUS) + MOTOR_SHIFT ) * MOTOR_ROC * 10000 );
        power = ( A * std::exp(-B * speed_r) + C ) * speed_r;
//	power = static_cast<int>( abs(right*RADIUS) / 5 * 1000000   );
    }
    pwm_right.setDutyCycle(power);
//    std::cout << "PIN12 PWM: " << PWM_FREQUENCY << " - Input: " << right << " - Power: " << power << " - Result: " << result << std::endl;
  }

  ~ZS_X11_Driver() {
    disconnect();
  }

private:
  static inline void leftSpeedPulseMonitor(ZS_X11_Driver* self)
  {
    while (!self->stop_requested) {
	struct gpiod_line_event event;
	int ret = gpiod_line_event_wait(self->left_speed_pulse, nullptr);
	if (ret < 0) break;
        if (gpiod_line_event_read(self->left_speed_pulse, &event) == 0 &&
			event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
	    self->tick_counter_l += self->_direction_l;
	}
    }
  }

  static inline void rightSpeedPulseMonitor(ZS_X11_Driver* self)
  {
    while (!self->stop_requested) {
	struct gpiod_line_event event;
	int ret = gpiod_line_event_wait(self->right_speed_pulse, nullptr);
	if (ret < 0) break;
	if (gpiod_line_event_read(self->right_speed_pulse, &event) == 0 &&
			event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
	    self->tick_counter_r += self->_direction_r;
	}
    }
  } 

  int timeout_ms_;


  gpiod_chip* chip = nullptr;
  gpiod_line* left_reverse = nullptr;
  gpiod_line* right_reverse = nullptr;
  gpiod_line* left_speed_pulse = nullptr;
  gpiod_line* right_speed_pulse = nullptr;

  std::atomic<bool> stop_requested;
  std::thread tleft;
  std::thread tright;

  static ZS_X11_Driver* instance_;
  RPI_PWM pwm_left;
  RPI_PWM pwm_right;

  ZS_X11_Driver(const ZS_X11_Driver&) = delete;
  ZS_X11_Driver& operator=(const ZS_X11_Driver&) = delete;
};

#endif // BLUHMBOT__ZS_X11_DRIVER_HPP
