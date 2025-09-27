#ifndef BLUHMBOT__ZK_BM1_DRIVER_HPP
#define BLUHMBOT__ZK_BM1_DRIVER_HPP

#include <string>
#include <sstream>
#include <cstdlib>
#include <iostream>


#include <rpi_pwm.h>

#include <set>
#include <utility>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <atomic>

#define DEBUG false
// 400Hz to 2kHz or module will get too hot
#define MIN_PWM_FREQUENCY 400
#define MAX_PWM_FREQUENCY 2000

class zk_bm1_Driver
{

public:
  zk_bm1_Driver() = default;
  ~zk_bm1_Driver() { disconnect(); }

  // Non-copyable (hardware handles / device state are unique)
  zk_bm1_Driver(const zk_bm1_Driver&)            = delete;
  zk_bm1_Driver& operator=(const zk_bm1_Driver&) = delete;

  // Movable (optional, handy for containers)
  zk_bm1_Driver(zk_bm1_Driver&&) noexcept        = default;
  zk_bm1_Driver& operator=(zk_bm1_Driver&&) noexcept = default;

  int connect(int in1, int in2, int frequency)
  {
    pin_in1 = in1;
    pin_in2 = in2;
    pwm_frequency = frequency;

    // Protecting hardware for non-conformance of PWM frequency to avoid overheating.
    if( frequency < 400 ) pwm_frequency = MIN_PWM_FREQUENCY;
    if( frequency > 2000 ) pwm_frequency = MAX_PWM_FREQUENCY;

    if( DEBUG ) std::cout << "[DEBUG] ZK-BM1 Driver - Received in1 " << in1 << "; in2 " << in2 << " Assigning pin_in1 "<< pin_in1 << " and pin_in2 " << pin_in2 << std::endl;
    // Check for shared PWM controllers:
    if ( _getPWM(pin_in1) == _getPWM(pin_in2)) {
      single_pwm = true;
    };
    if( DEBUG ) std::cout << "[DEBUG] ZK-BM1 Driver - Enabling pin " << pin_in1 << " and " << pin_in2 << std::endl;
    _enablePWM(pwm_in1, pin_in1);
    if ( !single_pwm ) _enablePWM(pwm_in2, pin_in2);
    return 0;
  }

  void disconnect()
  {
    _disablePWM(pwm_in1);
    if (!single_pwm) _disablePWM(pwm_in2);
  }

  // Non-blocking: set the desired target [-100..100] percent.
  void set_motor_values(double speed_percent)
  {
    double s = std::clamp(invert * speed_percent, -100.0, 100.0);
    target_pct_.store(static_cast<float>(s), std::memory_order_relaxed);
  }

  // Configure ramp (units: percent-per-second, seconds)
  void set_ramp(double max_accel_pct_s, double max_decel_pct_s, double reverse_dwell_s = 0.5)
  {
    max_accel_pct_s_ = std::max(0.0f, static_cast<float>(max_accel_pct_s));
    max_decel_pct_s_ = std::max(0.0f, static_cast<float>(max_decel_pct_s));
    reverse_dwell_s_ = std::max(0.0f, static_cast<float>(reverse_dwell_s));
  }

  // Update with explicit dt (seconds)
  void update(double dt_seconds)
  {
    float dt = static_cast<float>(std::max(0.0, dt_seconds));
    if (dt <= 0.0f) return;
    _step(dt);
  }

  // Self-timed update using steady clock (use if you don't have dt)
  void update()
  {
    using clock = std::chrono::steady_clock;
    auto now = clock::now();
    if (!have_last_tick_) {
      last_tick_ = now;
      have_last_tick_ = true;
      return;
    }
    float dt = std::chrono::duration<float>(now - last_tick_).count();
    // Clamp dt to avoid giant steps after stalls
    dt = std::clamp(dt, 0.0f, 0.2f); // max 200 ms per step
    last_tick_ = now;
    if (dt > 0.0f) _step(dt);
  }

//    std::cout << "PIN13 PWM: " << PWM_FREQUENCY << " - Input: " << left << " - Result: " << result << std::endl;

  void set_invert(bool invert_dir) {
    invert = invert_dir?-1:1;
  };

  // Min speed threshold in percent (e.g., 8.0 means anything below |8|% outputs 0%)
  void set_min_speed(float speed_percent) { min_speed_pct_ = speed_percent; }

private:
  bool single_pwm = false;

  RPI_PWM pwm_in1;
  RPI_PWM pwm_in2;
  int pin_in1;
  int pin_in2;
  int pwm_frequency = 400;
  // Ramping state in percent [-100..100]
  std::atomic<float> target_pct_{0.0f};
  float current_pct_ = 0.0f;
  float last_applied_pct_ = 0.0f;
  float max_accel_pct_s_ = 50.0f;   // default: 0→100% in 2 s
  float max_decel_pct_s_ = 80.0f;   // default: faster decel
  float reverse_dwell_s_ = 0.5f;
  float dwell_remaining_s_ = 0.0f;
  bool  dwell_active_ = false;
  const float epsilon_pct_ = 0.05f; // snap-to-zero threshold (percent)
  bool have_last_tick_ = false;
  std::chrono::steady_clock::time_point last_tick_;
  int invert = 1;
  float min_speed_pct_ = 0.0f;

  void _setPinFunction(int gpio, const std::string& alt) {
    std::string command = "pinctrl set " + std::to_string(gpio) + " " + alt;
    int result = system(command.c_str());
    if (result != 0) {
        std::cout << "[ERROR] ZK-BM1 Driver Failed to set GPIO" << gpio << " to " << alt << "\n";
    }
  }

  void _enablePWM(RPI_PWM& pwm, int pin) {
      int result = pwm.start(_getPWM(pin), pwm_frequency, 0);
      if( DEBUG ) std::cout << "[DEBUG] ZK-BM1 Driver -  pwm.start PIN " << pin << " Result: " << result << std::endl;
  }

  void _disablePWM(RPI_PWM& pwm) {
      pwm.stop();
  }

  std::string _getMap(int gpio) {
    static const std::unordered_map<int, std::string> gpioToMap = {
        {12, "a0"},
        {13, "a0"},
        {14, "a0"},
        {15, "a0"},
        {18, "a3"},
        {19, "a3"}
    };

    auto it = gpioToMap.find(gpio);
    if (it != gpioToMap.end()) {
        return it->second;
    } else {
        std::cout << "[ERROR] ZK-BM1 Driver PWM Mode for GPIO " << gpio << " not found." << std::endl;
        return "";
    }
  };

  int _getPWM(int gpio) {
    static const std::unordered_map<int, int> gpioToPWM = {
        {12, 0},
        {13, 1},
        {14, 2},
        {15, 3},
        {18, 2},
        {19, 3}
    };

    auto it = gpioToPWM.find(gpio);
    if (it != gpioToPWM.end()) {
        return it->second;
    } else {
        std::cout << "[ERROR] ZK-BM1 Driver PWM for GPIO " << gpio << " not found." << std::endl;
        return -1;
    }
  };

  // One integration step of the slew/dwell logic
  void _step(float dt)
  {
    const float target = target_pct_.load(std::memory_order_relaxed);

    // Direction change? (current non-zero and target has opposite sign)
    const bool opposite_sign = (current_pct_ * target < 0.0f) && (std::fabs(current_pct_) > epsilon_pct_);

    if (opposite_sign || dwell_active_) {
      // Ramp to zero using decel limit
      if (!dwell_active_) {
        const float to_zero = -current_pct_;
        const float lim = max_decel_pct_s_ * dt;
        const float step = std::clamp(to_zero, -lim, lim);
        current_pct_ += step;
        if (std::fabs(current_pct_) <= epsilon_pct_) {
          current_pct_ = 0.0f;
          dwell_active_ = (reverse_dwell_s_ > 0.0f);
          dwell_remaining_s_ = reverse_dwell_s_;
        }
      }
      // Dwell at zero if needed
      if (dwell_active_) {
        current_pct_ = 0.0f;
        dwell_remaining_s_ -= dt;
        if (dwell_remaining_s_ <= 0.0f) dwell_active_ = false;
      }
      _apply_speed_percent(current_pct_);
      return;
    }

    // Normal slew toward target (choose accel or decel limit)
    const bool decel = (std::fabs(target) < std::fabs(current_pct_));
    const float lim = (decel ? max_decel_pct_s_ : max_accel_pct_s_) * dt;
    const float delta = std::clamp(target - current_pct_, -lim, lim);
    current_pct_ += delta;
    if (std::fabs(current_pct_) < epsilon_pct_) current_pct_ = 0.0f;

    _apply_speed_percent(current_pct_);
  }

  // Write the signed percent to hardware, honoring single/dual-PWM and min-speed
  void _apply_speed_percent(float signed_pct)
  {
    float hw = (std::fabs(signed_pct) < std::fabs(min_speed_pct_)) ? 0.0f : signed_pct;
    hw = std::clamp(hw, -100.0f, 100.0f);

    if (single_pwm) {
      // Re-select the active pin only when we leave zero again
      const bool leaving_zero = (std::fabs(last_applied_pct_) <= epsilon_pct_) && (std::fabs(hw) > epsilon_pct_);
      if (leaving_zero) {
        pwm_in1.setDutyCycle(0.0f);
        _disablePWM(pwm_in1);
        if (hw < 0.0f) {
          _setPinFunction(pin_in1, "ip");
          _setPinFunction(pin_in2, _getMap(pin_in2));
          _enablePWM(pwm_in1, pin_in2);
        } else {
          _setPinFunction(pin_in2, "ip");
          _setPinFunction(pin_in1, _getMap(pin_in1));
          _enablePWM(pwm_in1, pin_in1);
        }
      }
      pwm_in1.setDutyCycle(std::fabs(hw)); // percent 0..100
    } else {
      if (hw < 0.0f) {
        pwm_in1.setDutyCycle(0.0f);
        pwm_in2.setDutyCycle(std::fabs(hw));
      } else if (hw == 0.0f) {
        pwm_in1.setDutyCycle(0.0f);
        pwm_in2.setDutyCycle(0.0f);
      } else {
        pwm_in1.setDutyCycle(std::fabs(hw));
        pwm_in2.setDutyCycle(0.0f);
      }
    }
    last_applied_pct_ = hw;
  }
};

#endif // BLUHMBOT__ZK_BM1_DRIVER_HPP
