// Compile: g++ -std=c++17 -o gpio_monitor gpio_monitor.cpp -lgpiod -pthread
#include <gpiod.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

std::atomic<bool> running(true);

void monitor_gpio(const std::string& chipname, unsigned int line_num) {
    gpiod_chip* chip = gpiod_chip_open_by_name(chipname.c_str());
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "Failed to get GPIO line\n";
        gpiod_chip_close(chip);
        return;
    }

    if (gpiod_line_request_rising_edge_events(line, "gpio-monitor") < 0) {
        std::cerr << "Failed to request rising edge events\n";
        gpiod_chip_close(chip);
        return;
    }

    std::cout << "Monitoring GPIO" << line_num << " for rising edges...\n";

    while (running) {
        struct gpiod_line_event event;
        int ret = gpiod_line_event_wait(line, nullptr);
        if (ret < 0) {
            std::cerr << "Error waiting for event\n";
            break;
        } else if (ret == 0) {
            continue; // timeout
        }

        if (gpiod_line_event_read(line, &event) == 0 &&
            event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            std::cout << "Rising edge detected!\n";
        }
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
}

int main() {
    std::string chipname = "gpiochip0"; // Adjust if needed
    unsigned int line_num = 4;         // GPIO pin number

    std::thread gpio_thread(monitor_gpio, chipname, line_num);

    std::cout << "Press Enter to stop...\n";
    std::cin.get();

    running = false;
    gpio_thread.join();

    std::cout << "GPIO monitoring stopped.\n";
    return 0;
}

