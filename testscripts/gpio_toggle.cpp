// Compile: g++ gpio_toggle.cpp -o gpio_toggle -lgpiod
#include <gpiod.h>
#include <iostream>
#include <unistd.h> // for sleep()

int main() {
    const char* chipname = "gpiochip0"; // Adjust based on your Pi5 GPIO chip
    unsigned int line_num = 27;         // GPIO pin number (BCM numbering)

    gpiod_chip* chip = gpiod_chip_open_by_name(chipname);
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, line_num);
    if (!line) {
        std::cerr << "Failed to get GPIO line\n";
        gpiod_chip_close(chip);
        return 1;
    }

    if (gpiod_line_request_output(line, "gpio_toggle", 0) < 0) {
        std::cerr << "Failed to request line as output\n";
        gpiod_chip_close(chip);
        return 1;
    }

    // Toggle the pin
    gpiod_line_set_value(line, 1);
    sleep(1);
    gpiod_line_set_value(line, 0);

    gpiod_line_release(line);
    gpiod_chip_close(chip);

    return 0;
}

