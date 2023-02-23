#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

int main() {
    // Open the joystick device
    int joystick_fd = open("/dev/input/js0", O_RDONLY);
    if (joystick_fd == -1) {
        std::cerr << "Could not open joystick device." << std::endl;
        return 1;
    }

    // Get the number of axes and buttons on the joystick
    int num_axes = 0, num_buttons = 0;
    ioctl(joystick_fd, JSIOCGAXES, &num_axes);
    ioctl(joystick_fd, JSIOCGBUTTONS, &num_buttons);
    std::cout << "Joystick connected with " << num_axes << " axes and " << num_buttons << " buttons." << std::endl;

    // Read the joystick events
    struct js_event e;
    while (read(joystick_fd, &e, sizeof(e)) > 0) {
        if (e.type == JS_EVENT_AXIS) {
            // Handle axis events here
            std::cout << "Axis " << static_cast<int>(e.number) << " moved to " << e.value << std::endl;
        } else if (e.type == JS_EVENT_BUTTON) {
            // Handle button events here
            std::cout << "Button " << static_cast<int>(e.number) << " pressed with value " << e.value << std::endl;
        }
    }

    // Close the joystick device
    close(joystick_fd);

    return 0;
}
