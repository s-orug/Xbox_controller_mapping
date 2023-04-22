#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <pthread.h>
#include <unistd.h>

struct AxisThreadArgs {
  int joystick_fd;
  int axis_number;
};

struct ButtonThreadArgs {
  int joystick_fd;
  int button_number;
};

void* monitorAxis(void* arg) {
  AxisThreadArgs* args = static_cast<AxisThreadArgs*>(arg);
  struct js_event e;
  while (true) {
    if (read(args->joystick_fd, &e, sizeof(e)) > 0) {
      if (e.type == JS_EVENT_AXIS && e.number == args->axis_number) {
        std::cout << "Axis " << args->axis_number << " moved to " << e.value << std::endl;
      }
    }
  }
  return nullptr;
}

void* monitorButton(void* arg) {
  ButtonThreadArgs* args = static_cast<ButtonThreadArgs*>(arg);
  struct js_event e;
  while (true) {
    if (read(args->joystick_fd, &e, sizeof(e)) > 0) {
      if (e.type == JS_EVENT_BUTTON && e.number == args->button_number) {
        std::cout << "Button " << args->button_number << " pressed with value " << e.value << std::endl;
      }
    }
  }
  return nullptr;
}

int main() {
  // Open the joystick device
  int joystick_fd = open("/dev/input/js0", O_RDONLY);
  if (joystick_fd == -1) {
    std::cerr << "Could not open joystick device." << std::endl;
    return 1;
  }

  // Get the number of axes and buttons on the joystick
  int num_axes = 8, num_buttons = 12;
  ioctl(joystick_fd, JSIOCGAXES, &num_axes);
  ioctl(joystick_fd, JSIOCGBUTTONS, &num_buttons);
  std::cout << "Joystick connected with " << num_axes << " axes and " << num_buttons << " buttons." << std::endl;

  // Create a thread for each axis and button
  pthread_t threads[num_axes + num_buttons];
  int thread_count = 0;
  for (int i = 0; i < num_axes; i++) {
    AxisThreadArgs* args = new AxisThreadArgs{joystick_fd, i};
    pthread_create(&threads[thread_count++], nullptr, monitorAxis, static_cast<void*>(args));
  }
  for (int i = 0; i < num_buttons; i++) {
    ButtonThreadArgs* args = new ButtonThreadArgs{joystick_fd, i};
    pthread_create(&threads[thread_count++], nullptr, monitorButton, static_cast<void*>(args));
  }

  // Wait for all threads to finish
  for (int i = 0; i < num_axes + num_buttons; i++) {
    pthread_join(threads[i], nullptr);
  }

  // Close the joystick device
  close(joystick_fd);

  return 0;
}
