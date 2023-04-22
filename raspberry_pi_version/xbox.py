import os
import struct
import fcntl

device_path = "/dev/input/js0"
joystick_fd = os.open(device_path, os.O_RDONLY)

num_axes = 0
num_buttons = 0
fcntl.ioctl(joystick_fd, 0x80016a11, struct.pack('I', num_axes))
fcntl.ioctl(joystick_fd, 0x80016a12, struct.pack('I', num_buttons))
print("Joystick connected with", num_axes, "axes and", num_buttons, "buttons.")

arr = [0, 0, 0, 0, 0, 0, 0, 0]

while True:
    event = os.read(joystick_fd, 8)
    time, value, event_type, number = struct.unpack("IhBB", event)
    if event_type == 1:
        print("Button", number, "pressed with value", value)
    elif event_type == 2 and number != 1:
        arr[number] = value
        print("Axis", number, "moved to", value)
    print(arr)

os.close(joystick_fd)
