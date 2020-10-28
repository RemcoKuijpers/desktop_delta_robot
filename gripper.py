from gpiozero import Servo
from time import sleep

class Gripper():
    def __init__(self):
        self.servo = Servo(17, min_pulse_width=400/1000000, max_pulse_width=2400/1000000)

    def close(self):
        self.servo.value = 0.2

    def open(self):
        self.servo.value = -0.8

if __name__ == "__main__":
    gripper = Gripper()
    while True:
        try:
            print("Closing Gripper")
            gripper.close()
            sleep(2)
            print("Opening Gripper")
            gripper.open()
            sleep(2)
        except KeyboardInterrupt:
            gripper.servo.detach()
            print("Gripper detached")
            break
