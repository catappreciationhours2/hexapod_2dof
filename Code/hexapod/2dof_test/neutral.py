# test_calibration.py
from hexapod import controller
import time

USB_PORT = "/dev/ttyUSB0"  # Update this to your Pi's serial port

def test_neutral_pose():
    print("Testing Stand Position... Robot should lift to 60mm height.")
    controller.stand(USB_PORT)
    time.sleep(5)
    
    print("Testing Sit Position... Robot should lower to 10mm height.")
    controller.sit(USB_PORT)
    time.sleep(5)

if __name__ == "__main__":
    test_neutral_pose()