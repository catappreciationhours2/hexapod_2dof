# test_air_walk.py
from hexapod import controller
import time

USB_PORT = "/dev/ttyUSB0"

def test_gait():
    print("CAUTION: Prop robot up so legs don't touch ground!")
    time.sleep(2)
    
    print("Executing 100mm Forward Walk Cycle...")
    # distance=100mm, angle=90 (forward)
    controller.walkCycle(USB_PORT, distance=100, angle=90)
    
    print("Walking complete. Sitting...")
    controller.sit(USB_PORT)

if __name__ == "__main__":
    test_gait()