import time
from hexapod.controller import stand, sit, walkCycle

# Your PCA9685 default I2C address
PCA_ADDRESS = 0x40 

def run_air_walk_test():
    try:
        print("2. Starting Walk Cycle (Forward 100mm)...")
        # distance = 100mm, angle = 90 degrees (straight forward)
        walkCycle(PCA_ADDRESS, distance=100, angle=90)
        
        print("3. Walk complete. Waiting...")
        time.sleep(2)

        print("4. Sitting down...")
        sit(PCA_ADDRESS)
        print("Test Finished Successfully.")

    except Exception as e:
        print(f"Test Failed: {e}")
    except KeyboardInterrupt:
        print("\nTest stopped by user. Sitting...")
        sit(PCA_ADDRESS)

if __name__ == "__main__":
    # WARNING: Make sure the robot is propped up so the legs 
    # move freely in the air before running this!
    run_air_walk_test()