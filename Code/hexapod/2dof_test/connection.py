import serial
import time

# Update this if your port is different (e.g., /dev/ttyACM0)
PORT = "/dev/ttyUSB0" 
BAUD = 115200 # Standard for SSC-32U, try 9600 if this fails

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT} successfully!")
    
    # SSC-32U protocol: "#<channel> P<pulse_width> T<time>"
    # Move channel 0 to 1500us (center) over 500ms
    print("Moving servo on Channel 0 to center...")
    ser.write(b"#0 P1500 T500\r")
    time.sleep(1)
    
    # Move it to 1000us
    print("Moving servo on Channel 0 to left...")
    ser.write(b"#0 P1000 T500\r")
    time.sleep(1)
    
    ser.close()
    print("Test complete.")

except Exception as e:
    print(f"Error: {e}")