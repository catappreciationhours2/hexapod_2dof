# test_body_move.py
from hexapod.body import bodyPos, bodyAngle
from hexapod.leg import startLegPos, recalculateLegAngles
from hexapod.ssc32uDriver import connect, anglesToSerial, sendData
import time

USB_PORT = "/dev/ttyUSB0"

def test_body_lean():
    port = connect(USB_PORT)
    
    # 1. Start in neutral
    body_model = bodyPos(pitch=0, roll=0, yaw=0)
    leg_angles = startLegPos(body_model, start_radius=170, start_height=60)
    sendData(port, anglesToSerial(leg_angles, 500, 1000))
    time.sleep(2)

    # 2. Test Pitch (Lean Forward/Backward)
    print("Leaning Forward...")
    # Simulate analog stick forward (analog_y = 1)
    pitch, roll = bodyAngle(0, 1, max_angle=10) 
    new_body = bodyPos(pitch=pitch, roll=roll)
    # Re-calculate angles based on STATIC feet positions but MOVING body
    feet_pos = [[170*0.5, 147, -60], [170, 0, -60], [85, -147, -60], 
                [-85, -147, -60], [-170, 0, -60], [-85, 147, -60]]
    
    new_angles = recalculateLegAngles(feet_pos, new_body)
    sendData(port, anglesToSerial(new_angles, 500, 1000))
    
    time.sleep(3)
    print("Returning to neutral...")
    sendData(port, anglesToSerial(leg_angles, 500, 1000))

if __name__ == "__main__":
    test_body_lean()