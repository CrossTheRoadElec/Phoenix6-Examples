import time
from phoenix6.hardware import CANcoder

# Initialize the CANcoder with the appropriate device ID
DEVICE_ID = 1  # Replace with your actual device ID
CAN_INTERFACE = 'can0'

cancoder = CANcoder(DEVICE_ID, CAN_INTERFACE)

try:
    while True:

        position = cancoder.get_absolute_position().value
        print(f"Absolute Position: {position:.4f} rotations")
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("Stopped reading CANcoder.")