import time
from phoenix6 import hardware, controls, unmanaged

def main():
    # Create a TalonFX motor controller on CAN ID 2
    motor = hardware.TalonFX(2, "can0")
    
    # Create a duty cycle control request
    duty_cycle = controls.DutyCycleOut(0)
    
    print("Starting motor ramp test...")
    
    # Enable the motor
    unmanaged.feed_enable(100)
    
    # Ramp up from 0 to 1
    for i in range(0, 11):
        duty_cycle.output = i / 10.0
        motor.set_control(duty_cycle)
        unmanaged.feed_enable(100)  # Keep feeding the enable
        print(f"Duty Cycle: {i/10.0}")
        time.sleep(0.5)
    
    # Ramp down from 1 to -1
    for i in range(10, -11, -1):
        duty_cycle.output = i / 10.0
        motor.set_control(duty_cycle)
        unmanaged.feed_enable(100)  # Keep feeding the enable
        print(f"Duty Cycle: {i/10.0}")
        time.sleep(0.5)
    
    # Ramp up from -1 to 0
    for i in range(-10, 1):
        duty_cycle.output = i / 10.0
        motor.set_control(duty_cycle)
        unmanaged.feed_enable(100)  # Keep feeding the enable
        print(f"Duty Cycle: {i/10.0}")
        time.sleep(0.5)
    
    # Stop the motor
    motor.set_control(controls.NeutralOut())
    unmanaged.feed_enable(100)  # Final feed
    print("Test complete")

if __name__ == "__main__":
    main() 
