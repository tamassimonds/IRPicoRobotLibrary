from machine import Pin
import time

# Global variables to store pulse start time and duration
pulse_start_time = [0, 0]
pulse_duration = [0, 0]

# Interrupt handlers to capture start and end of pulse
def irq_handler_16(pin):
    global pulse_start_time, pulse_duration
    if pin.value():  # Rising edge
        pulse_start_time[0] = time.ticks_us()
    else:  # Falling edge
        pulse_duration[0] = time.ticks_diff(time.ticks_us(), pulse_start_time[0])

def irq_handler_17(pin):
    global pulse_start_time, pulse_duration
    if pin.value():  # Rising edge
        pulse_start_time[1] = time.ticks_us()
    else:  # Falling edge
        pulse_duration[1] = time.ticks_diff(time.ticks_us(), pulse_start_time[1])

# Initialize GPIO pins
pin_16 = Pin(16, Pin.IN, Pin.PULL_DOWN)
pin_17 = Pin(17, Pin.IN, Pin.PULL_DOWN)

# Attach interrupt handlers
pin_16.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irq_handler_16)
pin_17.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=irq_handler_17)

while True:
    # Print the pulse duration in microseconds
    print("PWM on Pin 16:", pulse_duration[0], "us", "PWM on Pin 17:", pulse_duration[1], "us")
    time.sleep(0.1)  # Delay for readability
