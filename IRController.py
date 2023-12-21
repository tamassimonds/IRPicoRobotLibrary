import framebuf
from machine import Pin, I2C, UART, ADC
import machine
# from ssd1306 import SSD1306_I2C
# from vl53l0x import VL53L0X
import struct
import math
import time



led = Pin(25, Pin.OUT)

led.on()

print("Setting up i2c")
sda = Pin(14)
scl = Pin(15)
id = 1  # Set to 1 if using I2C1

#UART
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

try:
    i2c = I2C(id=id, sda=sda, scl=scl)
except ValueError as e:
    print("Failed to initialize I2C:", e)
    raise



# Desired frame rate (e.g., 10 updates per second)
desired_update_rate = 30
# Calculate the time each loop should take in seconds
loop_duration = 1.0 / desired_update_rate

sensorsInitialized = [] #This is a collection of all the sensors that have been defined by user


servoInputToGPIOPin = [] #Servo input num to gpio pin
motorPinMap = []

motors = [] # Start at 1. 0 is None to match board markings


class TOFDistanceSensor():

    value = None
    tof = None
    def __init__(self):
        
        #Initilize the sensor
        try:
            self.tof = VL53L0X(i2c)
            self.tof.set_measurement_timing_budget(80000)  # Value in microseconds, try adjusting this value
            # Update VCSEL pulse periods
            self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[0], 18)  # Pre-ranges
            self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[1], 14)  # Final range
            
            sensorsInitialized.append(self)
        except:
            print("Error initializing TOF sensor")
            print("Make sure it's plugged in")
            
    def update(self):
        sensorValue = self.tof.ping()-50
        #The sensor max is 2000mmm, so we will set the max to 2000mm
        if sensorValue > 2000:
            sensorValue = 2000
        self.value = sensorValue

        
class OLEDDisplay():

    oled = None
    def __init__(self):
        pix_res_x  = 128 # SSD1306 horizontal resolution
        pix_res_y = 64
        self.oled = SSD1306_I2C(pix_res_x, pix_res_y, i2c) # oled controller
        self.oled.write_cmd(0xc0)
    
    def update(self):
        self.oled.text("ADC: ",5,8)
        self.oled.text(str(round(123,2)),40,8)


class ServoMotor():
    servo = None
    angle = 0  # Current angle of the servo motor
    
    def __init__(self, servoNum):
        # Initialize the PWM on the given Pin
        self.servo = machine.PWM(machine.Pin(servoInputToGPIOPin[servoNum]))
        
        # Set the frequency to 50Hz
        self.servo.freq(50)
        
        # Initialize servo at 0 degrees
        self.set_angle(0)
        
        # Append the servo object to the list of initialized sensors
        sensorsInitialized.append(self)
    
    def set_angle(self, angle):
        # Assuming pulse width range is from 1ms to 2ms
        # and servo angle is from 0 to 180 degrees
        pulse_width = int(1000 + angle * (1000 / 180))
        
        # Convert pulse width to duty cycle (0 to 65535)
        duty_cycle = int((pulse_width * 65535) // 20000)
        
        # Set the duty cycle
        self.servo.duty_u16(duty_cycle)
        
        # Update the current angle
        self.angle = angle
    
    def update(self):
        # For now, just setting the servo to its current angle
        # You could expand this method to include more complex behavior
        self.set_angle(self.angle)

class Motor():
    forward_pin = None
    reverse_pin = None
    speed = 0  # Current speed of the motor (can be positive or negative)
    
    def __init__(self, motorNum, flipDirection=False):
        
        
        if flipDirection:
            self.forward_pin = machine.PWM(machine.Pin(motorPinMap[motorNum][0]))
            self.reverse_pin = machine.PWM(machine.Pin(motorPinMap[motorNum][1]))
        else:
            self.forward_pin = machine.PWM(machine.Pin(motorPinMap[motorNum][1]))
            self.reverse_pin = machine.PWM(machine.Pin(motorPinMap[motorNum][0]))
            


        pwm_frequency = 1000  
        self.forward_pin.freq(pwm_frequency)
        self.reverse_pin.freq(pwm_frequency)


        # Initialize the motor to stop
        self.set_speed(0)
        
        # Append the motor object to the list of initialized sensors
        sensorsInitialized.append(self)
    
    def set_speed(self, speed):
        #Speed 0 - 100
        # If speed is positive, go forward; if negative, go reverse
        if speed >= 0:
            self.forward_pin.duty_u16(int(speed * 65535 // 100))
            self.reverse_pin.duty_u16(0)
        else:
            self.forward_pin.duty_u16(0)
            self.reverse_pin.duty_u16(int(-speed * 65535 // 100))
        
        # Update the current speed
        self.speed = speed
    
    def stop(self):
        self.set_speed(0)
    
    def update(self):
        # For now, just setting the motor to its current speed
        # You could expand this method to include more complex behavior
        self.set_speed(self.speed)


class Movement:
    
    drive_train = None #Changes Movement depending on drive train setup
    # drive_train = "Mecanum"
    # drive_train = "Tank"
    # drive_train = "Omni"

    def __init__(self, drive_train):
        self.drive_train = drive_train

    def stop_all():
        for motor in motors:
            motor.stop()
    
   
    def forward(speed):
        pass
    def backward(speed):
        pass
    def right(speed):
        pass
    def left(speed):
        pass
    
    


import math
from ustruct import pack, unpack

class HMC5883L:
    address = 0x1E  # I2C address for HMC5883L
    
    # Register addresses
    CONFIG_A = 0x00
    CONFIG_B = 0x01
    MODE = 0x02
    DATA_X_MSB = 0x03
    DATA_X_LSB = 0x04
    DATA_Z_MSB = 0x05
    DATA_Z_LSB = 0x06
    DATA_Y_MSB = 0x07
    DATA_Y_LSB = 0x08
    
    def __init__(self, ):
        self.i2c = i2c
        x, y, z = self.read_raw_data()
        
        # Calculate angle in radians
        angle_rad = math.atan2(y, x)
        self.initial_direction = math.degrees(angle_rad)
        self.current_angle = 0.0
        
        # Initialize sensor settings
        self.i2c.writeto_mem(self.address, self.CONFIG_A, b'\x70')  # 8-average, 15 Hz, normal measurement
        self.i2c.writeto_mem(self.address, self.CONFIG_B, b'\xa0')  # Gain=5
        self.i2c.writeto_mem(self.address, self.MODE, b'\x00')  # Continuous measurement mode
        sensorsInitialized.append(self)

    def read_raw_data(self):
        data = self.i2c.readfrom_mem(self.address, self.DATA_X_MSB, 6)
        x, z, y = unpack('>hhh', data)
        return x, y, z
    
    def update(self):
        x, y, z = self.read_raw_data()
        
        # Calculate angle in radians
        angle_rad = math.atan2(y, x)
        
        # Convert to degrees
        angle_deg = math.degrees(angle_rad)
        
        # Adjust by initial direction
        self.current_angle = angle_deg - self.initial_direction
        
        # Normalize to [-180, 180]
        self.current_angle = (self.current_angle + 180) % 360 - 180
       
class DipSwitch:
    
    #decimal is decimal represenation
    #state is array of switches values eg [0,0,1,0]
    dip_switch_state = []
    decimal = None

    def __init__(self, pin_number):
        self.adc = ADC(Pin(pin_number))
        self.thresholds = [300, 7400, 12600, 17500, 22200, 25200, 28000, 30500, 33000, 35000, 36600, 38000, 39600, 41000, 42000]
        self.dip_switch_state = [0] * len(self.thresholds)  # Initialize all to 0
        self.decimal = 0  # Count of active switches

    def decimal_to_binary_array(self, decimal_number):
        # Get the binary representation, then reverse it using reversed()
        binary_string = bin(decimal_number)[2:]
        reversed_binary_string = ''.join(reversed(binary_string))
        # Convert each character in the string to an integer
        binary_array = [int(digit) for digit in reversed_binary_string]
        # Pad the array with 0's to ensure it's always 4 elements long
        while len(binary_array) < 4:
            binary_array.append(0)
        return binary_array

    def read(self):
        adc_value = self.adc.read_u16()
        # Reset the switch state
        self.dip_switch_state = [0,0,0,0]
        self.decimal = 0
        # Determine which threshold the voltage is closest to
        thresholds = [300, 7400, 12600, 17500, 22200, 25200, 28000, 30500, 33000, 35000, 36600, 38000, 39600, 41000, 42000]
    


        # Determine which threshold the voltage is closest to
        for i, threshold in enumerate(thresholds):
            if adc_value < threshold:
                self.decimal = i
                self.dip_switch_state = self.decimal_to_binary_array(i)
                break 
        
       
        return self.dip_switch_state

    def update(self):
        # This method will read and update the switch states
        self.read()

    
def update():
    # Record the start time
    start_time = time.time()
    
    # Update all sensors
    for sensor in sensorsInitialized:
        sensor.update()

    



    if uart0.any():
        data0 = uart0.read()
        print(data0)
    if uart1.any():
        data1 = uart1.read()
        print(data1)



    # Record the end time
    end_time = time.time()
    
    # Calculate the time taken for the loop
    elapsed_time = end_time - start_time
    
    # Calculate the remaining time to sleep to maintain the desired frame rate
    sleep_time = loop_duration - elapsed_time
    
    # Sleep only if there's remaining time
    if sleep_time > 0:
        time.sleep(sleep_time)

def init(boardType=None):
    global servoInputToGPIOPin, motorPinMap


    print("setting up board")
    if boardType == "GeneralBoard":
        servoInputToGPIOPin = [None, 21,20,19,20,21]
        motorPinMap = [None,[6,7],[8,9],[10,11],[12,13],[14,15],[21,22]]
    elif boardType == "SensorBoard":
        servoInputToGPIOPin = [None, 2,3,4,5,6,7,8]
    else:
        raise Exception("Invalid Board Type. In Init need to pass board type as string") 
    
    #Init Motors
    motors.append(None)
    for i in range(1,6):
        motors.append(Motor(i))



