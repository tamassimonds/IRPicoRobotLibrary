import time
from ibus import IBus
from machine import Pin, PWM
import math

#Turns LED on
led = Pin(25, Pin.OUT)

led.toggle()

ibus_in = IBus(0)


import IRController as controller


controller.init("GeneralBoard")
servo = controller.ServoMotor(1)

motor1 = controller.Motor(1)
motor2 = controller.Motor(2)

import time
import IRController as controller


controller.init("GeneralBoard")
servo = controller.ServoMotor(1)

rightMotor = controller.Motor(1)
leftMotor = controller.Motor(2)

servo = controller.ServoMotor(1)


while True:
    res = ibus_in.read()
    # if signal then display immediately
    if (res[0] == 1):
        data = [IBus.normalize(res[1]),   
            IBus.normalize(res[2]),
            IBus.normalize(res[3]),
            IBus.normalize(res[4]),
            IBus.normalize(res[5], type="dial"),
            IBus.normalize(res[6], type="dial")]
#         print ("Status {} CH 1 {} Ch 2 {} Ch 3 {} Ch 4 {} Ch 5 {} Ch 6 {}".format(
#             res[0],    # Status
#             IBus.normalize(res[1]),   
#             IBus.normalize(res[2]),
#             IBus.normalize(res[3]),
#             IBus.normalize(res[4]),
#             IBus.normalize(res[5], type="dial"),
#             IBus.normalize(res[6], type="dial")),
#             end="")
#         print (" - {}".format(time.ticks_ms()))
        print(data)
#         print(data[2])
#         speed.freq(1000)
#         IN4.high()
#         IN3.low()
#         

        forwardAmount = data[1]
        sideAmount = data[0]

        servo.set_angle(data[4]*2.7)
        if sideAmount < -30:
            #going right
            rightMotor.set_speed(100)
            leftMotor.set_speed(-100)
            pass
        elif sideAmount > 30:
            #going left
            rightMotor.set_speed(-100)
            leftMotor.set_speed(100)

        else:
            print("Going forward")
            if forwardAmount > 30:
                #going forward
                rightMotor.set_speed(100)
                leftMotor.set_speed(100)
                pass
            elif forwardAmount < -30:
                #going backward
                rightMotor.set_speed(-100)
                leftMotor.set_speed(-100)


                
            else:
                #Stop motors
                rightMotor.set_speed(0)
                leftMotor.set_speed(0)

                
      

        
        
    else:
        pass
        #print ("Status offline {}".format(res[0]))
    









