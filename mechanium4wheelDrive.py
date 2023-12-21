import time
from ibus import IBus
from machine import Pin, PWM
import math

#Turns LED on
led = Pin(25, Pin.OUT)

led.toggle()

ibus_in = IBus(0)


import time
import IRController as controller


controller.init("GeneralBoard")


frontRightMotor = controller.Motor(1,flipDirection=False)
frontLeftMotor = controller.Motor(2,flipDirection=False)
backRightMotor = controller.Motor(3,flipDirection=False)
backLeftMotor = controller.Motor(4,flipDirection=True)

servo = controller.ServoMotor(1)


def activationFunction(x):
    #This function is used to adjust power to input
    #A Linear relation doesn't work very well
    #https://www.desmos.com/calculator/ijblikql2h
    
    if x < 0:
        return -((abs(x))**(1.5))
    else:
        return x**(1.5)

def getStickAngle(forwardAmount, sideAmount):
    #returns angle in degrees
    angle = math.degrees(math.atan2(forwardAmount,sideAmount))
    angle = angle%360
    return angle

def cos_degree(angle_degree):
    angle_rad = math.radians(angle_degree)
    return math.cos(angle_rad)

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
        

        forwardAmount = activationFunction(data[1]/100)*100
        sideAmount = activationFunction(data[0]/100)*100
        
        print(data)
        
        angle = getStickAngle(data[1],data[0])
        sideInput = data[0]
        forwardInput = data[1]
        turningInput = data[3]
        # print("forwardInput", forwardInput)
        # print("SideInput", sideInput)
        # print(angle)
        # print("Forward Amount", forwardAmount)
        # print("sideAmount Amount", sideAmount)
        frontRightSpeed = 0
        frontLeftSpeed = 0
        backRightSpeed = 0
        backLeftSpeed = 0
        
        
        turningSpeedMultiple = 0.6 # Slow down turning 

        #turning Code
        if abs(turningInput) > 30:
            #Turning 

            frontRightSpeed = -turningInput *turningSpeedMultiple
            frontLeftSpeed = turningInput*turningSpeedMultiple
            backRightSpeed = -turningInput*turningSpeedMultiple
            backLeftSpeed = turningInput*turningSpeedMultiple
        
            

        else:
            #moving
            if abs(forwardInput) < 20 and abs(sideInput)<20:
                #Stop if stick near middle
                frontRightSpeed = 0
                frontLeftSpeed = 0
                backRightSpeed = 0
                backLeftSpeed = 0
            else:
                if sideInput > 50:
                    

                    frontRightSpeed = sideInput 
                    frontLeftSpeed = -sideInput
                    backRightSpeed = -sideInput
                    backLeftSpeed = sideInput
                
                
                elif sideInput < -50:
                    frontRightSpeed = sideInput 
                    frontLeftSpeed = -sideInput
                    backRightSpeed = -sideInput
                    backLeftSpeed = sideInput

                else:
                    #Going forward or backward
                    
                    frontRightSpeed = forwardAmount
                    frontLeftSpeed = forwardAmount
                    backRightSpeed = forwardAmount
                    backLeftSpeed = forwardAmount

       
        frontRightMotor.set_speed(frontRightSpeed)
        frontLeftMotor.set_speed(frontLeftSpeed)
        backRightMotor.set_speed(backRightSpeed)
        backLeftMotor.set_speed(backLeftSpeed)
                
      

        
        
    else:
        pass
        #print ("Status offline {}".format(res[0]))
    

















