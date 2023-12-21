import time
from ibus import IBus
from machine import Pin, PWM
import math

#Turns LED on
led = Pin(25, Pin.OUT)

led.toggle()

ibus_in = IBus(0)


import IRController as controller



import time
import IRController as controller


controller.init("GeneralBoard")
servo = controller.ServoMotor(1)

frontRightMotor = controller.Motor(2,flipDirection=True)
frontLeftMotor = controller.Motor(1,flipDirection=False)
backRightMotor = controller.Motor(4,flipDirection=False)
backLeftMotor = controller.Motor(3,flipDirection=True)


servo = controller.ServoMotor(1)


def activationFunction(x):
    #This function is used to adjust power to input
    #A Linear relation doesn't work very well
    #https://www.desmos.com/calculator/ijblikql2h
    
    if x < 0:
        return -((-x)**(1/3))
    else:
        return x**(1/3)

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
        turnInput = data[3]
        servoInput = data[4]

        servo.set_angle(servoInput*2.7)
        # print("forwardInput", forwardInput)
        # print("SideInput", sideInput)
        # print(angle)
        # print("Forward Amount", forwardAmount)
        # print("sideAmount Amount", sideAmount)
        rightSpeed = 0
        leftSpeed = 0
        
        turningSpeedMultiple = 0.8 # Slow down turning 
        
        if abs(forwardInput) < 20 and abs(sideInput)<20 and abs(turnInput )< 20:
            #Stop if stick near middle
            frontRightMotor.set_speed(0)
            frontLeftMotor.set_speed(0)
            backLeftMotor.set_speed(0)       
            backRightMotor.set_speed(0)
        elif abs(turnInput) > 50:
            
            if turnInput > 50:
                frontRightMotor.set_speed(-100*turningSpeedMultiple)
                frontLeftMotor.set_speed(100*turningSpeedMultiple)
                backLeftMotor.set_speed(100*turningSpeedMultiple)       
                backRightMotor.set_speed(-100*turningSpeedMultiple)
            elif turnInput < -50:
                frontRightMotor.set_speed(100*turningSpeedMultiple)
                frontLeftMotor.set_speed(-100*turningSpeedMultiple)
                backLeftMotor.set_speed(-100*turningSpeedMultiple)       
                backRightMotor.set_speed(100*turningSpeedMultiple)

        else:
            if sideInput < -70:

                #going left
                
                if forwardInput > 80:
                    #Forward Left
                    frontRightMotor.set_speed(100)
                    frontLeftMotor.set_speed(0)
                    backLeftMotor.set_speed(100)       
                    backRightMotor.set_speed(0)
                    
                elif forwardAmount < -80:
                    #Back Left
                    frontRightMotor.set_speed(0)
                    frontLeftMotor.set_speed(-100)
                    backLeftMotor.set_speed(-0)       
                    backRightMotor.set_speed(-100)
                else:
                    
                    frontLeftMotor.set_speed(-100)
                    backLeftMotor.set_speed(100)   
                    frontRightMotor.set_speed(100)    
                    backRightMotor.set_speed(-100)
            
            elif sideInput > 70:

                #going left
                
                if forwardInput > 80:
                    #Forward Left
                    frontRightMotor.set_speed(0)
                    frontLeftMotor.set_speed(100)
                    backLeftMotor.set_speed(-0)       
                    backRightMotor.set_speed(100)
                    
                elif forwardAmount < -80:
                    #Back Left
                    
                    frontRightMotor.set_speed(-100)
                    frontLeftMotor.set_speed(0)
                    backLeftMotor.set_speed(-100)       
                    backRightMotor.set_speed(0)
                else:
                    
                    frontLeftMotor.set_speed(100)
                    backLeftMotor.set_speed(-100)   
                    frontRightMotor.set_speed(-100)    
                    backRightMotor.set_speed(100)

            else:
                #Going forward or backward
                frontRightMotor.set_speed(forwardAmount)
                frontLeftMotor.set_speed(forwardAmount)
                backLeftMotor.set_speed(forwardAmount)       
                backRightMotor.set_speed(forwardAmount)

       
        


        
        
    else:
        pass
        #print ("Status offline {}".format(res[0]))
    















