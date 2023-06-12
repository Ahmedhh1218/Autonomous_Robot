import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
    
 
class Control():
    def __init__(self,EnaA,In1A,In2A,EnaB,In1B,In2B):
        self.EnaA = EnaA
        self.In1A = In1A
        self.In2A = In2A
        self.EnaB = EnaB
        self.In1B = In1B
        self.In2B = In2B
        
        GPIO.setup(self.EnaA,GPIO.OUT)
        GPIO.setup(self.In1A,GPIO.OUT)
        GPIO.setup(self.In2A,GPIO.OUT)
        GPIO.setup(self.EnaB,GPIO.OUT)
        GPIO.setup(self.In1B,GPIO.OUT)
        GPIO.setup(self.In2B,GPIO.OUT)
        
        self.pwmA = GPIO.PWM(self.EnaA, 100);
        self.pwmA.start(0);
        self.pwmB = GPIO.PWM(self.EnaB, 100);
        self.pwmB.start(0);
        
 
    def Drive(self,speed=0.5, turn=0):
        speed *=100
        driveSpeed = speed
        turn *=100
        turnSpeed = turn
        if driveSpeed>100: driveSpeed=100
        elif driveSpeed<-100: driveSpeed= -100
        if turnSpeed>100: turnSpeed=100
        elif turnSpeed<-100: turnSpeed= -100
        
        self.pwmA.ChangeDutyCycle(abs(driveSpeed))
        self.pwmB.ChangeDutyCycle(abs(turnSpeed))
        
        if driveSpeed>0:
            GPIO.output(self.In1A,GPIO.HIGH)
            GPIO.output(self.In2A,GPIO.LOW)

        else:
            GPIO.output(self.In1A,GPIO.LOW)
            GPIO.output(self.In2A,GPIO.HIGH)

        if turnSpeed>0:
            GPIO.output(self.In1B,GPIO.HIGH)
            GPIO.output(self.In2B,GPIO.LOW)
        else:
            GPIO.output(self.In1B,GPIO.LOW)
            GPIO.output(self.In2B,GPIO.HIGH)
 
    def Stop(self):
        self.pwmA.ChangeDutyCycle(0);
        self.pwmB.ChangeDutyCycle(0);
     
 
def main():
    motor.Stop()
    
 
if __name__ == '__main__':
    motor= Control(18,14,15,21,16,20)
    main()