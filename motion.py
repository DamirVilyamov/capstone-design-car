import wiringpi
from time import sleep

IN1_PIN = 1 # LEFT FORWARD
IN2_PIN = 4 # LEFT BACKWARS
IN3_PIN = 5 # RIGHT FORWARD
IN4_PIN = 6 # RIGHT BACKWARD

INPUT = 0
OUTPUT = 1
#for more precise voltage control
SOFT_PWM_OUTPUT = 2

MAX_SPEED = 70
MIN_SPEED = 0

class MotorControl:
    
    def __init__(self):
        print("Motor init\n");
        if(wiringpi.wiringPiSetup() != -1):
            wiringpi.pinMode(IN1_PIN, SOFT_PWM_OUTPUT)
            wiringpi.pinMode(IN2_PIN, SOFT_PWM_OUTPUT)
            wiringpi.pinMode(IN3_PIN, SOFT_PWM_OUTPUT)
            wiringpi.pinMode(IN4_PIN, SOFT_PWM_OUTPUT)
                 
            #range of valuesfrom 0 to 100
            wiringpi.softPwmCreate(IN1_PIN, 0, 100)
            wiringpi.softPwmCreate(IN2_PIN, 0, 100)
            wiringpi.softPwmCreate(IN3_PIN, 0, 100)
            wiringpi.softPwmCreate(IN4_PIN, 0, 100)
            
        else:
            print('MOTOR INIT FAILED')
        
        
    def forward(self, power=1.0):
        print("FORWARD\n")
        wiringpi.softPwmWrite(IN1_PIN, int(MAX_SPEED * power * 0.8))
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, int(MAX_SPEED * power))
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)
        
    def back(self, power=1.0):
        print("BACKWARD\n")
        wiringpi.softPwmWrite(IN1_PIN, int(MIN_SPEED * 0.8))
        wiringpi.softPwmWrite(IN2_PIN, MAX_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN4_PIN, MAX_SPEED)
        
    def smoothLeft(self, power=1.0):
        print("SMOOTH LEFT")
        wiringpi.softPwmWrite(IN1_PIN, int(MAX_SPEED*0.3*power))
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, int(MAX_SPEED*1.2*power))
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)
        
    def smoothRight(self, power=1.0):
        print("SMOOTH RIGHT")
        wiringpi.softPwmWrite(IN1_PIN, int(MAX_SPEED*1.2*power))
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, int(MAX_SPEED*0.3*power))
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)

    def right(self, power=1.0):
        print("RIGHT")
        wiringpi.softPwmWrite(IN1_PIN, int(MAX_SPEED*power))
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)
        
    def left(self, power=1.0):
        print("LEFT")
        wiringpi.softPwmWrite(IN1_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, int(MAX_SPEED*power))
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)

    def stop(self):
        print("STOP\n");
        wiringpi.softPwmWrite(IN1_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN2_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN3_PIN, MIN_SPEED)
        wiringpi.softPwmWrite(IN4_PIN, MIN_SPEED)


move = MotorControl()
# move.smoothLeft()
# sleep(2)
# move.smoothRight()
# sleep(2)
move.stop()