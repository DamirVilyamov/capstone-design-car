import wiringpi
# from time import sleep
import time

TRIG_PIN = 28 
ECHO_PIN = 29

class ObstacleDetector:
    def isObstacleInRange(self, minimalDistance):
        wiringpi.wiringPiSetup()
        wiringpi.pinMode(TRIG_PIN, 1)
        wiringpi.pinMode(ECHO_PIN, 0)

        start_time = 0
        end_time = 0

        wiringpi.digitalWrite(TRIG_PIN, 0)
        wiringpi.digitalWrite(TRIG_PIN, 1)
        # sleep(0.000010)
        wiringpi.digitalWrite(TRIG_PIN, 0)
        while(True):
            if(wiringpi.digitalRead(ECHO_PIN) != 0):
                break
            
        now = time.time()
        start_time = round(now * 1000000)
        while(True):
            if(wiringpi.digitalRead(ECHO_PIN) != 1):
                break
        now = time.time()
        end_time = round(now * 1000000)
        distance = ((end_time - start_time) /29.0) / 2.0
        return minimalDistance > distance


# obstacleDetector = ObstacleDetector()
# while(1):
#     print(obstacleDetector.isObstacleInRange(10))


# int getDistance() {
# 	int start_time=0, end_time=0; 
# 	float distance=0;
# 	digitalWrite(TRIG_PIN, LOW) ;
# 	digitalWrite(TRIG_PIN, HIGH) ; 
# 	delayMicroseconds(10);
# 	digitalWrite(TRIG_PIN, LOW) ;
# 	while (digitalRead(ECHO_PIN) == 0) ; 
# 	start_time = micros() ;
# 	while (digitalRead(ECHO_PIN) == 1) ; 
# 	end_time = micros() ;
# 	distance = (end_time - start_time) / 29. / 2. ; 
# 	return (int)distance;
# }

# import time
# now = time.time()

# later = time.time()
# difference = later - now
# print(difference)

# from gpiozero import DistanceSensor
# distance = DistanceSensor(echo=20, trigger=21).distance * 100
# print(distance)