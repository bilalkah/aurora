# Import libraries
import RPi.GPIO as GPIO
import time

class myServo:
    def __init__(self, pin=11, freq=50):
        self.pin = pin
        self.freq = freq
        self.duty = 2
        self.setup()
    
    def setup(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.freq)
        self.pwm.start(0)
    
    def change_duty(self, us):
        self.us = us
        self.duty = 2+(us/18)
        self.pwm.ChangeDutyCycle(self.duty)
        time.sleep(2)

    def change_freq(self, freq):
        self.freq = freq
        self.pwm.ChangeFrequency(freq)
    
    def change_range(self, min_us, max_us):
        self.min_us = min_us
        self.max_us = max_us
        self.change_duty(self.us)
        
    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()
    
    def __del__(self):
        self.cleanup()

"""        
if __name__ == "__main__":
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

# Set pin 11 as an output, and set servo1 as pin 11 as PWM
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50) # Note 11 is pin, 50 = 50Hz pulse

#start PWM running, but with value of 0 (pulse off)
servo1.start(0)
print ("Waiting for 2 seconds")
time.sleep(2)

#Let's move the servo!
print ("Rotating 180 degrees in 10 steps")

# Define variable duty
duty = 2

for i in range(10):
    servo1.ChangeDutyCycle(i+2)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    time.sleep(0.7)
for i in range(9):
    servo1.ChangeDutyCycle(11-i)
    time.sleep(0.3)
    servo1.ChangeDutyCycle(0)
    time.sleep(0.7)

try:
    while True:
        #Ask user for angle and turn servo to it
        angle = float(input('Enter angle between 0 & 180: '))
        servo1.ChangeDutyCycle(2+(angle/18))
        time.sleep(0.5)
        servo1.ChangeDutyCycle(0)
finally:
    #Clean things up at the end
    servo1.stop()
    GPIO.cleanup()
    print("Goodbye!")"""