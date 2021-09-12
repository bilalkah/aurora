from utils import *
from monitor import *
import time 
from math import ceil
import argparse  
from servo import *

if __name__ == "__main__":
    """# Arg parser for connecting to the vehicle
    parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
    parser.add_argument('--connect', 
                    help="Vehicle connection target string.")
    args = parser.parse_args()
    connection_string = args.connect


    # Vehicle connection
    vehicle = connectCopter(connection_string)"""
    """
    myThread = ThreadedVideoStream(vehicle = None, imwrite=True, imshow=True,livestream=True, s_address='127.0.0.1')
    myThread.setColor("blue")
    """
    servo = Servo()
    time.sleep(5)
    servo.change_duty(1500)
    time.sleep(5)
    servo.change_duty(1000)
    time.sleep(5)
    servo.change_duty(2000)
    time.sleep(5)
    """
    try:
        while True:
            print("Bekle")
            time.sleep(5)
            continue
                
    except KeyboardInterrupt:
        myThread.finish()
    """