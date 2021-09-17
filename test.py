from utils import *
from monitor import *
import time 
from math import ceil
import argparse  
#from servo import *

if __name__ == "__main__":
    """
    # Arg parser for connecting to the vehicle
    parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
    parser.add_argument('--connect', 
                    help="Vehicle connection target string.")
    args = parser.parse_args()
    connection_string = args.connect


    # Vehicle connection
    vehicle = connectCopter(connection_string)
    """
    myThread = ThreadedVideoStream(vehicle = vehicle, imwrite=False, livestream=True, s_address="192.168.43.233")
    myThread.setColor("red")
    
    
    
    try:
        while True:
            print("bekle")
            time.sleep(5)
                
    except KeyboardInterrupt:
        myThread.finish()
    
    