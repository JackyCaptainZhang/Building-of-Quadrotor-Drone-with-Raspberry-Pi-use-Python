import RPi.GPIO as gpio
import time as time

trigger_pin = 17
echo_pin = 27
Max_distance = 210
timeout = Max_distance * 58.8


def find_pulse_time(pin, level, timeout):
    t0 = time.time()
    while(gpio.input(pin) != level):
        if((time.time() - t0) > timeout * 0.000001):
            return 0;
    t0 = time.time()
    while(gpio.input(pin) == level):
        if((time.time() - t0) > timeout * 0.000001):
            return 0;
    pulsetime = (time.time() - t0) * 1000000
    return pulsetime
    
def get_distance_from_sonic():
    gpio.output(trigger_pin , gpio.HIGH)
    time.sleep(0.00001)
    gpio.output(trigger_pin , gpio.LOW)
    pulse_time = find_pulse_time(echo_pin, gpio.HIGH, timeout)
    distance = pulse_time * 0.017  # 340/2/10000
    return distance
    
def setup():
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)      
    gpio.setup(trigger_pin, gpio.OUT)   
    gpio.setup(echo_pin, gpio.IN) 

def loop():
    while(True):
        distance = get_distance_from_sonic() 
        print ("The distance is :" , int(distance * 10)/10 , 'cm')
        time.sleep(0.35)

 
print ('Program is starting...')
setup()
try:
    loop()
except KeyboardInterrupt:  
    gpio.cleanup()