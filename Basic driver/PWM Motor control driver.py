import RPi.GPIO as gpio
import  time as time

pin = 26

gpio.setwarnings(False)
gpio.setmode(gpio.BCM) #  Take notice of gpio number type!! BCM or BOARD!!
gpio.setup(pin, gpio.OUT)
gpio.output(pin, gpio.LOW)

motor = gpio.PWM(pin, 400)
motor.start(0)

motor.ChangeDutyCycle(95)
time.sleep(3)
print("high throttle calibration finish");

motor.ChangeDutyCycle(40)
time.sleep(3);
print("low throttle calibration finish");

throttle = 40
for i in range(10):
    throttle += 1
    print (f"Throttle :  {throttle}")
    motor.ChangeDutyCycle(throttle)
    time.sleep(0.3);

motor.stop
gpio.cleanup





 