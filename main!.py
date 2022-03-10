import RPi.GPIO as gpio
import time as time
import threading
import pigpio
import smbus
import math
import matplotlib.pyplot as plt
import numpy as np

trigger_pin = 17
echo_pin = 27
Max_distance = 210
timeout = Max_distance * 58.8
motor1_pin = 23
motor2_pin = 24
motor3_pin = 26
motor4_pin = 16
pi = pigpio.pi()
power_registor_address = 0x6b
bus = smbus.SMBus(1)
sensor_address = 0x68
height_initial = 0
throttle1 = 0
throttle2 = 0
throttle3 = 0
throttle4 = 0
P_h, I_h, I_h_Max, D_h, err_h, last_err_h = 0,0,80,0,0,0  # height index
P_p, I_p, I_p_Max, D_p, err_p, last_err_p = 0,0,80,0,0,0  # pitch index
P_r, I_r, I_r_Max, D_r, err_r, last_err_r = 0,0,80,0,0,0  # roll index


class ultrasonic:
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
        
    def setup():
        global height_initial
        gpio.setwarnings(False)
        gpio.setmode(gpio.BCM)      
        gpio.setup(trigger_pin, gpio.OUT)   
        gpio.setup(echo_pin, gpio.IN)
        height_initial = ultrasonic.get_initial_distance_from_sonic()
           
    def get_initial_distance_from_sonic():
        gpio.output(trigger_pin , gpio.HIGH)
        time.sleep(0.00001)
        gpio.output(trigger_pin , gpio.LOW)
        pulse_time = ultrasonic.find_pulse_time(echo_pin, gpio.HIGH, timeout)
        distance = pulse_time * 0.017  
        return (int(distance * 10)/10)
    
    def get_distance_from_sonic():
        gpio.output(trigger_pin , gpio.HIGH)
        time.sleep(0.00001)
        gpio.output(trigger_pin , gpio.LOW)
        pulse_time = ultrasonic.find_pulse_time(echo_pin, gpio.HIGH, timeout)
        distance = pulse_time * 0.017  
        return int((distance - height_initial) * 10/10)

class MPU6050:
    def fetch_data(address):
        high = bus.read_byte_data(sensor_address, address)
        low = bus.read_byte_data(sensor_address, address + 1)
        data = (high << 8) + 8
        return data

    def fetch_real_value(address):
        value = MPU6050.fetch_data(address)
        x = 0xffff
        if(value >= 0x8000):
            return -((x - value) + 1)
        else:
            return value

    def fetch_rotation_X(x , y , z):
        rad = math.atan2(y, math.sqrt((x * x) + (z * z)))
        return math.degrees(rad)

    def fetch_rotation_Y(x , y , z):
        rad = math.atan2(x, math.sqrt((y * y) + (z * z)))
        return math.degrees(rad)

    def fetch_rotations():
            accX = MPU6050.fetch_real_value(0x3b) * 9.80 
            accY = MPU6050.fetch_real_value(0x3d) * 9.80 
            accZ = MPU6050.fetch_real_value(0x3f) * 9.80 
            x = int(-MPU6050.fetch_rotation_Y(accX/16384 , accY/16384 , accZ/16384) *10)/10
            y = int(MPU6050.fetch_rotation_X(accX/16384 , accY/16384 , accZ/16384) *10)/10
            return x,y

class motor:
    def Setup_Calibration (motor_pin, pwm_freq, high, low):
        pi.write(motor_pin, 0)
        pi.set_PWM_frequency(motor_pin, pwm_freq)
        pi.set_PWM_range(motor_pin, 100)
        pi.set_PWM_dutycycle(motor_pin, high)
        time.sleep(3)
        print("high throttle calibration finish")
        pi.set_PWM_dutycycle(motor_pin, low)
        time.sleep(3)
        print("low throttle calibration finish")
    
    def four_motor_initialisation(pwm_freq, high, low):
        threads = []
        thread1 = threading.Thread(target= motor.Setup_Calibration, args=(motor1_pin, pwm_freq, high, low))
        thread2 = threading.Thread(target= motor.Setup_Calibration, args=(motor2_pin, pwm_freq, high, low))
        thread3 = threading.Thread(target= motor.Setup_Calibration, args=(motor3_pin, pwm_freq, high, low))
        thread4 = threading.Thread(target= motor.Setup_Calibration, args=(motor4_pin, pwm_freq, high, low))
        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()
        threads.append(thread1)
        threads.append(thread2)
        threads.append(thread3)
        threads.append(thread4)
        for t in threads:
            t.join()
        print ("---------------------------------------------")
        print ("Four motors' setup and calibration finished!")
        print ("---------------------------------------------")
        print (" Prepare to take off!!!")
        print ("---------------------------------------------")

    def stop_motors():
        pi.write(motor1_pin, 0)
        pi.write(motor2_pin, 0)
        pi.write(motor3_pin, 0)
        pi.write(motor4_pin, 0)

    def vertical(vertical_throttle):
        global throttle1, throttle2, throttle3, throttle4
        if(vertical_throttle >= 95):
            vertical_throttle =95
        threads = []
        thread1 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor1_pin, vertical_throttle))
        thread2 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor2_pin, vertical_throttle))
        thread3 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor3_pin, vertical_throttle))
        thread4 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor4_pin, vertical_throttle))
        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()
        threads.append(thread1)
        threads.append(thread2)
        threads.append(thread3)
        threads.append(thread4)
        for t in threads:
            t.join()
        throttle1 = vertical_throttle
        throttle2 = vertical_throttle
        throttle3 = vertical_throttle
        throttle4 = vertical_throttle
        #print(throttle1, throttle2, throttle3, throttle4)
    
    def pitch(pitch_throttle):
        global throttle1, throttle2, throttle3, throttle4
        if (pitch_throttle >= 0):
            a = throttle2 + pitch_throttle
            b = throttle3 + pitch_throttle
            if(a >= 95):
                a = 95
            if(b >= 95):
                b = 95
            threads = []
            thread5 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor2_pin, (a)))
            thread6 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor3_pin, (b)))
            thread5.start()
            thread6.start()
            threads.append(thread5)
            threads.append(thread6)
            throttle2 = a
            throttle3 = b
        if (pitch_throttle < 0):
            c = throttle1 - pitch_throttle
            d = throttle4 - pitch_throttle
            if(c >= 95):
                c = 95
            if(d >= 95):
                d = 95
            threads = []
            thread5 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor1_pin, (c)))
            thread6 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor4_pin, (d)))
            thread5.start()
            thread6.start()
            threads.append(thread5)
            threads.append(thread6)
            throttle1 = c
            throttle4 = d
        for t in threads:
            t.join()
        #print(throttle1, throttle2, throttle3, throttle4)
        
    def roll(roll_throttle):
        global throttle1, throttle2, throttle3, throttle4
        e = throttle4 + roll_throttle
        f = throttle3 + roll_throttle
        g = throttle1 - roll_throttle
        h = throttle2 - roll_throttle
        if (roll_throttle >= 0):
            if( e >= 95):
                e = 95
            if(f >= 95):
                f = 95
            threads = []
            thread7 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor4_pin, (e)))
            thread8 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor3_pin, (f)))
            thread7.start()
            thread8.start()
            threads.append(thread7)
            threads.append(thread8)
            throttle3 = e
            throttle4 = f
        if (roll_throttle < 0):
            if(g >= 95):
                g = 95
            if(h >= 95):
                h = 95
            threads = []
            thread7 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor1_pin, (g)))
            thread8 = threading.Thread(target= pi.set_PWM_dutycycle, args=(motor2_pin, (h)))
            thread7.start()
            thread8.start()
            threads.append(thread7)
            threads.append(thread8)
            throttle1 = g
            throttle2 = h
        for t in threads:
            t.join()
        #print(throttle1, throttle2, throttle3, throttle4)

            
class PIDs:
    def PID_in_height(target, kp, ki, kd):
        global  P_h, I_h, I_h_Max, D_h, last_err_h, err_h
        err_h = target - ultrasonic.get_distance_from_sonic()
        P_h = err_h * kp
        I_h += err_h * ki * 0.2
        if(I_h >= I_h_Max):
            I_h = I_h_Max
        if(I_h <= -I_h_Max):
            I_h = -I_h_Max 
        D_h = (err_h - last_err_h) * kd / 0.2
        last_err_h = err_h
        #print(P_h + I_h + D_h, ultrasonic.get_distance_from_sonic()) 
        return P_h + I_h + D_h
    
    def height_control(target, kp, ki, kd):
        PIDh = PIDs.PID_in_height(target, kp, ki, kd)
        motor.vertical(40 + PIDh/3) 
        time.sleep(0.01)
        
    def PID_in_pitch(target, kp, ki, kd):
        global P_p, I_p, I_p_Max, D_p, err_p, last_err_p
        err_p = target - MPU6050.fetch_rotations()[0]
        P_p = err_p * kp
        I_p += err_p * ki * 0.2
        if(I_p >= I_p_Max):
            I_p = I_p_Max
        if(I_p <= -I_p_Max):
            I_p = -I_p_Max
        D_p = (err_p - last_err_p) * kd / 0.2
        last_err_p = err_p
        #print((P_p + I_p + D_p)/3, MPU6050.fetch_rotations()[0]) 
        return P_p + I_p + D_p
        
    def pitch_control(target, kp, ki, kd):
        PIDp = PIDs.PID_in_pitch(target, kp, ki, kd)
        if(ultrasonic.get_distance_from_sonic() > 5):
            motor.pitch(PIDp/3)
            time.sleep(0.01)
        
    def PID_in_roll(target, kp, ki, kd):
        global P_r, I_r, I_r_Max, D_r, err_r, last_err_r
        err_r = target - MPU6050.fetch_rotations()[1]
        P_r = err_r * kp
        I_r += err_r * ki * 0.2
        if(I_r >= I_r_Max):
            I_r = I_r_Max
        if(I_r <= -I_r_Max):
            I_r = -I_r_Max
        D_r = (err_r - last_err_r) * kd / 0.2
        last_err_r = err_r
        #print((P_r + I_r + D_r)/3, MPU6050.fetch_rotations()[1]) 
        return P_r + I_r + D_r  
        
    def roll_control(target, kp, ki, kd):
        PIDr = PIDs.PID_in_roll(target, kp, ki, kd)
        if(ultrasonic.get_distance_from_sonic() > 5):
            motor.roll(PIDr/3)
            time.sleep(0.01)
      
    
def drone1():
    print('DRONE 1 PID Height ON !!')
    time.sleep(1)
    ultrasonic.setup()
    motor.four_motor_initialisation(400,95,40)  
    time.sleep(2)
    global throttle1, throttle2, throttle3, throttle4
    plt.ion()
    plt.figure()
    t = [0]
    t_now = 0
    T1 = [throttle1]
    T2 = [throttle2]
    T3 = [throttle3]
    T4 = [throttle4]
    plt.plot(t, T1, label = 'Throttle 1', color = 'red')
    plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
    plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
    plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
    plt.legend(loc = 'upper left')
    while True:
        t_now += 1
        PIDs.height_control(40, 1, 0.3, 0.1)  # ADJUST PID value!!
        t.append(t_now)
        T1.append(throttle1)
        T2.append(throttle2)
        T3.append(throttle3)
        T4.append(throttle4)
        plt.plot(t, T1, label = 'Throttle 1', color = 'red')
        plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
        plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
        plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
        plt.pause(0.01)
        if(MPU6050.fetch_rotations()[1] >= 75):
            break
    motor.stop_motors()
    print('GPIO Cleanup!')
    
def drone2():
    print('DRONE 2 PIDs Height and Pitch ON !!')
    time.sleep(1)
    ultrasonic.setup()  
    time.sleep(1)
    global throttle1, throttle2, throttle3, throttle4
    plt.ion()
    plt.figure()
    t = [0]
    t_now = 0
    T1 = [throttle1]
    T2 = [throttle2]
    T3 = [throttle3]
    T4 = [throttle4]
    plt.plot(t, T1, label = 'Throttle 1', color = 'red')
    plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
    plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
    plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
    plt.legend(loc = 'upper left')
    while True:
        t_now += 1
        PIDs.height_control(40, 1, 0.3, 0.1)  # ADJUST PID value!!
        PIDs.pitch_control(0, 0.5, 0.2, 0.1) # ADJUST PID value!!
        t.append(t_now)
        T1.append(throttle1)
        T2.append(throttle2)
        T3.append(throttle3)
        T4.append(throttle4)
        plt.plot(t, T1, label = 'Throttle 1', color = 'red')
        plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
        plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
        plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
        plt.pause(0.01)
        if(MPU6050.fetch_rotations()[1] >= 75):
            break
    motor.stop_motors()
    print('GPIO Cleanup!')

def drone3():
    print('DRONE 3 PIDs Height and Roll ON !!')
    time.sleep(1)
    ultrasonic.setup()  
    time.sleep(1)
    global throttle1, throttle2, throttle3, throttle4
    plt.ion()
    plt.figure()
    t = [0]
    t_now = 0
    T1 = [throttle1]
    T2 = [throttle2]
    T3 = [throttle3]
    T4 = [throttle4]
    plt.plot(t, T1, label = 'Throttle 1', color = 'red')
    plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
    plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
    plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
    plt.legend(loc = 'upper left')
    while True:
        t_now += 1
        PIDs.height_control(40, 1, 0.3, 0.1)  # ADJUST PID value!!
        PIDs.roll_control(0, 0.5, 0.2, 0.1)
        t.append(t_now)
        T1.append(throttle1)
        T2.append(throttle2)
        T3.append(throttle3)
        T4.append(throttle4)
        plt.plot(t, T1, label = 'Throttle 1', color = 'red')
        plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
        plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
        plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
        plt.pause(0.01)
        if(MPU6050.fetch_rotations()[1] >= 75):
            break
    motor.stop_motors()
    print('GPIO Cleanup!')
    
def drone4():
    print('DRONE 4 ALL PIDs ON !!')
    time.sleep(1)
    ultrasonic.setup() 
    time.sleep(1)
    global throttle1, throttle2, throttle3, throttle4
    plt.ion()
    plt.figure()
    t = [0]
    t_now = 0
    T1 = [throttle1]
    T2 = [throttle2]
    T3 = [throttle3]
    T4 = [throttle4]
    plt.plot(t, T1, label = 'Throttle 1', color = 'red')
    plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
    plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
    plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
    plt.legend(loc = 'upper left')
    while True:
        t_now += 1
        PIDs.height_control(40, 1, 0.3, 0.1)  # ADJUST PID value!!
        PIDs.pitch_control(0, 0.5, 0.2, 0.1) # ADJUST PID value!!
        PIDs.roll_control(0, 0.5, 0.2, 0.1)
        t.append(t_now)
        T1.append(throttle1)
        T2.append(throttle2)
        T3.append(throttle3)
        T4.append(throttle4)
        plt.plot(t, T1, label = 'Throttle 1', color = 'red')
        plt.plot(t, T2, '-r', label = 'Throttle 2', color = 'green')
        plt.plot(t, T3, '-r', label = 'Throttle 3', color = 'blue')
        plt.plot(t, T4, '-r', label = 'Throttle 4', color = 'purple')
        plt.pause(0.01)
        if(MPU6050.fetch_rotations()[1] >= 75):
            break
    motor.stop_motors()
    print('GPIO Cleanup!')
        
try:
    drone1()   # Height control and calibration
    #drone2()   # Height and pitch control
    #drone3()   # Height and roll control
    #drone4()   # Complete control

except KeyboardInterrupt:
    motor.stop_motors()
    print('GPIO Cleanup!')
    
    
        
    
    