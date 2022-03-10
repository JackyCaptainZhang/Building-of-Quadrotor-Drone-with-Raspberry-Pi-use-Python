import smbus
import math
import time

power_registor_address = 0x6b
bus = smbus.SMBus(1)
sensor_address = 0x68

def fetch_data(address):
    high = bus.read_byte_data(sensor_address, address)
    low = bus.read_byte_data(sensor_address, address + 1)
    data = (high << 8) + 8
    return data

def fetch_real_value(address):
    value = fetch_data(address)
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

def loop():
    while True:
        gyroX = fetch_real_value(0x43)
        gyroY = fetch_real_value(0x45)
        gyroZ = fetch_real_value(0x47)
        print("-------------- data from gyroscope ---------------")
        print("Raw gyroX data" , gyroX , "Processed rad rotation in X per second" , gyroX/131)
        print("Raw gyroY data" , gyroY , "Processed rad rotation in Y per second" , gyroY/131)
        print("Raw gyroZ data" , gyroZ , "Processed rad rotation in Z per second" , gyroZ/131)
        accX = fetch_real_value(0x3b) * 9.8
        accY = fetch_real_value(0x3d) * 9.8
        accZ = fetch_real_value(0x3f) * 9.8
        print("-------------- data from accelerator ---------------")
        print("Raw accX data" , accX , "Processed acceleration in X" , accX/16384)
        print("Raw accY data" , accY , "Processed acceleration in Y" , accY/16384)
        print("Raw accZ data" , accZ , "Processed acceleration in Z" , accZ/16384)
        print("-------------- data from temperature ---------------")
        temp = fetch_real_value(0x41)
        print("Raw temperature data" , temp , "Processed temperature" , temp/340 + 36.53)
        print("-------------- current rotational degrees ---------------")
        print("rotation in X" , fetch_rotation_X(accX/16384 , accY/16384 , accZ/16384))
        print("rotation in Y" , fetch_rotation_Y(accX/16384 , accY/16384 , accZ/16384))
        time.sleep(1)


bus.write_byte_data(sensor_address , power_registor_address , 0)
loop()

    
