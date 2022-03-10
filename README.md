# Building-of-Quadrotor-Drone-with-Raspberry-Pi-use-Python
This project build the basic drone PIDs stable control systems based on the raspberry pi 4B+ hardware together with a MPU6050 gyroscope, an ultrasonic sensor, four electric motor speed controllers and four brushless motors.


This project aims to keep the drone at the specific set height and keep postures stable all the time. To do this, a MPU6050 gyroscope and an ultrasonic sensor will be needed. The block doagram of the control system is as follows:
![image](https://user-images.githubusercontent.com/55009904/157682165-7ade7f13-b28d-413b-855a-f3f8b2df69dc.png)

You can see the **folder named "Basic Driver"**, which contains the **basic drivers for the motor control**, fetching raw data from MPU6050 gyroscope and the ultrasonic sensors. And as for the other **three files outside** the folder, the **"Calibrated MPU6050"** is to **convert the raw data we have fetched into the real needed data and in this project, is the rotations angles in X and Y axixs.** The **"Ultrasonic sensor"** is to test whether the height data is right or not. All files above are only for testing and learning purpose!

**The **"MAIN!"****  file is the most important part and contains all the things we needed for the drone and only run this file is enough for this project!!(**you can download the MAIN! file only and you can get all the things I have mentioned above!**) There are four classes in this file (four claases in one file beacuse the editor I used on the RPI do not support importing multiple customized class files....) : **Ultrasonic sensor, MPU6050, motor and PIDs. ** Ultrasonic sensor and MPU6050 provide the driver and data process.** **Motor provides the basic motor control driver and defines three basic motions for the drone: verticle(the height control), pitch(forward and backward control) and the roll(Left and right control)** {More actions may be defined in the future...} In the last **main session** you can see that there are four executive parts: Drone1 to Drone4. **Drone 1, 2 and 3 is for testing purpose and only turn on the partial PIDs. Drone4 turn on the complete PID control!!** 

**If you want to change the PID target value and parameters, simplly call the corresponded PID control methods and change the parameters in the bracket. You can see the name of each parameters clearly!!**

Running this file, you can see the figure is plotted real time. The Y axis is the percentage of throttle from 0 to 100. The X axis is the time.

You can change all the pins number at the beginning! They are all global variables.
The pin connections for my drone is as below. You can use the same pin connections for convenience. 
![image](https://user-images.githubusercontent.com/55009904/157687073-dcc0d62b-cf4f-4803-b776-f94a0f152ff0.png)

Enjoy and have a nice day!!

Any questions, just leave comments!  :))

Jacky Captain
