## Cockipit to monitoring Serial Data from 
## Arduino Connected with SparkFun LSM9DS1 and GPS NEO-6M
## Serial data contains: 
## Complementary filter result from Accelerometer, Gyroscope and Magnetometer;
## GPS Data converted to X-axis and Y-axis relative of vehicle initial position;
## Kalman Filter result with X-axis and Y-axis relative of vehicle initial position;

import numpy as np
import matplotlib.pyplot as plt

plt.style.use('seaborn-white')
plt.rcParams.update({'font.size': 15})

with open('kalman.txt') as f:
    lines = f.readlines()
    gpsX = [line.split(',')[0] for line in lines]
    gpsY = [line.split(',')[1] for line in lines]
    gpsZ = [line.split(',')[2] for line in lines]
    kalmanX = [line.split(',')[3] for line in lines]
    kalmanY = [line.split(',')[4] for line in lines]
    kalmanZ = [line.split(',')[5] for line in lines]
    biasX = [float(line.split(',')[6])/100.0 for line in lines]
    biasY = [float(line.split(',')[7])/100.0 for line in lines]
    biasZ = [line.split(',')[8] for line in lines]
    time = [line.split(',')[9] for line in lines]
    
    gpsX_ = gpsX
    gpsY_ = gpsY
    gpsZ_ = gpsZ
    kalmanX_ = kalmanX
    kalmanY_ = kalmanY
    kalmanZ_ = kalmanZ
    biasX_ = biasX
    biasY_ = biasY
    biasZ_ = biasZ
    time_ = time

new_strings = []
for string in time_:
    new_string = string.replace("\n", "")
    new_strings.append(new_string)

results = list(map(int, new_strings))
gpsX_ = list(map(float, gpsX_))
gpsY_ = list(map(float, gpsY_))
gpsZ_ = list(map(float, gpsZ_))
kalmanX_ = list(map(float, kalmanX_))
kalmanY_ = list(map(float, kalmanY_))
biasX_ = list(map(float, biasX_))
biasY_ = list(map(float, biasY_))
biasZ_ = list(map(float, biasZ_))

if(max(gpsX_) > max(kalmanX_)):
    max = max(gpsX_)
else:
    max = max(list(map(float, gpsX)))

figure = plt.figure()

plt.subplot(2,2,1)
plt.title('Kalman Filter (IMU + GPS Data Fusion) - X axis', fontname="Times New Roman Bold", weight='bold')                                             
plt.ylabel('X (m)')
plt.xlabel('Time (s)')                    
plt.plot(np.subtract(gpsX_,kalmanX_))           
plt.legend(loc='upper right')


plt.subplot(2,2,2)
plt.title('Kalman Filter (IMU + GPS Data Fusion) - Y axis', fontname="Times New Roman Bold", weight='bold')                                             
plt.ylabel('Y (m)')
plt.xlabel('Time (s)')                    
plt.plot(np.subtract(gpsY_,kalmanY_))         

plt.subplot(2,2,3)
plt.title('Kalman Filter (IMU + GPS Data Fusion) - Z axis', fontname="Times New Roman Bold", weight='bold')                                             
plt.ylabel('Z (m)')
plt.xlabel('Time (s)')                       
plt.plot(gpsZ_, 'b2-', label='GPS')           
plt.legend(loc='upper right')                                              
plt.plot(kalmanZ_, 'm2-', label='Kalman Filter')  
plt.legend(loc='upper right') 
figure.subplots_adjust(hspace=0.4)

figure = plt.figure()
plt.locator_params(axis='y', numticks=6)  
plt.subplot(2,2,1)
plt.title('Bias - X axis', fontname="Times New Roman Bold", weight='bold')                          
plt.ylabel('Bias X-axis m/s$^2$')
plt.xlabel('Time (s)')                     
plt.plot(results,biasX_,  'c-')   
x = [0, 50, 100, 150, 200, 250]
plt.xticks(x)

plt.subplot(2,2,2)
plt.title('Bias - Y axis', weight='bold')   
plt.ylabel('Bias Y-axis m/s$^2$')
plt.xlabel('Time (s)')                              
plt.plot(results,biasY_, 'c-')    
x = [0, 50, 100, 150, 200, 250]
plt.xticks(x)

exp = lambda x: 's'**(2)
plt.subplot(2,2,3)
plt.title('Bias - Z axis', weight='bold')   
plt.ylabel('Bias Z-axis m/s$^2$')
plt.xlabel('Time (s)')                              
plt.plot(results, biasZ_, 'c-')       
x = [0, 50, 100, 150, 200, 250]
plt.xticks(x)

figure.subplots_adjust(hspace=0.4)

print(results)
plt.show()



