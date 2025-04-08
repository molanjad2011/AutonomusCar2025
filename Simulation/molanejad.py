import avisengine
import config
import time
import cv2
import numpy as np 
# import argparse

# Creating an instance of the Car class
car = avisengine.Car()

# Connecting to the server (Simulator)
car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

# Counter variable
counter = 0

debug_mode = True

# Sleep for 3 seconds to make sure that client connected to the simulator 
time.sleep(3)

try:
    while(True):
        # Counting the loops
        counter = counter + 1

        # Get the data. Need to call it every time getting image and sensor data
        car.getData()

        # Start getting image and sensor data after 4 loops
        if(counter > 4):
            car.setSpeed(20)

            # Set the Steering of the car -10 degree from center, results the car to steer to the left
            car.setSteering(-10)
        
            # Set the angle between sensor rays to 45 degrees, Use this only if you want to set it from python client
            # Notice: Once it is set from the client, it cannot be changed using the GUI
            car.setSensorAngle(45) 

            # Returns a list with three items which the 1st one is Left sensor data\
            # the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors() 

            l , m , r = sensors
            if m < 400 : 
                car.setSpeed(-100)
                car.setSteering(0)
            else:
                car.setSteering(0)
                car.setSpeed(100)
            # Returns an opencv image type array. if you use PIL you need to invert the color channels.
            image = car.getImage()

            # Returns an integer which is the real time car speed in KMH
            carSpeed = car.getSpeed()

            if(debug_mode):
                print(f"Speed : {carSpeed}") 
                print(f'Left : {str(sensors[0])} | Middle : {str(sensors[1])} | Right : {str(sensors[2])}')

            gray = cv2.cvtColor(image , cv2.COLOR_BGR2GRAY)
            canny = cv2.GaussianBlur(gray ,(7,7), cv2.BORDER_DEFAULT)
            edge = cv2.Canny(canny, 100, 200)

            # Hough Line Transform
            lines = cv2.HoughLinesP(edge, 1, np.pi/180, 68, minLineLength=15, maxLineGap=250)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    
                    # Calculate the slope of the line
                    slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
                    
                    # Adjust steering based on the slope
                    if slope < -0.5:
                        car.setSteering(-10)  # Steer left
                    elif slope > 0.5:
                        car.setSteering(10)   # Steer right
                    else:
                        car.setSteering(0)    # Go straight

            cv2.imshow('frames', image)
            cv2.imshow("hough", edge)

            if cv2.waitKey(10) == ord('q'):
                break

            time.sleep(0.001)

finally:
    car.stop()
    cv2.destroyAllWindows()