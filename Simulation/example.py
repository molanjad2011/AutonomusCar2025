"""
@ 2023, Copyright AVIS Engine
- An Example Compatible with AVISEngine version 2.0.1 / 1.2.4 (ACL Branch) or higher
"""

import avisengine
import config
import time
import cv2
import numpy as np

# Creating an instance of the Car class
car = avisengine.Car()

# Connecting to the server (Simulator)
car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

# Counter variable
counter = 0

debug_mode = True

# Sleep for 3 seconds to make sure that client connected to the simulator
time.sleep(3)


def bird_eye_view_transform(image, src_points, dst_points, output_size):
    """
    این تابع تصویر ورودی را با استفاده از ماتریس هموگرافی به نمای بالای سر تبدیل می‌کند.
    
    پارامترها:
    - image: تصویر ورودی (BGR)
    - src_points: نقاط منبع (مثلاً نقاط کلیدی روی جاده در تصویر اصلی) به صورت آرایه numpy float32
    - dst_points: نقاط مقصد (مختصات مورد نظر در تصویر نمای بالای سر) به صورت آرایه numpy float32
    - output_size: سایز خروجی به صورت (عرض, ارتفاع)
    
    خروجی:
    - warped: تصویر تبدیل‌شده به نمای بالای سر
    """
    # محاسبه ماتریس هموگرافی
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    # اعمال تبدیل پرسپکتیو
    warped = cv2.warpPerspective(image, M, output_size)
    return warped


def detect_edges(image, low_threshold=50, high_threshold=150):
    """
    تصویر ورودی را به خاکستری تبدیل، به کمک GaussianBlur نویز را کاهش داده و از Canny برای استخراج لبه‌ها استفاده می‌کند.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 3)
    edges = cv2.Canny(blurred, low_threshold, high_threshold)
    return edges

def detect_lines_hough(edges, rho=1, theta=np.pi/180, threshold=50, min_line_length=100, max_line_gap=50):
    """
    از Hough Transform برای تشخیص خطوط در تصویر باینری استفاده می‌کند.
    خروجی لیستی از خطوط به‌صورت [rho, theta] است.
    """
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    line_params = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # محاسبه زاویه خط
            theta_line = np.arctan2((y2 - y1), (x2 - x1))
            # محاسبه rho به صورت: rho = x*cos(theta) + y*sin(theta)
            rho_line = x1 * np.cos(theta_line) + y1 * np.sin(theta_line)
            line_params.append([rho_line, theta_line])
    return line_params

try:
    while True:
        # Counting the loops
        counter = counter + 1

        # Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        car.setSpeed(100)

        # Set the Steering of the car -10 degree from center, results the car to steer to the left
        car.setSteering(0)

        # Set the angle between sensor rays to 45 degrees, Use this only if you want to set it from python client
        # Notice: Once it is set from the client, it cannot be changed using the GUI
        car.setSensorAngle(25)

        # Get the data. Need to call it every time getting image and sensor data
        car.getData()

        # Start getting image and sensor data after 4 loops
        if counter > 4:

            # Returns a list with three items which the 1st one is Left sensor data\
            # the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors()

            l, m, r = sensors


            if l < 900 or m < 600 or r < 900:
                car.setSpeed(-50)
                car.setSteering(180)
            else:
                car.setSpeed(100)

            if m < 900:
                car.setSteering(-180)
            else:
                car.setSteering(0)
                

            # Returns an opencv image type array. if you use PIL you need to invert the color channels.
            image = car.getImage() 
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            height, width = gray_image.shape

            src_points  = np.float32([
                [(width // 2 - 90),(height // 2) - 40],
                [(width // 2 + 85), (height // 2) - 40],
                [width - 10 , height - 140],
                [10,height - 120]
            ])

            dst_points = np.float32([
                [0, 0],
                [width, 0],
                [width, height],
                [0, height]
            ])

            bev_image = bird_eye_view_transform(gray_image, src_points, dst_points, (width, height))
            
            cv2.imshow("Bird eye view", bev_image)

            finImage = cv2.GaussianBlur(bev_image, (7, 7), 3)

            edge = cv2.Canny(finImage, 50, 150)

            poly = np.array([[
                (10,height - 120),
                ((width // 2) - 90,(height // 2) - 40),
                ((width // 2) + 85, (height // 2) - 40),
                (width - 10 , height - 140)
            ]])
            


            mask = np.zeros_like(gray_image)
            mask = cv2.fillPoly(mask, poly, 255)
            cv2.imshow("mask", mask)
            mask = cv2.bitwise_and(gray_image, mask)
            cv2.imshow("mask fin", gray_image)



            hough = cv2.HoughLinesP(
                edge,
                rho=1,
                theta=np.pi / 180,
                threshold=50,
                lines=np.array([]),
                minLineLength=10,
                maxLineGap=10
            )



            left_lines = []
            right_lines = []

            linescount = 0
            if hough is not None:
                for lines in hough:
                    linescount += 1
                    x1, y1, x2, y2 = lines.reshape(4)
                
                    # محاسبه شیب و عرض از مبدأ
                    parameters = np.polyfit((x1, x2), (y1, y2), 1)
                    slope = parameters[0]
                    intercept = parameters[1]

                    if abs(slope) > 0.05:  # حذف خطوط افقی یا تقریباً افقی
                        if slope < 0:
                            left_lines.extend([(slope, intercept)] * 2)  # افزایش وزن خطوط چپ
                            cv2.line(bev_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        else:
                            right_lines.append((slope, intercept))
                            cv2.line(bev_image, (x1, y1), (x2, y2), (0, 0, 255), 3)


                    for x1, y1, x2, y2 in lines:
                        # print(x1,x2)
                        cv2.line(edge, (x1, y1), (x2, y2), (255, 255, 255), 3)
                print("Count of lines is : ", linescount)

            left_line_avg = np.average(left_lines, axis=0) if left_lines else (0, 0)
            right_line_avg = np.average(right_lines, axis=0) if right_lines else (0, 0)

            avg_slope = np.mean([right_line_avg, left_line_avg])
            steeering_angle = np.arctan(avg_slope) * (180/ np.pi)

            def make_coordinates(image, line_parameters):
                slope, intercept = line_parameters
                y1 = image.shape[0]  # انتهای تصویر (کف جاده)
                y2 = int(y1 * 0.6)  # کمی بالاتر از کف جاده
            
                if slope == 0:  # جلوگیری از تقسیم بر صفر
                    return None
            
                x1 = int((y1 - intercept) / slope)
                x2 = int((y2 - intercept) / slope)
            
                return np.array([x1, y1, x2, y2])
            
            left_line = make_coordinates(edge, left_line_avg)
            # right_line = make_coordinates(edge, right_line_avg)

            # cv2.line(image, (left_line[0], left_line[1]), (left_line[2], left_line[3]), (255, 0, 0), 5)
            # cv2.line(image, (right_line[0], right_line[1]), (right_line[2], right_line[3]), (255, 0, 0), 5)

            if abs(steeering_angle) > 10:
                car.setSteering(int(steeering_angle))

            cv2.imshow("edge", edge)

            

            # Returns an integer which is the real time car speed in KMH
            carSpeed = car.getSpeed()

            if debug_mode:
                print(f"Speed : {carSpeed}")
                print(
                    f"Left : {str(sensors[0])} | Middle : {str(sensors[1])} | Right : {str(sensors[2])}"
                )

            # Showing the opencv type image
            

            if cv2.waitKey(10) == ord("q"):
                break

            time.sleep(0.001)

finally:
    car.stop()
