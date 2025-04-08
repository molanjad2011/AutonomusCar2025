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


def detect_lane_and_steer(image):
    """
    پردازش تصویر برای تشخیص خطوط جاده و تنظیم فرمان خودرو
    """

    # تبدیل تصویر به سطح خاکستری
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # اعمال Gaussian Blur برای کاهش نویز
    blurred = cv2.GaussianBlur(gray, (7, 7), 3)

    # استفاده از تشخیص لبه Canny
    edges = cv2.Canny(blurred, 50, 150)

    # تعریف ناحیه مورد نظر (ROI)
    height, width = edges.shape
    region = np.array([[(0, height-100), (width // 2 - 100  , (height // 2) - 28),
                        ((width // 2) + 50, (height // 2)-28), (width - 10, height-100)]], dtype=np.int32)

    mask = np.zeros_like(edges)
    cv2.imshow("mask",mask)
    masked = cv2.fillPoly(mask, region, 255)
    cv2.imshow("masked",masked)

    # ترکیب لبه‌ها با ماسک
    masked_edges = cv2.bitwise_and(edges, mask)

    # تشخیص خطوط جاده با Hough Transform
    lines = cv2.HoughLinesP(masked_edges, rho=3, theta=np.pi / 180, threshold=50, minLineLength=20, maxLineGap=30)
    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-5)  # جلوگیری از تقسیم بر صفر
            if slope < 0:  # خط چپ
                left_lines.append((x1, y1, x2, y2))
            else:  # خط راست
                right_lines.append((x1, y1, x2, y2))
            cv2.line(edges, (x1, y1), (x2, y2), (255, 255, 255), 3)
            cv2.line(masked_edges, (x1, y1), (x2, y2), (255, 255, 255), 3)

    # محاسبه زاویه میانگین خطوط
    def average_slope(lines):
        slopes = [((y2 - y1) / (x2 - x1 + 1e-5)) for x1, y1, x2, y2 in lines]
        return np.mean(slopes) if slopes else 0

    left_slope = average_slope(left_lines)
    right_slope = average_slope(right_lines)

    # میانگین زاویه برای هدایت خودرو
    avg_slope = (left_slope + right_slope) / 2
    steering_angle = np.arctan(avg_slope) * (180 / np.pi)  # تبدیل به درجه

    # نمایش پردازش‌ها (اختیاری)
    cv2.imshow("Edges", edges)
    cv2.imshow("Masked Edges", masked_edges)

    return steering_angle
    """
    پردازش تصویر برای تشخیص موانع بر اساس ویژگی‌های HVS
    """

    # تبدیل تصویر به سطح خاکستری
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # اعمال Gaussian Blur برای کاهش نویز
    blurred = cv2.GaussianBlur(gray, (7, 7), 3)

    # استفاده از تشخیص لبه Canny
    edges = cv2.Canny(blurred, 50, 150)

    # تعریف ماسک برای تمرکز بر ناحیه مهم (مانند قسمت پایین تصویر)
    height, width = edges.shape
    region = np.array([[(10, height), (width // 3, height // 2),
                        (2 * width // 3, height // 2), (width - 10, height)]], dtype=np.int32)

    mask = np.zeros_like(edges)
    masked = cv2.fillPoly(mask, region, 255)
    cv2.imshow("masked",masked)
    # ترکیب لبه‌ها با ماسک برای مشخص کردن ناحیه مهم
    masked_edges = cv2.bitwise_and(edges, mask)

    # اعمال Hough Transform برای تشخیص خطوط
    lines = cv2.HoughLinesP(masked_edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=20, maxLineGap=30)

    obstacle_detected = False
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        obstacle_detected = True

    # نمایش پردازش‌ها (در صورت نیاز)
    cv2.imshow("Edges", edges)
    cv2.imshow("Masked Edges", masked_edges)
    cv2.imshow("Final Detection", image)

    return obstacle_detected

try:
    while True:
        # Counting the loops
        counter += 1

        # Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        # car.setSpeed(50)

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
                
            # Returns an opencv image type array. if you use PIL you need to invert the color channels.
            image = car.getImage()
            
            # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # finImage = cv2.GaussianBlur(gray, (7, 7), 3)
            # cv2.imshow("blur", finImage)

            # edge = cv2.Canny(finImage, 50, 150)

            # poly = np.array([[(5,250), (120, 127), (127, 120), (255, 250)]])

            # mask = np.zeros_like(edge)
            # mask = cv2.fillPoly(mask, poly, 255)
            # cv2.imshow("mask", mask)
            # mask = cv2.bitwise_and(edge, mask)


            # hough = cv2.HoughLinesP(
            #     mask,
            #     rho=3,
            #     theta=np.pi / 180,
            #     threshold=100,
            #     lines=np.array([]),
            #     minLineLength=20,
            #     maxLineGap=25
            # )

            # linescount = 0
            # try:
            #     for line in hough:
            #         linescount += 1
            #         for x1, y1, x2, y2 in line:
            #             # print(x1,x2)
            #             cv2.line(edge, (x1, y1), (x2, y2), (255, 255, 255), 3)
            #     print("Count of lines is : ", linescount)
            # except:
            #     print("No lines for detect")
            # cv2.imshow("edge", edge)

            
            angle = detect_lane_and_steer(image)
            
            # تنظیم فرمان بر اساس زاویه
            if angle > 10:  # جاده به سمت راست می‌پیچد
                car.setSteering(-int(angle))  
            elif angle < -10:  # جاده به سمت چپ می‌پیچد
                car.setSteering(-int(angle))  
            else:  # مستقیم حرکت کند
                car.setSteering(0)
            
            car.setSpeed(100)  # سرعت ثاب
            

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
