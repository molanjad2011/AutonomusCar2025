import pyfirmata
import cv2
import time
import numpy as np
import oop
import RPi.GPIO as GPIO
from pyfirmata import Arduino, util
from pyfirmata.pyfirmata import Board

port = Arduino("test usb")

class Car():
    def __init__(self):
        # ایجاد کنترل‌کننده و تنظیم پین‌های موتور
        self.robot = oop.control('d:2:o', 'd:4:o', 'd:7:o', 'd:5:o', 'd:3:p', 'd:6:p', 'd:9:s')
        
        # دسترسی به پین‌های موتور از کلاس control
        self.motor11 = self.robot.motor11  # پین اول موتور 1
        self.motor12 = self.robot.motor12  # پین دوم موتور 1
        self.motor21 = self.robot.motor21  # پین اول موتور 2
        self.motor22 = self.robot.motor22  # پین دوم موتور 2
        self.ina1 = self.robot.ina1      # کنترل سرعت موتور 1
        self.ina2 = self.robot.ina2      # کنترل سرعت موتور 2
        self.Servo = self.robot.Servo    # سروو موتور
        
        # متغیرهای کنترلی
        self.steering_value = 0
        self.speed_value = 0
        self.sensor_status = 1
        self.image_mode = 1
        self.get_Speed = 1
        self.sensor_angle = 90
        self.MIN_ANGLE = 60
        self.MAX_ANGLE = 120
        self.CENTER_ANGLE = 90
        
        # تنظیمات دوربین
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,512)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,512)
        self.cap.set(cv2.CAP_PROP_FPS,60)
        
        # تنظیمات سنسور التراسونیک
        self.TRIG = 40  # پین GPIO 40 برای Trigger
        self.ECHO = 38  # پین GPIO 38 برای Echo
        
        # راه‌اندازی پین‌های GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)
        
        # مقداردهی اولیه trigger
        GPIO.output(self.TRIG, False)
        time.sleep(0.1)  # تاخیر برای آماده شدن سنسور

    def get_distance(self):
        # cm ultrasonic
        # ارسال پالس 10 میکروثانیه‌ای
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        # محاسبه زمان رفت و برگشت پالس
        pulse_start = time.time()
        pulse_end = time.time()
        timeout = pulse_start + 0.1

        # اندازه‌گیری زمان شروع
        while GPIO.input(self.ECHO) == 0 and pulse_start < timeout:
            pulse_start = time.time()

        # اندازه‌گیری زمان پایان
        while GPIO.input(self.ECHO) == 1 and pulse_end < timeout:
            pulse_end = time.time()

        # محاسبه فاصله
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # سرعت صوت
        distance = round(distance, 2)

        return distance if 2 < distance < 400 else 0
def normalize_steering(self, steering_percent):
    # تبدیل 180 به چپ و 0 به راست
    # -100% = 180 درجه (چپ کامل)
    # +100% = 0 درجه (راست کامل)
    angle = 90 - (steering_percent * 0.4)  # ضریب 0.4 برای محدود کردن دامنه حرکت
    
    # محدود کردن زاویه بین 70 تا 110 درجه
    if angle > 110:
        angle = 110
    elif angle < 70:
        angle = 70
        
    return angle

    def get_distance(self):
        """اندازه‌گیری فاصله با سنسور التراسونیک
        خروجی: فاصله بر حسب سانتی‌متر"""
        
        # ...existing code...  
        
        # محاسبه فاصله 
        pulse_duration = pulse_end - pulse_start   
        distance_cm = pulse_duration * 17150  # تبدیل به س انتی‌متر (سرعت صوت / 2)
        distance_cm = round(distance_cm, 2)
    
        # برگرداندن فاصله معتبر (بین 2 تا 400 سانتی‌متر)
        return distance_cm if 2 < distance_cm < 400 else 0 

    def setSteering(self, steering):
        """تنظیم زاویه فرمان - ورودی بین -90 تا 90 درجه"""
        steering_percent = (steering / 90.0) * 100
        steering_percent = max(-100, min(100, steering_percent))
        servo_angle = self.normalize_steering(steering_percent)
        self.steering_value = steering
        
        speed_left = abs(self.speed_value)/3000.0
        speed_right = abs(self.speed_value)/3000.0
        
        if steering_percent > 0:  # چرخش به راست
            speed_right *= (1 - abs(steering_percent/200))
        else:  # چرخش به چپ
            speed_left *= (1 - abs(steering_percent/200))
            
        self.robot.forward(speed1=speed_left, speed2=speed_right, servo=servo_angle)

    def setSpeed(self, speed):
        """تنظیم سرعت - ورودی بین -3000 تا 3000"""
        self.speed_value = speed
        speed_normalized = abs(speed) / 3000.0
        
        steering_percent = (self.steering_value / 90.0) * 100
        speed_left = speed_normalized
        speed_right = speed_normalized
        
        if steering_percent > 0:
            speed_right *= (1 - abs(steering_percent/200))
        else:
            speed_left *= (1 - abs(steering_percent/200))
        
        if speed >= 0:
            self.robot.forward(speed1=speed_left, 
                            speed2=speed_right,
                            servo=self.normalize_steering(steering_percent))
        else:
            self.robot.back(speed1=speed_left, 
                          speed2=speed_right,
                          servo=self.normalize_steering(steering_percent))

    def getData(self):
        ret, self.image = self.cap.read()
        distance = self.get_distance()
        self.sensors = [distance, distance, distance]
        self.current_speed = abs(self.speed_value)

    def getImage(self):
        return self.image

    def getSensors(self):
        return self.sensors

    def setSensorAngle(self, angle):
        self.sensor_angle = angle

    def stop(self):
        self.speed_value = 0
        self.steering_value = 0
        self.robot.stop()

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        GPIO.cleanup()  # آزادسازی پین‌های GPIO
        self.stop()