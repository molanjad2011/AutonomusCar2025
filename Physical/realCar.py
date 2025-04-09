import pyfirmata
import cv2
import re
import base64
import time
import socket
import numpy as np
import utils
import oop
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
        self.steering_value = 0
        self.speed_value = 0
        self.sensor_status = 1
        self.image_mode = 1
        self.get_Speed = 1
        self.sensor_angle = 90
        self.MIN_ANGLE = 60
        self.MAX_ANGLE = 120
        self.CENTER_ANGLE = 90
        self.image = None
        self.sensors = None
        self.current_speed = None
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,512)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,512)
        self.cap.set(cv2.CAP_PROP_FPS,60)

    def normalize_steering(self, steering_percent):
        """تبدیل درصد فرمان به زاویه سروو (60 تا 120 درجه)"""
        angle = self.CENTER_ANGLE + (steering_percent * 0.3)
        return max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle))
    def setSteering(self, steering):
        """تنظیم زاویه فرمان - ورودی بین -90 تا 90 درجه"""
        # تبدیل زاویه به درصد
        steering_percent = (steering / 90.0) * 100
        # محدود کردن درصد
        steering_percent = max(-100, min(100, steering_percent))
        # تبدیل به زاویه سروو
        servo_angle = self.normalize_steering(steering_percent)
        self.steering_value = steering
        
        # محاسبه سرعت موتورها برای چرخش نرم
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
        
        # محاسبه سرعت موتورها با توجه به زاویه فرمان
        steering_percent = (self.steering_value / 90.0) * 100
        speed_left = speed_normalized
        speed_right = speed_normalized
        
        if steering_percent > 0:  # چرخش به راست
            speed_right *= (1 - abs(steering_percent/200))
        else:  # چرخش به چپ
            speed_left *= (1 - abs(steering_percent/200))
        
        if speed >= 0:
            self.robot.forward(speed1=speed_left, 
                            speed2=speed_right,
                            servo=self.normalize_steering(steering_percent))
        # else:
        #     self.robot.back(speed1=speed_left, 
        #                   speed2=speed_right,
        #                   servo=self.normalize_steering(steering_percent))

    def getData(self):
        """دریافت داده‌ها از دوربین و سنسورها"""
        ret, self.image = self.cap.read()
        self.sensors = [0, 0, 0]  # مقادیر پیش‌فرض برای سنسورها
        self.current_speed = abs(self.speed_value)

    def getImage(self):
        """برگرداندن تصویر دوربین"""
        return self.image

    def getSensors(self):
        """برگرداندن مقادیر سنسورها"""
        return self.sensors

    def setSensorAngle(self, angle):
        """تنظیم زاویه سنسور"""
        self.sensor_angle = angle

    def stop(self):
        """توقف کامل ربات"""
        self.speed_value = 0
        self.steering_value = 0
        self.robot.stop()

    def __del__(self):
        """آزادسازی منابع"""
        if hasattr(self, 'cap'):
            self.cap.release()
        self.stop()