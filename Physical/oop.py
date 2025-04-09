# کتابخانه‌های مورد نیاز برای کنترل آردوینو و زمان‌بندی
import time
from pyfirmata import Arduino, util
from time import sleep
from pyfirmata.pyfirmata import Board

# راه‌اندازی برد آردوینو روی پورت COM3
port = Arduino('com3')
    
# کلاس اصلی برای کنترل حرکات ربات
class control():
    def __init__(self, motor11, motor12, motor21, motor22, ina1, ina2, Servo):
        # مقداردهی اولیه پین‌های موتور و سروو
        # موتور اول - پین‌های کنترل جهت
        self.motor11 = port.get_pin(motor11)  # پین اول موتور 1
        self.motor12 = port.get_pin(motor12)  # پین دوم موتور 1
        # موتور دوم - پین‌های کنترل جهت
        self.motor21 = port.get_pin(motor21)  # پین اول موتور 2
        self.motor22 = port.get_pin(motor22)  # پین دوم موتور 2
        # پین‌های کنترل سرعت موتورها
        self.ina1 = port.get_pin(ina1)        # کنترل سرعت موتور 1 (PWM)
        self.ina2 = port.get_pin(ina2)        # کنترل سرعت موتور 2 (PWM)
        self.Servo = port.get_pin(Servo)      # کنترل موتور سروو برای فرمان

    # تابع حرکت به جلو
    def forward(self, speed1=1, speed2=1, servo=90):
        # تنظیم جهت موتور 1 برای حرکت به جلو
        self.motor11.write(1)  # پین 1 موتور 1 روشن
        self.motor12.write(0)  # پین 2 موتور 1 خاموش
        # تنظیم جهت موتور 2 برای حرکت به جلو
        self.motor21.write(0)  # پین 1 موتور 2 خاموش
        self.motor22.write(1)  # پین 2 موتور 2 روشن
        # تنظیم سرعت موتورها و زاویه سروو
        self.ina1.write(speed1)  # سرعت موتور 1
        self.ina2.write(speed2)  # سرعت موتور 2
        self.Servo.write(servo)  # زاویه سروو در حالت مستقیم (90 درجه)

    # تابع حرکت به عقب
    def back(self, speed1=1, speed2=1, servo=90):
        # تنظیم جهت موتور 1 برای حرکت به عقب
        self.motor11.write(0)  # پین 1 موتور 1 خاموش
        self.motor12.write(1)  # پین 2 موتور 1 روشن
        # تنظیم جهت موتور 2 برای حرکت به عقب
        self.motor21.write(1)  # پین 1 موتور 2 روشن
        self.motor22.write(0)  # پین 2 موتور 2 خاموش
        # تنظیم سرعت موتورها و زاویه سروو
        self.ina1.write(speed1)  # سرعت موتور 1
        self.ina2.write(speed2)  # سرعت موتور 2
        self.Servo.write(servo)  # زاویه سروو در حالت مستقیم (90 درجه)

    # تابع چرخش به راست
    def right(self, speed1=.8, speed2=.6, servo=110):
        # تنظیم جهت موتور 1 برای حرکت به جلو
        self.motor11.write(1)  # پین 1 موتور 1 روشن
        self.motor12.write(0)  # پین 2 موتور 1 خاموش
        # تنظیم جهت موتور 2 برای حرکت به جلو
        self.motor21.write(0)  # پین 1 موتور 2 خاموش
        self.motor22.write(1)  # پین 2 موتور 2 روشن
        # تنظیم سرعت موتورها و زاویه سروو
        self.ina1.write(speed1)  # سرعت موتور 1
        self.ina2.write(speed2)  # سرعت موتور 2
        self.Servo.write(servo)  # زاویه سروو برای چرخش به راست (110 درجه)

    # تابع چرخش به چپ
    def left(self, speed1=.6, speed2=.8, servo=70):
        # تنظیم جهت موتور 1 برای حرکت به جلو
        self.motor11.write(1)  # پین 1 موتور 1 روشن
        self.motor12.write(0)  # پین 2 موتور 1 خاموش
        # تنظیم جهت موتور 2 برای حرکت به جلو
        self.motor21.write(0)  # پین 1 موتور 2 خاموش
        self.motor22.write(1)  # پین 2 موتور 2 روشن
        # تنظیم سرعت موتورها و زاویه سروو
        self.ina1.write(speed1)  # سرعت موتور 1
        self.ina2.write(speed2)  # سرعت موتور 2
        self.Servo.write(servo)  # زاویه سروو برای چرخش به چپ (70 درجه)

    # تابع توقف ربات
    def stop(self, speed1=0, speed2=0, servo=90):
        # تنظیم جهت موتورها برای توقف
        self.motor11.write(1)  # پین 1 موتور 1 روشن
        self.motor12.write(1)  # پین 2 موتور 1 روشن
        self.motor21.write(1)  # پین 1 موتور 2 روشن
        self.motor22.write(1)  # پین 2 موتور 2 روشن
        # تنظیم سرعت موتورها و زاویه سروو
        self.ina1.write(speed1)  # سرعت موتور 1 (0)
        self.ina2.write(speed2)  # سرعت موتور 2 (0)
        self.Servo.write(servo)  # زاویه سروو در حالت مستقیم (90 درجه)

# ایجاد یک نمونه از کلاس کنترل با تنظیمات پین‌های مشخص شده
# d:2:o - پین دیجیتال 2 به عنوان خروجی
# d:3:p - پین دیجیتال 3 به عنوان PWM
# s - برای کنترل سروو
robot = control('d:2:o', 'd:4:o', 'd:7:o', 'd:5:o', 'd:3:p', 'd:6:p', 'd:9:s')

# کد زیر برای کنترل دستی ربات است (در حالت کامنت)
# while True:
#     x = int(input("m : "))  # دریافت دستور از کاربر
#     if x == 1:
#         robot.right()  # چرخش به راست
#     elif x == 2:
#         robot.back()   # حرکت به عقب
#     elif x == 3:
#         robot.forward()  # حرکت به جلو
#     elif x == 4:
#         robot.left()   # چرخش به چپ
