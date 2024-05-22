import RPi.GPIO as GPIO
import time
from RPLCD.i2c import CharLCD
import threading

class RobotControl:
    def __init__(self):
        LCD_COLUMNS = 16
        LCD_ROWS = 2

        self.lcd = CharLCD('PCF8574', 0x27, cols=LCD_COLUMNS, rows=LCD_ROWS)

        self.con1ena = 10  # PWM pin
        self.con1in1 = 4   # Control pin
        self.con1in2 = 14  # Control pin
        self.con1in3 = 8
        self.con1in4 = 11
        self.con1enb = 25

        self.con2ena = 13
        self.con2in1 = 16
        self.con2in2 = 19
        self.con2in3 = 20
        self.con2in4 = 26
        self.con2enb = 21

        GPIO.setmode(GPIO.BCM)

        # Reset all GPIO settings
        GPIO.cleanup()

        # Set up the control pins as outputs
        GPIO.setup(self.con1in1, GPIO.OUT)
        GPIO.setup(self.con1in2, GPIO.OUT)
        GPIO.setup(self.con1ena, GPIO.OUT)

        GPIO.setup(self.con1in3,GPIO.OUT)
        GPIO.setup(self.con1in4,GPIO.OUT)
        GPIO.setup(self.con1enb,GPIO.OUT)

        GPIO.setup(self.con2in1,GPIO.OUT)
        GPIO.setup(self.con2in2,GPIO.OUT)
        GPIO.setup(self.con2ena,GPIO.OUT)
         
        GPIO.setup(self.con2in3,GPIO.OUT)
        GPIO.setup(self.con2in4,GPIO.OUT)
        GPIO.setup(self.con2enb,GPIO.OUT)
        
        # Initialize control pins to low
        GPIO.output(self.con1in1, GPIO.LOW)
        GPIO.output(self.con1in2, GPIO.LOW)
        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.LOW)
        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.LOW)

        # Set up PWM on the enable pin
        self.front_right_pwm = GPIO.PWM(self.con1ena, 1000)
        self.front_right_pwm.start(0)
        self.front_left_pwm = GPIO.PWM(self.con1enb, 1000)
        self.front_left_pwm.start(0)
        self.back_left_pwm = GPIO.PWM(self.con2ena, 1000)
        self.back_left_pwm.start(0)
        self.back_right_pwm = GPIO.PWM(self.con2enb, 1000)
        self.back_right_pwm.start(0)

        self.movement_status = "Hello World"

    def forward(self):
        # front right
        GPIO.output(self.con1in1, GPIO.LOW)
        GPIO.output(self.con1in2, GPIO.HIGH)
        
        # front left
        GPIO.output(self.con1in3, GPIO.LOW)
        GPIO.output(self.con1in4, GPIO.HIGH)
        
        # back left
        GPIO.output(self.con2in1, GPIO.LOW)
        GPIO.output(self.con2in2, GPIO.HIGH)
        
        # back right
        GPIO.output(self.con2in3, GPIO.HIGH)
        GPIO.output(self.con2in4, GPIO.LOW)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Move Forward"

    def backward(self):
        GPIO.output(self.con1in1, GPIO.HIGH)
        GPIO.output(self.con1in2, GPIO.LOW)
        
        GPIO.output(self.con1in3, GPIO.HIGH)
        GPIO.output(self.con1in4, GPIO.LOW)
        
        GPIO.output(self.con2in1, GPIO.HIGH)
        GPIO.output(self.con2in2, GPIO.LOW)
        
        GPIO.output(self.con2in3, GPIO.LOW)
        GPIO.output(self.con2in4, GPIO.HIGH)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Move Backward"

    def strafe_left(self):
        GPIO.output(self.con1in1, GPIO.LOW)
        GPIO.output(self.con1in2, GPIO.HIGH)
    
        GPIO.output(self.con1in3, GPIO.HIGH)
        GPIO.output(self.con1in4, GPIO.LOW)
        
        GPIO.output(self.con2in1, GPIO.LOW)
        GPIO.output(self.con2in2, GPIO.HIGH)
        
        GPIO.output(self.con2in3, GPIO.LOW)
        GPIO.output(self.con2in4, GPIO.HIGH)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Move Left"

    def strafe_right(self):
        GPIO.output(self.con1in1, GPIO.HIGH)
        GPIO.output(self.con1in2, GPIO.LOW)
    
        GPIO.output(self.con1in3, GPIO.LOW)
        GPIO.output(self.con1in4, GPIO.HIGH)
        
        GPIO.output(self.con2in1, GPIO.HIGH)
        GPIO.output(self.con2in2, GPIO.LOW)
        
        GPIO.output(self.con2in3, GPIO.HIGH)
        GPIO.output(self.con2in4, GPIO.LOW)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Move Right"
   
    def turn_right(self):
        GPIO.output(self.con1in1, GPIO.HIGH)
        GPIO.output(self.con1in2, GPIO.LOW)
        
        GPIO.output(self.con1in3, GPIO.LOW)
        GPIO.output(self.con1in4, GPIO.HIGH)
        
        GPIO.output(self.con2in1, GPIO.LOW)
        GPIO.output(self.con2in2, GPIO.HIGH)
        
        GPIO.output(self.con2in3, GPIO.LOW)
        GPIO.output(self.con2in4, GPIO.HIGH)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Turn Right"
   
   
    def turn_left(self):
        GPIO.output(self.con1in1, GPIO.LOW)
        GPIO.output(self.con1in2, GPIO.HIGH)
        
        GPIO.output(self.con1in3, GPIO.HIGH)
        GPIO.output(self.con1in4, GPIO.LOW)
        
        GPIO.output(self.con2in1, GPIO.HIGH)
        GPIO.output(self.con2in2, GPIO.LOW)
        
        GPIO.output(self.con2in3, GPIO.HIGH)
        GPIO.output(self.con2in4, GPIO.LOW)
        
        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)
        
        self.movement_status = "Turn Left"

    def stop(self):
        self.front_right_pwm.ChangeDutyCycle(0)
        self.front_left_pwm.ChangeDutyCycle(0)
        self.back_left_pwm.ChangeDutyCycle(0)
        self.back_right_pwm.ChangeDutyCycle(0)
        
        self.movement_status = "Hello World"
    
    def cleanup(self):
        self.front_right_pwm.stop()
        self.front_left_pwm.stop()
        self.back_left_pwm.stop()
        self.back_right_pwm.stop()
        GPIO.cleanup()

    def update_lcd(self):
        prev_status = ""
        
        while True:
            if self.movement_status != prev_status:
                self.lcd.clear()
                if self.movement_status == "Hello World":
                    self.lcd.write_string("     Hello\n\r     World!")
                elif self.movement_status == "Move Forward":
                    self.lcd.write_string("     Moving\n\r    Forward!")
                elif self.movement_status == "Move Backward":
                    self.lcd.write_string("     Moving\n\r    Backward!")
                elif self.movement_status == "Move Left":
                    self.lcd.write_string("     Moving\n\r     Left!")
                elif self.movement_status == "Move Right":
                    self.lcd.write_string("     Moving\n\r     Right!")
                elif self.movement_status == "Turn Left":
                    self.lcd.write_string("    Turning\n\r     Left!")
                elif self.movement_status == "Turn Right":
                    self.lcd.write_string("    Turning\n\r    Right!")
                elif self.movement_status == "Steer FR":
                    self.lcd.write_string("    Steering\n\r     Right!")
                elif self.movement_status == "Steer FL":
                    self.lcd.write_string("    Steering\n\r     Left!")
                elif self.movement_status == "Steer BR":
                    self.lcd.write_string("   Reversing\n\r    Right!")
                elif self.movement_status == "Steer BL":
                    self.lcd.write_string("   Reversing\n\r     Left!")
                elif self.movement_status == "Diagonal":
                    self.lcd.write_string("     Moving\n\r   Diagonally!")
                prev_status = self.movement_status

if __name__ == "__main__":
    robot = RobotControl()
    try:
        robot.forward()
        time.sleep(20)  # Run forward for 20 seconds
        robot.stop()
    finally:
        robot.cleanup()
