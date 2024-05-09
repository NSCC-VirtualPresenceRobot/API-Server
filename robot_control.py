import time
import evdev
from evdev import InputDevice, ecodes
from RPLCD.i2c import CharLCD
import RPi.GPIO as GPIO
import threading

class RobotControl:
    def __init__(self):
        GPIO.setwarnings(False)

        LCD_COLUMNS = 16
        LCD_ROWS = 2

        self.lcd = CharLCD('PCF8574', 0x27, cols=LCD_COLUMNS, rows=LCD_ROWS)

        self.con1ena = 10
        self.con1in1 = 4
        self.con1in2 = 14
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

        GPIO.setup(self.con1in1,GPIO.OUT)
        GPIO.setup(self.con1in2,GPIO.OUT)
        GPIO.setup(self.con1ena,GPIO.OUT)

        GPIO.setup(self.con1in3,GPIO.OUT)
        GPIO.setup(self.con1in4,GPIO.OUT)
        GPIO.setup(self.con1enb,GPIO.OUT)

        GPIO.setup(self.con2in1,GPIO.OUT)
        GPIO.setup(self.con2in2,GPIO.OUT)
        GPIO.setup(self.con2ena,GPIO.OUT)
         
        GPIO.setup(self.con2in3,GPIO.OUT)
        GPIO.setup(self.con2in4,GPIO.OUT)
        GPIO.setup(self.con2enb,GPIO.OUT)
        
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.LOW)
        GPIO.output(self.con1ena,GPIO.LOW)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.LOW)
        GPIO.output(self.con1enb,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.LOW)
        GPIO.output(self.con2ena,GPIO.LOW)

        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.LOW)
        GPIO.output(self.con2enb,GPIO.LOW)

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
    
    # Start from here, add new 6 movements
    def forward_left(self):
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.HIGH)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.LOW)

        GPIO.output(self.con2in3,GPIO.HIGH)
        GPIO.output(self.con2in4,GPIO.LOW)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Steer FL"

    def forward_right(self):
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.LOW)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.HIGH)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.HIGH)

        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.LOW)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Steer FR"

    def diagonal_up_left(self):
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.HIGH)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.HIGH)

        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.LOW)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Diagonal"

    def diagonal_up_right(self):
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.LOW)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.HIGH)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.LOW)

        GPIO.output(self.con2in3,GPIO.HIGH)
        GPIO.output(self.con2in4,GPIO.LOW)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Diagonal"

    def diagonal_back_left(self):
        GPIO.output(self.con1in1,GPIO.LOW)
        GPIO.output(self.con1in2,GPIO.LOW)

        GPIO.output(self.con1in3,GPIO.HIGH)
        GPIO.output(self.con1in4,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.LOW)
        GPIO.output(self.con2in2,GPIO.LOW)

        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.HIGH)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Diagonal"

    def diagonal_back_right(self):
        GPIO.output(self.con1in1,GPIO.HIGH)
        GPIO.output(self.con1in2,GPIO.LOW)

        GPIO.output(self.con1in3,GPIO.LOW)
        GPIO.output(self.con1in4,GPIO.LOW)

        GPIO.output(self.con2in1,GPIO.HIGH)
        GPIO.output(self.con2in2,GPIO.LOW)

        GPIO.output(self.con2in3,GPIO.LOW)
        GPIO.output(self.con2in4,GPIO.LOW)


        self.front_right_pwm.ChangeDutyCycle(100)
        self.front_left_pwm.ChangeDutyCycle(100)
        self.back_left_pwm.ChangeDutyCycle(100)
        self.back_right_pwm.ChangeDutyCycle(100)

        self.movement_status = "Diagonal"

    def stop(self):
        self.front_right_pwm.ChangeDutyCycle(0)
        self.front_left_pwm.ChangeDutyCycle(0)
        self.back_left_pwm.ChangeDutyCycle(0)
        self.back_right_pwm.ChangeDutyCycle(0)
        
        self.movement_status = "Hello World"

    def handle_movement(self):
        # Control RPi directly
        # Track state of each directional key
        direction_pressed = {
            ecodes.KEY_W: False,
            ecodes.KEY_S: False,
            ecodes.KEY_A: False,
            ecodes.KEY_D: False,
            ecodes.KEY_LEFT: False,
            ecodes.KEY_RIGHT: False
        }
        
        for event in self.device.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code in direction_pressed:
                    if event.value == 1:
                        direction_pressed[event.code] = True
                        if any(direction_pressed.values()):
                            if direction_pressed[ecodes.KEY_W]:
                                self.forward()
                            elif direction_pressed[ecodes.KEY_S]:
                                self.backward()
                            elif direction_pressed[ecodes.KEY_A]:
                                self.strafe_left()
                            elif direction_pressed[ecodes.KEY_D]:
                                self.strafe_right()
                            elif direction_pressed[ecodes.KEY_LEFT]:
                                self.turn_left()
                            elif direction_pressed[ecodes.KEY_RIGHT]:
                                self.turn_right()
                            elif direction_pressed[ecodes.KEY_W] and direction_pressed[ecodes.KEY_A]:
                                self.diagonal_up_left()
                            elif direction_pressed[ecodes.KEY_W] and direction_pressed[ecodes.KEY_D]:
                                self.diagonal_up_right()
                            elif direction_pressed[ecodes.KEY_S] and direction_pressed[ecodes.KEY_A]:
                                self.diagonal_back_left()
                            elif direction_pressed[ecodes.KEY_S] and direction_pressed[ecodes.KEY_D]:
                                self.diagonal_back_right()
                            elif direction_pressed[ecodes.KEY_W] and direction_pressed[ecodes.KEY_LEFT]:
                                self.forward_left()
                            elif direction_pressed[ecodes.KEY_W] and direction_pressed[ecodes.KEY_RIGHT]:
                                self.forward_right()
                        else:
                            self.stop()
                    elif event.value == 0:
                        direction_pressed[event.code] = False
                        if not any(direction_pressed.values()):
                            self.stop()

    
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

    movement_thread = threading.Thread(target=robot.handle_movement)
    movement_thread.start()

    lcd_thread = threading.Thread(target=robot.update_lcd)
    lcd_thread.start()

    # Make sure threads finish before cleanup
    movement_thread.join()
    lcd_thread.join()

    GPIO.cleanup()
