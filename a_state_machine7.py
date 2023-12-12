import time
from machine import Pin
from pololu_3pi_2040_robot import robot
from machine import Pin, UART
import math

uart = UART(0, 115200, tx=Pin(28), rx=Pin(29), stop = 1, timeout = 10) # Data mode
# uart = UART(0, 38400, bits=8, tx=Pin(28), rx=Pin(29), stop = 1, timeout = 10) # Command mode
#led = Pin(25, Pin.OUT)

display = robot.Display()
motors = robot.Motors()
encoders = robot.Encoders()
display = robot.Display()
yellow_led = robot.YellowLED()
bump_sensors = robot.BumpSensors()
bump_sensors.calibrate()
time.sleep_ms(1000)
speed = 2
x = 0
y = 0
Bigcar_x = 0
Bigcar_y = 0
drive_motors = False
last_time_gyro_reading = None
turn_rate = 0.0     # degrees per second
robot_angle = 0.0   # degrees
flag1 = False
flag = False
target_angle = 0.0
max_speed = 1000
last_time_far_from_target = None
current_state = -1
kp = 140
kd = 4
data_angle = 0
previous_left_encoders=0
previous_right_encoders=0

imu = robot.IMU()
imu.reset()
imu.enable_default()


def send_position(a,b):
    data_to_send = "({},{})".format(a, b)

    # Encode the string into bytes
    encoded_data = data_to_send.encode()

    # Send the encoded data over UART
    uart.write(encoded_data)

def get_previous_encoders():
    global previous_left_encoders,previous_right_encoders
    previous_left_encoders,previous_right_encoders = encoders.get_counts()

def calculate_distance():

    global x,y
    left_encoder, right_encoder = encoders.get_counts()
    left_distance = (left_encoder - previous_left_encoders)/360.0*3.1415926*3.175
    right_distance = (right_encoder - previous_right_encoders)/360.0*3.1415926*3.175

    robot_angle2 = math.radians(robot_angle)  # Convert degrees to radians
    distance = (left_distance + right_distance)/2
    x -= math.sin(robot_angle2)*distance
    y += math.cos(robot_angle2)*distance


   

def start_to_turn(angle):
    #global varible in def should be announced(used in other functions)
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    target_angle = robot_angle + angle
    drive_motors = not drive_motors
    #important to avoid oscillation
    time.sleep_ms(500)
    last_time_far_from_target = time.ticks_ms()
    last_time_gyro_reading = time.ticks_us()

def left_turn():
    #global varible in def should be announced
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    display.fill(0)
    display.text(f"left_turn", 0, 20, 1)
    display.show()
    #turn_speed and far_from_target are local variables without global annoucement
    if drive_motors:
        far_from_target = abs(robot_angle - target_angle) > 3
        if far_from_target:
            last_time_far_from_target = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 250:
            drive_motors = False
    if drive_motors:
        turn_speed = (target_angle - robot_angle) * kp - turn_rate * kd
        if turn_speed > max_speed: turn_speed = max_speed
        if turn_speed < -max_speed: turn_speed = -max_speed
        motors.set_speeds(-turn_speed, turn_speed)
    else:
        motors.off()


def right_turn():
    #global varible in def should be announced
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    display.fill(0)
    display.text(f"right_turn", 0, 20, 1)
    display.show() 
    #turn_speed and far_from_target are local variables without global annoucement
    if drive_motors:
        far_from_target = abs(robot_angle - target_angle) > 3
        if far_from_target:
            last_time_far_from_target = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 250:
            drive_motors = False
    if drive_motors:
        turn_speed = (target_angle - robot_angle) * kp - turn_rate * kd
        if turn_speed > max_speed: turn_speed = max_speed
        if turn_speed < -max_speed: turn_speed = -max_speed
        motors.set_speeds(-turn_speed, turn_speed)
    else:
        motors.off()

def forward():
    motors.set_left_speed(max_speed)
    motors.set_right_speed(max_speed)
    display.fill(0)
    display.text(f"Moving Forward", 0, 20, 1)
    display.show()

def backward():
    motors.set_left_speed(-max_speed)
    motors.set_right_speed(-max_speed)
    display.fill(0)
    display.text(f"Moving Backword", 0, 20, 1)
    display.show()  

def stop():
    motors.set_left_speed(0)
    motors.set_right_speed(0)
    display.fill(0)
    display.text(f"Stop", 0, 20, 1)
    display.show()

def wait_for_uart_angle():
    global data_angle
    while True:
        if uart.any():  # Check if there is data available
            data_angle = int(uart.read())
            break  # Exit the loop after receiving data

def display_state(number):
    display.fill(0)
    display.text(f"state{number}", 50, 0, 1)
    display.show()



while True:

    bump_sensors.read()
    if imu.gyro.data_ready():
        imu.gyro.read()
        turn_rate = imu.gyro.last_reading_dps[2]  # degrees per second
        now = time.ticks_us()
        #only do when turn command is given
        if last_time_gyro_reading:
            dt = time.ticks_diff(now, last_time_gyro_reading)
            robot_angle += turn_rate * dt / 1000000
        last_time_gyro_reading = now


    display.fill(0)
    display.text(f"angle:{robot_angle:>9.3f}", 0, 35)
    display.text(f"x:{x:>5}", 0, 44)
    display.text(f"y:{y:>5}", 0, 55)
    display.show()

    data = uart.read()

    #led.value(0)  # yellow LED on
    #time.sleep(2)
    #led.value(1)  # yellow LED off
    if data:
        display.fill(0)
        display.text("data"+str(data), 0, 0)
        display.show()


    if current_state == -1:
        display_state(-1)
        #the same time as zar start to move
        if not flag1:
            start_time = time.ticks_ms()
            last_angle = 0
            flag1 = True
        # detects that bigcar is turning left 90 degrees
        if  robot_angle - last_angle > 30:
            elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
            Bigcar_distance = speed * (elapsed_time / 1000.0)
            Bigcar_y += Bigcar_distance
            send_position(Bigcar_x,Bigcar_y)
            last_angle = robot_angle
            start_time = time.ticks_ms()
        # detects that bigcar is turning right 90 degrees
        if  robot_angle - last_angle < -30:
            elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
            Bigcar_distance = speed * (elapsed_time / 1000.0)
            Bigcar_x -= Bigcar_distance
            send_position(Bigcar_x,Bigcar_y)
            last_angle = robot_angle
            start_time = time.ticks_ms()
        if  data == b'go':
            if not flag:
                start_down = time.ticks_us()
                motors.set_left_speed(max_speed)
                motors.set_right_speed(max_speed)
                elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
                Bigcar_distance = speed * (elapsed_time / 1000.0)
                Bigcar_y += Bigcar_distance
                send_position(Bigcar_x,Bigcar_y)
                flag = True

        if  flag:              
            dt = time.ticks_diff(time.ticks_us(), start_down)
            if dt > 2000000:
                stop()
                current_state = 0


    if current_state == 0:
        display_state(0)
        if data == b'1':
            get_previous_encoders()
            forward()
            
            current_state = 1
        if data == b'2':
            backward()
            start_back = time.ticks_us()
     
            current_state = 2
        if data == b'3':
            display_state(3)
            wait_for_uart_angle()
            current_state = 3
            start_to_turn(data_angle)
        if data == b'4':
            display_state(4)
            wait_for_uart_angle()
            current_state = 4
            start_to_turn(-data_angle)

    if current_state == 1:
        display_state(1)
        if bump_sensors.left_is_pressed() or bump_sensors.right_is_pressed():
            backward()
            start_back = time.ticks_us()
            current_state = 2

        if data == b'3':
            calculate_distance()
            motors.off()
            display_state(3)
            wait_for_uart_angle()
            current_state = 3
            send_position(x,y)
            start_to_turn(data_angle)
                
        if data == b'4':
            calculate_distance()
            motors.off()
            display_state(4)
            wait_for_uart_angle()
            current_state = 4
            send_position(x,y)
            start_to_turn(-data_angle)

    if current_state == 2:
        display_state(2)
        now_back = time.ticks_us()
        dt = time.ticks_diff(now_back, start_back)
        if dt > 500000:
            stop()
            current_state = 0
            calculate_distance()
            send_position(x,y)

    if current_state == 3:
        display_state(3)
        left_turn()
        if not drive_motors:
            stop()
            current_state = 0
     

    if current_state == 4:
        display_state(4)
        right_turn()
        if not drive_motors:
            stop()
            current_state = 0
         




    
   # uart.write(b'Hello')
