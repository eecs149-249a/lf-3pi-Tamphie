import time
from machine import Pin
from pololu_3pi_2040_robot import robot
from machine import Pin, UART


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


drive_motors = False
last_time_gyro_reading = None
turn_rate = 0.0     # degrees per second
robot_angle = 0.0   # degrees
target_angle = 0.0
max_speed = 3000
last_time_far_from_target = None
current_state = 0
kp = 140
kd = 4
angle_to_turn = 90
imu = robot.IMU()
imu.reset()
imu.enable_default()

def start_to_turn(angle):
    #written global varible in def should be announced
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    target_angle = robot_angle + angle
    drive_motors = not drive_motors
    #important to avoid oscillation
    time.sleep_ms(500)
    last_time_far_from_target = time.ticks_ms()
    last_time_gyro_reading = time.ticks_us()

def left_turn():
    #written global varible in def should be announced
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    display.fill(0)
    display.text("left_turn", 0, 0)
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
    #written global varible in def should be announced
    global target_angle, drive_motors
    global last_time_far_from_target, last_time_gyro_reading
    display.fill(0)
    display.text("right_turn", 0, 0)
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
    display.text("Moving Forward", 0, 0)
    display.show()

def backward():
    motors.set_left_speed(-max_speed)
    motors.set_right_speed(-max_speed)
    display.fill(0)
    display.text("Moving Backword", 0, 0)
    display.show()  

def stop():
    motors.set_left_speed(0)
    motors.set_right_speed(0)
    display.fill(0)
    display.text("Stop", 0, 0)
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


    data = uart.read()
    #led.value(0)  # yellow LED on
    #time.sleep(2)
    #led.value(1)  # yellow LED off
    display.fill(0)
    display.text(str(data), 0, 0)
    display.show()

    if current_state == 0:
        display.fill(0)
        display.text("state0", 0, 0)
        display.show()  
        stop()
        if data == b'1':
            forward()
            current_state = 1
        if data == b'2':
            backward()
            start_back = time.ticks_us()
            current_state = 2
        if data == b'3':
            start_to_turn(angle_to_turn)
            current_state = 3
        if data == b'4':
            start_to_turn(-angle_to_turn)
            current_state = 4

    if current_state == 1:
        if bump_sensors.left_is_pressed():
            backward()
            start_back = time.ticks_us()
            current_state = 2
        elif bump_sensors.right_is_pressed():
            backward()
            start_back = time.ticks_us()
            current_state = 2

    if current_state == 2:
        display.fill(0)
        display.text("state2", 0, 0)
        display.show()  
        now_back = time.ticks_us()
        dt = time.ticks_diff(now_back, start_back)
        if dt > 2000000:
            current_state = 0

    if current_state == 3:
        display.fill(0)
        display.text("state3", 0, 0)
        display.show()  
        left_turn()
        if not drive_motors:
            current_state = 0

    if current_state == 4:
        display.fill(0)
        display.text("state4", 0, 0)
        display.show()  
        right_turn()
        if not drive_motors:
            current_state = 0   




    
   # uart.write(b'Hello')
