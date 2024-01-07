import math
import serial
import time
import cv2
from cam_func import *
import cam_func

arduino = serial.Serial(port="COM18", baudrate=250000, timeout=0.01)

def send_motor_speed(x, y, z):
    command = f"{x:.2f};{y:.2f};{z:.2f}\n"
    arduino.write(bytes(command, "utf-8"))
    data = arduino.readline()
    return data

class BASE:
    def __init__(self, base_speed, max_speed, wheel_diameter = 0.058,lx = 0.15,ly = 0.15):
        self.vx = 0
        self.vy = 0
        self.wz = 0
        self.base_speed = base_speed
        self.max_speed = max_speed
        self.wheel_diameter = wheel_diameter
        self.lx = lx
        self.ly = ly

    def send_motor_speed_rpm(self, x, y, z):
        x,y,z = base.motor_control_function_Vx(x),base.motor_control_function_Vy(y),base.motor_control_function_Wz(z)
        command = f"{x:.2f};{y:.2f};{z:.2f}\n"
        arduino.write(bytes(command, "utf-8"))
        data = arduino.readline()
        return data

    def forward_kinematic(self, rpm_FL, rpm_FR, rpm_BL, rpm_BR):
        rpm_to_rad_per_sec = 2 * math.pi / 60

        omega_FL = rpm_FL * rpm_to_rad_per_sec
        omega_FR = rpm_FR * rpm_to_rad_per_sec
        omega_BL = rpm_BL * rpm_to_rad_per_sec
        omega_BR = rpm_BR * rpm_to_rad_per_sec

        vx = (self.wheel_diameter / 2) / 4 * (omega_FL + omega_FR + omega_BL + omega_BR)
        vy = (self.wheel_diameter / 2) / 4 * -(-omega_FL + omega_FR + omega_BL - omega_BR)
        wz = (
            (self.wheel_diameter / 2)
            / (4 * (self.lx + self.ly))
            * (omega_FL - omega_FR + omega_BL - omega_BR)
        )

        return vx, vy, wz

    def motor_control_function(self, base_speed, pid_output):
        left_adjustment = -pid_output
        right_adjustment = pid_output

        speed_fl = min(max(base_speed + left_adjustment, -max_speed), max_speed)
        speed_fr = min(max(base_speed + right_adjustment, -max_speed), max_speed)
        speed_bl = min(max(base_speed + left_adjustment, -max_speed), max_speed)
        speed_br = min(max(base_speed + right_adjustment, -max_speed), max_speed)

        self.vx, self.vy, self.wz = self.forward_kinematic(speed_fl, speed_fr, speed_bl, speed_br)

        if abs(self.wz) > 0.45:
            self.vx = 0.01
            self.wz *= 10

        val = send_motor_speed(self.vx, self.vy, self.wz)

        self.Debuging(speed_fl, speed_fr, speed_bl, speed_br, self.vx, self.vy, self.wz, val)

    def Debuging(self, speed_fl, speed_fr, speed_bl, speed_br, vx, vy, wz, val):
        x = round(vx, 2)
        y = round(vy, 2)
        z = round(wz, 2)
        fl = round(speed_fl, 3)
        fr = round(speed_fr, 3)
        bl = round(speed_bl, 3)
        br = round(speed_br, 3)
        print(
            "FL: {} FR: {} BL: {} BR: {} Vx: {} Vy: {} Wz: {} Val: {}         ".format(
                fl, fr, bl, br, x, y, z, val
            ),
            end="\r",
        )

    def motor_control_function_Vx(self, base_speed):
        speed_fl = min(max(base_speed, -self.max_speed), self.max_speed)
        speed_fr = min(max(base_speed, -self.max_speed), self.max_speed)
        speed_bl = min(max(base_speed, -self.max_speed), self.max_speed)
        speed_br = min(max(base_speed, -self.max_speed), self.max_speed)

        vx, vy, wz = self.forward_kinematic(speed_fl, speed_fr, speed_bl, speed_br)

        return vx

    def motor_control_function_Vy(self, base_speed):
        speed_fl = min(max(base_speed, -self.max_speed), self.max_speed)
        speed_fr = min(max(-base_speed, -self.max_speed), self.max_speed)
        speed_bl = min(max(-base_speed, -self.max_speed), self.max_speed)
        speed_br = min(max(base_speed, -self.max_speed), self.max_speed)

        vx, vy, wz = self.forward_kinematic(speed_fl, speed_fr, speed_bl, speed_br)

        return vy

    def motor_control_function_Wz(self, pid_output):
        left_adjustment = pid_output
        right_adjustment = -pid_output

        speed_fl = min(max(left_adjustment, -self.max_speed), self.max_speed)
        speed_fr = min(max(right_adjustment, -self.max_speed), self.max_speed)
        speed_bl = min(max(left_adjustment, -self.max_speed), self.max_speed)
        speed_br = min(max(right_adjustment, -self.max_speed), self.max_speed)

        vx, vy, wz = self.forward_kinematic(speed_fl, speed_fr, speed_bl, speed_br)

        return wz

class PID:
    def __init__(self, kp, ki, kd, max_speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.last_error = 0
        self.max_speed = max_speed
        
    def do_pid(self, input, destination_input , delta_time):
        # error
        error = destination_input - input

        # out of frame error
        if input == 999:
            error = self.last_error * 10 # out frame weight

        # intregral term
        if error == 0:
            self.integral = 0
        else:
            self.integral += error * delta_time

        # derirative term
        derivative = (error - self.last_error) / delta_time

        # PID with Saturation
        output = min(max(self.kp * error + self.ki * self.integral + self.kd * derivative, -max_speed), max_speed)

        self.last_error = error

        return output

class CHECK_STATE:
    def __init__(self):
         self.state = 1

    def check_for_turn(self, line1, line3, line4):
        if self.state == 1 and sum(line1) >= 3:
            self.state = 2
        elif self.state == 2 and sum(line1) < 3:
            self.state = 3
        elif self.state == 3:
            if sum(line1) >= 1 and sum(line3) >= 1 and sum(line4) >= 1:
                self.state = 1
                return True
            elif sum(line3) >= 1 and sum(line4) >= 1:
                self.state = 1
                return True
            elif sum(line1) >= 1 and sum(line3) >= 1:
                self.state = 1
                return True
            elif sum(line1) >= 1 and sum(line4) >= 1:
                self.state = 1
                return True
            else:
                self.state = 1
                return False
        return False

# base params
base_speed, max_speed = 50 , 90  # in rpms
base = BASE(base_speed = base_speed, max_speed = max_speed)

# Track function 
def TrackLineTillTurn(cap, kp, ki, kd):
    state = CHECK_STATE()
    pid = PID(kp = kp,
            ki = ki,
            kd = kd,
            max_speed = max_speed)

    while True:
        # start = time.time()
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        binary_image = BGR2BIN(frame, threshold=65)

        x1, x2, y1, y2 = 80, 360, 160, 480

        # Use this Value format -> 'xxx:xxxxxxxx'
        line1 = horizontal(x1, binary_image)
        line2 = horizontal(x2, binary_image)
        line3 = vertical(y1, binary_image)
        line4 = vertical(y2, binary_image)

        sensor_readings = int(line2.split(":")[0])

        line1Dot = [int(dig) for dig in (line1.split(":")[1])]
        line3Dot = [int(dig) for dig in (line3.split(":")[1])]
        line4Dot = [int(dig) for dig in (line4.split(":")[1])]

        DisplayCam(x1,x2,y1,y2,frame)

        is_turn = state.check_for_turn(line1Dot, line3Dot, line4Dot)

        if not is_turn:
            pid_output = pid.do_pid(sensor_readings, 320, 0.03)
            base.motor_control_function(base_speed, pid_output)
        else:
            send_motor_speed(0, 0, 0)
            break

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

def TrackLineTillTurn_Time(cap, kp, ki, kd, milisec):
    start_time = time.time()
    pid = PID(kp = kp,
            ki = ki,
            kd = kd,
            max_speed = max_speed)
    
    while time.time() - start_time <= milisec / 1000.0:
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        binary_image = BGR2BIN(frame, threshold=95)

        x1, x2, y1, y2 = 80, 360, 160, 480

        DisplayCam(x1,x2,y1,y2,frame)

        line2 = horizontal(x2, binary_image)

        sensor_readings = int(line2.split(":")[0])

        pid_output = pid.do_pid(sensor_readings, 320, 0.03)
        base.motor_control_function(base_speed, pid_output)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    send_motor_speed(0, 0, 0)

def MoveTime(vx, vy, wz, milisec):
    start_time = time.time()
    while time.time() - start_time <= milisec / 1000.0:
        send_motor_speed(vx, vy, wz)
    send_motor_speed(0, 0, 0)

def DisplayCam(x1,x2,y1,y2,frame): 
    #### Uncomment this for Display #####
    height, width, _ = frame.shape
    # Draw line
    cv2.line(frame, (0, x1), (width, x1), (0, 0, 255), 1)
    cv2.line(frame, (0, x2), (width, x2), (0, 0, 255), 1)
    cv2.line(frame, (y1, 0), (y1, height), (0, 0, 255), 1)
    cv2.line(frame, (y2, 0), (y2, height), (0, 0, 255), 1)

    # Label line
    cv2.putText(
        frame,
        "line1",
        (width // 2, x1),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "line2",
        (width // 2, x2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "line3",
        (y1, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        "line4",
        (y2, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.imshow("Camera", frame)

def Tuning(cap, kp, ki, kd):
    Wz_pid = PID(kp = 0.5,
            ki = 0.0001,
            kd = kd,
            max_speed = max_speed)
    Vy_pid = PID(kp = 0.5,
            ki = 0.0001,
            kd = kd,
            max_speed = max_speed)

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        binary_image = BGR2BIN(frame, threshold=65)

        x1, x2, y1, y2 = 80, 360, 160, 480

        # Use this Value format -> 'xxx:xxxxxxxx'
        line1 = horizontal(x1, binary_image)
        line2 = horizontal(x2, binary_image)
        line3 = vertical(y1, binary_image)
        line4 = vertical(y2, binary_image)

        sensor_reading_1 = int(line1.split(":")[0])
        sensor_reading_2 = int(line2.split(":")[0])

        Wz_input = sensor_reading_1 - sensor_reading_2
        Vy_input = (sensor_reading_1 + sensor_reading_2)/2

        Wz_pid_output = Wz_pid.do_pid(Wz_input, 0, 0.03)
        Vy_pid_output = Vy_pid.do_pid(640-Vy_input, 320, 0.03)

        print(Vy_pid_output, Wz_pid_output)

        base.send_motor_speed_rpm(0,Vy_pid_output,-Wz_pid_output)

        if(abs(Vy_pid_output) < 4 and abs(Wz_pid_output) < 10):
            break
        
        DisplayCam(x1,x2,y1,y2,frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    send_motor_speed(0, 0, 0)