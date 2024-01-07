from function import line_tracking_pid, motor_control_function, check_for_turn

integral, previous_error = 0, 0
delta_time = 0.1
kp, ki, kd = 0.6, 0.2, 0.4
base_speed, max_speed = 50, 80 # in rpm

while True:
    line_input = input("Enter 8-Digit Number: ")

    sensor_readings = [int(digit) for digit in line_input]

    is_turn = check_for_turn(sensor_readings)

    if not is_turn:
        pid_output, integral, previous_error = line_tracking_pid(sensor_readings, kp, ki, kd, integral, previous_error, delta_time,max_speed)
        motor_control_function(base_speed, pid_output, max_speed)
    else:
        print("Turn detected or end of line")
