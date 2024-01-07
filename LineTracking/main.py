from lineTrackingFunction import *
import json

def read_config(file_path):
    with open(file_path, 'r') as file:
        config = json.load(file)
    return config

config_path = 'setting.json'
config = read_config(config_path)

print("Setup camera in progress...")
cap = LinetrackCam_Open()
print("Finished setup camera")

print("Start setup other params")
kp, ki, kd = config["mainPID"][0]["kp"], config["mainPID"][0]["ki"], config["mainPID"][0]["kd"]
print("Finished  other params")

print("Start Main program")

#Description of each function

#TrackLineTillTurn(cap, kp, ki, kd, line) LineTracking Till camera meet the Turn point
#TrackLineTillTurn_Time(cap, kp, ki, kd, xxx) LineTracking Till meet a certain time
#MoveTime(x,x,x,xxx) Move in each direction Vx Vy Wz and Duration (Vx Vy in m/s Wz in rad/s)
#Tuning(cap, kp, ki, kd) Make robot vertical align to line

#01
TrackLineTillTurn_Time(cap, kp, ki, kd, 9500)
TrackLineTillTurn(cap, kp, ki, kd)
TrackLineTillTurn_Time(cap, kp, ki, kd, 1350)
MoveTime(0, 0, -3.14, 1400)
TrackLineTillTurn(cap, kp, ki, kd)
TrackLineTillTurn_Time(cap, kp, ki, kd, 1350)
MoveTime(0, 0, -3.14, 1400)
TrackLineTillTurn_Time(cap, kp, ki, kd, 3000)
Tuning(cap)

time.sleep(2)

#13
MoveTime(-0.2,0,0,800)
MoveTime(0, 0, 3.14, 1400)
TrackLineTillTurn(cap, kp, ki, kd)
TrackLineTillTurn_Time(cap, kp, ki, kd, 4500)
Tuning(cap)

time.sleep(2)

#32
MoveTime(-0.2,0,0,1000)
MoveTime(0, 0, -3.14, 1400)
TrackLineTillTurn(cap, kp, ki, kd,2)
TrackLineTillTurn_Time(cap, kp, ki, kd, 1350)
MoveTime(0, 0, 3.14, 1400)
TrackLineTillTurn_Time(cap, kp, ki, kd, 1000)
Tuning(cap)

#test slip from 1 to 3
# MoveTime(0,0.3,0,4000)
# Tuning(cap, kp, ki, kd)
# MoveTime(0,0.2,0,2200)
# Tuning(cap, kp, ki, kd)