from robodk.robolink import *
import time, csv
from datetime import datetime

RDK = Robolink()

robot1 = RDK.Item('Robot1', ITEM_TYPE_ROBOT)
robot2 = RDK.Item('Robot2', ITEM_TYPE_ROBOT)
touch  = RDK.Item('Touch_1', ITEM_TYPE_TARGET)
prog1  = RDK.Item('Program_R1', ITEM_TYPE_PROGRAM)

if not robot1.Valid(): raise Exception("Robot1 nije pronađen")
if not robot2.Valid(): raise Exception("Robot2 nije pronađen")
if not touch.Valid():  raise Exception("Touch_1 nije pronađen")
if not prog1.Valid():  raise Exception("Program_R1 nije pronađen")

DT = 0.005  # 200 Hz

# brzine (kao kod tebe)
robot2.setSpeed(1200)
robot2.setAcceleration(5000)
robot2.setSpeedJoints(300)
robot2.setAccelerationJoints(900)

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE = rf"C:\Users\xgami\Desktop\R1R2_Record_{stamp}.csv"

# start Robot2 na Touch_1
robot2.MoveJ(touch, blocking=True)

# start Robot1 program
prog1.RunProgram()
print("RECORD start:", CSV_FILE)

t0 = time.time()
t_next = time.time()
n = 0

with open(CSV_FILE, "w", newline="") as f:
    w = csv.writer(f)
    # header: t + R1(6) + R2(6)
    w.writerow(["t",
                "R1_j1","R1_j2","R1_j3","R1_j4","R1_j5","R1_j6",
                "R2_j1","R2_j2","R2_j3","R2_j4","R2_j5","R2_j6"])

    while prog1.Busy():
        now = time.time()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += DT

        # Robot2 prati Touch_1 (tvoj dokazano-radi način)
        robot2.MoveJ(touch, blocking=False)

        # snimi jointove oba robota
        j1 = robot1.Joints()
        j2 = robot2.Joints()

        j1_6 = [float(str(j1[i]).replace('[','').replace(']','').strip()) for i in range(6)]
        j2_6 = [float(str(j2[i]).replace('[','').replace(']','').strip()) for i in range(6)]

        w.writerow([time.time()-t0] + j1_6 + j2_6)
        n += 1

print("RECORD gotov. Uzoraka:", n)
print("Spremljeno u:", CSV_FILE)
