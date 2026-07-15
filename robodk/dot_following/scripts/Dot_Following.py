from robodk.robolink import *
import time

RDK = Robolink()

robot2 = RDK.Item('Robot2', ITEM_TYPE_ROBOT)
touch  = RDK.Item('Touch_1', ITEM_TYPE_TARGET)
prog1  = RDK.Item('Program_R1', ITEM_TYPE_PROGRAM)

if not robot2.Valid(): raise Exception("Robot2 nije pronađen")
if not touch.Valid():  raise Exception("Touch_1 nije pronađen")
if not prog1.Valid():  raise Exception("Program_R1 nije pronađen")

# --- još malo brže (sigurno za simulaciju) ---
robot2.setSpeed(1200)          # mm/s
robot2.setAcceleration(5000)
robot2.setSpeedJoints(300)     # deg/s
robot2.setAccelerationJoints(900)

# 1) isto kao ručni klik
robot2.MoveJ(touch, blocking=True)

print("Robot2 na Touch_1. Pokrećem Robot1...")

# 2) pokreni Robot1
prog1.RunProgram()

print("Praćenje Touch_1 (200 Hz)")

# 3) HIGH-FREQ LOOP
DT = 0.005   # 200 Hz
t_next = time.time()

while prog1.Busy():
    now = time.time()
    if now < t_next:
        time.sleep(t_next - now)
    t_next += DT

    # non-stop "Go to Touch_1"
    robot2.MoveJ(touch, blocking=False)

print("Gotovo")
