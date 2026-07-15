from robodk.robolink import *
from robodk.robomath import *
import time

RDK = Robolink()

ROBOT2_NAME = 'Robot2'
PROG1_NAME  = 'Program_R1'

DT = 0.003
HOLD_TICKS = 6

# LOCK J4: 0.0 = hard lock (točno), stavi npr. 2.0 ako želiš malo tolerancije
J4_LOCK_TOL = 0.0

robot2 = RDK.Item(ROBOT2_NAME, ITEM_TYPE_ROBOT)
prog1  = RDK.Item(PROG1_NAME, ITEM_TYPE_PROGRAM)

if not robot2.Valid(): raise Exception("Robot2 nije pronađen")
if not prog1.Valid():  raise Exception("Program_R1 nije pronađen")

# targets
targets = []
t0 = RDK.Item('Start_Robot2', ITEM_TYPE_TARGET)
if not t0.Valid(): raise Exception("Nema targeta 'Start_Robot2'")
targets.append(t0)

for i in range(1, 401):
    ti = RDK.Item(f"Line_{i:03d}", ITEM_TYPE_TARGET)
    if not ti.Valid():
        raise Exception(f"Nedostaje Line_{i:03d}")
    targets.append(ti)

print("Ukupno targeta:", len(targets))

# brzine
robot2.setSpeed(1200)
robot2.setAcceleration(5000)
robot2.setSpeedJoints(300)
robot2.setAccelerationJoints(900)

# TMP target (fallback)
tmp = RDK.Item('TMP_R1', ITEM_TYPE_TARGET)
if not tmp.Valid():
    tmp = RDK.AddTarget('TMP_R1')
tmp.setAsCartesianTarget()

# joints helper
def fnum(v):
    while isinstance(v, (list, tuple)):
        if len(v) == 0: return None
        v = v[0]
    try:
        return float(v)
    except:
        s = str(v).replace('[','').replace(']','').strip()
        try: return float(s)
        except: return None

def joints6(j):
    try:
        out = [fnum(j[i]) for i in range(6)]
        if any(v is None for v in out): return None
        return out
    except:
        return None

def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def clamp(val, center, tol):
    if tol <= 0.0:
        return center
    d = ang_diff(val, center)
    if d > tol:  return center + tol
    if d < -tol: return center - tol
    return center + d

# frame/tool za IK
T_frame = robot2.PoseFrame()
T_tool  = robot2.PoseTool()

# idi na Pocetak (XYZ + rotacija)
T_abs0 = targets[0].PoseAbs()
tmp.setPoseAbs(T_abs0)
robot2.MoveJ(tmp, blocking=True)

# start jointovi (za lock J4 + seed za IK)
j_start = joints6(robot2.Joints())
if j_start is None:
    raise Exception("Ne mogu očitati start jointove Robot2")

j_prev = j_start[:]

# start Robot1
prog1.RunProgram()
print("START: prati XYZ+ROT, LOCK samo J4")

t_next = time.time()
idx = 0
tick_in_point = 0

while prog1.Busy() and idx < len(targets):

    now = time.time()
    if now < t_next:
        time.sleep(t_next - now)
    t_next += DT

    # uzmi CIJELI target pose (XYZ + ROT)
    T_abs = targets[idx].PoseAbs()

    ok = False
    try:
        T = invH(T_frame) * T_abs * invH(T_tool)
        j_sol = robot2.SolveIK(T, j_prev)
        j_new = joints6(j_sol)
        if j_new is not None:
            # LOCK J4 (index 3)
            j_new[3] = clamp(j_new[3], j_start[3], J4_LOCK_TOL)

            robot2.setJoints(j_new)
            j_prev = j_new
            ok = True
    except:
        ok = False

    if not ok:
        # fallback: MoveJ na pozu targeta
        tmp.setPoseAbs(T_abs)
        robot2.MoveJ(tmp, blocking=False)

    tick_in_point += 1
    if tick_in_point >= HOLD_TICKS:
        tick_in_point = 0
        idx += 1

# --- DODANO: zaustavi R1 kad se Robot2 zaustavi (tj. kad završi ova petlja) ---
if prog1.Busy():
    try:
        prog1.Stop()
    except:
        pass
    try:
        prog1.WaitFinished()
    except:
        pass
# ------------------------------------------------------------------------------

print("KRAJ.")
