from robodk.robolink import *
from robodk.robomath import *
import time
import csv
import os

RDK = Robolink()

ROBOT1_NAME = 'Robot1'
ROBOT2_NAME = 'Robot2'
PROG1_NAME  = 'Program_R1'

# =========================================================
# PODESAVANJA
# =========================================================
CONTROL_DT = 0.003
TARGET_HOLD_TICKS = 5
LOG_EVERY_N_TICKS = 1
J4_LOCK_TOL = 0.0

RESET_PAUSE_S = 0.2
# =========================================================

# -------- Items --------
robot1 = RDK.Item(ROBOT1_NAME, ITEM_TYPE_ROBOT)
robot2 = RDK.Item(ROBOT2_NAME, ITEM_TYPE_ROBOT)
prog1  = RDK.Item(PROG1_NAME, ITEM_TYPE_PROGRAM)

if not robot1.Valid(): raise Exception("Robot1 nije pronađen")
if not robot2.Valid(): raise Exception("Robot2 nije pronađen")
if not prog1.Valid():  raise Exception("Program_R1 nije pronađen")

t_start_r1 = RDK.Item('Start_Robot1', ITEM_TYPE_TARGET)
t_start_r2 = RDK.Item('Start_Robot2', ITEM_TYPE_TARGET)

if not t_start_r1.Valid(): raise Exception("Nema targeta 'Start_Robot1'")
if not t_start_r2.Valid(): raise Exception("Nema targeta 'Start_Robot2'")

# targets
targets = [t_start_r2]
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

# -------- helpers --------
def fnum(v):
    while isinstance(v, (list, tuple)):
        if len(v) == 0:
            return None
        v = v[0]
    try:
        return float(v)
    except:
        s = str(v).replace('[', '').replace(']', '').strip()
        try:
            return float(s)
        except:
            return None

def joints6(j):
    try:
        out = [fnum(j[i]) for i in range(6)]
        if any(v is None for v in out):
            return None
        return out
    except:
        return None

def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0

def clamp(val, center, tol):
    if tol <= 0.0:
        return center
    d = ang_diff(val, center)
    if d > tol:
        return center + tol
    if d < -tol:
        return center - tol
    return center + d

def stop_all_soft():
    try: prog1.Stop()
    except: pass
    try: robot1.Stop()
    except: pass
    try: robot2.Stop()
    except: pass

def find_tmp_list():
    # Trazi TMP_R1, TMP_R2, TMP_R3...
    out = []
    i = 1
    while True:
        it = RDK.Item(f"TMP_R{i}", ITEM_TYPE_TARGET)
        if not it.Valid():
            break
        it.setAsCartesianTarget()  # sigurnost
        out.append(it)
        i += 1
    return out

# -------- CSV --------
station_path = RDK.getParam('PATH_OPENSTATION')

# novo ime da ako je stari CSV otvoren u Excelu
out_file = os.path.join(station_path, "configs_line_following_detailed_notime.csv") if station_path else "configs_line_following_detailed_notime.csv"

csv_file = open(out_file, mode='w', newline='', encoding='utf-8')
writer = csv.writer(csv_file, delimiter=';')

#
writer.writerow([
    "config_idx","tmp_name",
    "sample_idx",
    "target_name","target_idx",
    "ik_ok",
    "r1_j1","r1_j2","r1_j3","r1_j4","r1_j5","r1_j6",
    "r2_j1","r2_j2","r2_j3","r2_j4","r2_j5","r2_j6"
])

tmp_list = find_tmp_list()
if len(tmp_list) == 0:
    csv_file.close()
    raise Exception("Nema TMP_R1/TMP_R2/... targeta u stanici.")

print("TMP targeti:", [t.Name() for t in tmp_list])

# -------- Run one config--------
def run_one_config(cfg_idx, tmp):
    print(f"\n=== CONFIG {cfg_idx}/{len(tmp_list)} using {tmp.Name()} ===")

    # ---------------------------------------------------------
    # RUCNI POSTUPAK (automatiziran)
    # 1) Start_Robot1
    # 2) Start_Robot2 preko AKTUALNOG TMP_Ri
    # 3) Pokreni line following
    # ---------------------------------------------------------
    stop_all_soft()
    time.sleep(RESET_PAUSE_S)

    # 1) Start_Robot1
    robot1.MoveJ(t_start_r1, blocking=True)

    # 2) Start_Robot2 preko bas tog TMP_Ri
    T_abs0 = t_start_r2.PoseAbs()
    tmp.setPoseAbs(T_abs0)
    robot2.MoveJ(tmp, blocking=True)

    # frame/tool (kao u tvom kodu)
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
    hold_tick = 0
    sample_idx = 0

    while prog1.Busy() and idx < len(targets):
        now = time.time()
        if now < t_next:
            time.sleep(t_next - now)
        t_next += CONTROL_DT

        # uzmi cijeli target pose (XYZ + ROT)
        T_abs = targets[idx].PoseAbs()

        ik_ok = 0
        try:
            T = invH(T_frame) * T_abs * invH(T_tool)
            j_sol = robot2.SolveIK(T, j_prev)
            j_new = joints6(j_sol)

            if j_new is not None:
                #
                j_new[3] = clamp(j_new[3], j_start[3], J4_LOCK_TOL)

                robot2.setJoints(j_new)
                j_prev = j_new
                ik_ok = 1
            else:
                #
                tmp.setPoseAbs(T_abs)
                robot2.MoveJ(tmp, blocking=False)
                ik_ok = 0
        except:
            try:
                tmp.setPoseAbs(T_abs)
                robot2.MoveJ(tmp, blocking=False)
            except:
                pass
            ik_ok = 0

        #LOG
        if (sample_idx % LOG_EVERY_N_TICKS) == 0:
            j1 = joints6(robot1.Joints()) or [""]*6
            j2 = joints6(robot2.Joints()) or [""]*6
            writer.writerow([
                cfg_idx, tmp.Name(),
                sample_idx,
                targets[idx].Name(), idx,
                ik_ok,
                *j1,
                *j2
            ])
            csv_file.flush()

        sample_idx += 1

        # target advance (brzina prolaska kroz targete)
        hold_tick += 1
        if hold_tick >= TARGET_HOLD_TICKS:
            hold_tick = 0
            idx += 1

    # Zaustavi R1 kad Robot2 zavrsi
    if prog1.Busy():
        try:
            prog1.Stop()
        except:
            pass
        try:
            prog1.WaitFinished()
        except:
            pass

    stop_all_soft()

# -------- Main loop: TMP_R1, TMP_R2, TMP_R3... --------
for i, tmp_i in enumerate(tmp_list, start=1):
    try:
        run_one_config(i, tmp_i)
    except Exception as e:
        print("EXCEPTION:", str(e))
        stop_all_soft()
        time.sleep(RESET_PAUSE_S)
        continue

csv_file.close()
print("\nSpremljeno:", out_file)
print("KRAJ.")
