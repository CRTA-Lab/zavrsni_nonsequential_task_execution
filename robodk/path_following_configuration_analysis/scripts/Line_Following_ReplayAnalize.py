from robodk.robolink import *
from robodk.robomath import *
import csv
import os
import time

RDK = Robolink()

ROBOT1_NAME = 'Robot1'
ROBOT2_NAME = 'Robot2'

# =========================================================
# PODESI PUTANJU DO "best_config_only.csv" (output analize)
# =========================================================
INPUT_BEST_CSV = r"C:/Users/xgami/Downloads/Završni_Stanice_Ispravak/Završni_Stanice_Ispravak/best_config_only.csv"

# STAVI ISTO KAO U SNIMAJU
CONTROL_DT = 0.003
LOG_EVERY_N_TICKS = 1

# Smooth replay:
# 0 = tocno sample po sample
# 1 = ubaci 1 interpolirani korak izmedu sampleova (glade)
# 2 = jos glade, ali moze biti teze za CPU
INTERP_SUBSTEPS = 1

# 1.0 = ista brzina, 0.5 = sporije 2x, 2.0 = brze 2x
PLAYBACK_SPEED = 1.0
# =========================================================


def ang_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def ffloat(x, default=None):
    try:
        return float(str(x).strip().replace(",", "."))
    except:
        return default


def fint(x, default=None):
    try:
        return int(float(str(x).strip()))
    except:
        return default


def joints_from_row(row, prefix):
    q = []
    for i in range(1, 7):
        v = ffloat(row.get(f"{prefix}_j{i}"), None)
        if v is None:
            return None
        q.append(v)
    return q


def read_best_csv(path):
    if not os.path.isfile(path):
        raise FileNotFoundError("Ne mogu naci best_config_only.csv: " + path)

    samples = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f, delimiter=';')
        if reader.fieldnames is None:
            raise Exception("CSV nema header.")

        for r in reader:
            if not r:
                continue
            if str(r.get("config_idx", "")).strip() == "":
                continue

            q1 = joints_from_row(r, "r1")
            q2 = joints_from_row(r, "r2")
            if q1 is None or q2 is None:
                continue

            samples.append({
                "config_idx": fint(r.get("config_idx"), 0),
                "tmp_name": (r.get("tmp_name") or "").strip(),
                "sample_idx": fint(r.get("sample_idx"), 0),
                "target_name": (r.get("target_name") or "").strip(),
                "target_idx": fint(r.get("target_idx"), 0),
                "ik_ok": fint(r.get("ik_ok"), 0),
                "r1": q1,
                "r2": q2
            })

    if len(samples) == 0:
        raise Exception("best_config_only.csv je prazan ili nema valjane podatke.")

    samples.sort(key=lambda x: x["sample_idx"])
    return samples


def interp_joint(q_prev, q_next, alpha):
    # Interpolacija uz kutni wrap
    q = []
    for i in range(6):
        d = ang_diff(q_next[i], q_prev[i])
        q.append(q_prev[i] + alpha * d)
    return q


def set_both_robots(robot1, robot2, q1, q2):
    robot1.setJoints(q1)
    robot2.setJoints(q2)


def main():
    robot1 = RDK.Item(ROBOT1_NAME, ITEM_TYPE_ROBOT)
    robot2 = RDK.Item(ROBOT2_NAME, ITEM_TYPE_ROBOT)

    if not robot1.Valid():
        raise Exception("Robot1 nije pronaden")
    if not robot2.Valid():
        raise Exception("Robot2 nije pronaden")

    samples = read_best_csv(INPUT_BEST_CSV)

    print("Ucitano sampleova:", len(samples))
    print("Replay config_idx:", samples[0]["config_idx"])
    print("Replay tmp_name  :", samples[0]["tmp_name"])

    try:
        prog1 = RDK.Item("Program_R1", ITEM_TYPE_PROGRAM)
        if prog1.Valid():
            try: prog1.Stop()
            except: pass
    except:
        pass

    # Postavi na prvi sample
    first = samples[0]
    set_both_robots(robot1, robot2, first["r1"], first["r2"])
    time.sleep(0.2)

    # Efektivni dt izmedu originalnih sampleova
    base_dt = (CONTROL_DT * LOG_EVERY_N_TICKS)

    step_dt = base_dt / float(INTERP_SUBSTEPS + 1)
    # Playback speed
    step_dt = step_dt / float(PLAYBACK_SPEED if PLAYBACK_SPEED > 0 else 1.0)

    print(f"Replay start: base_dt={base_dt:.6f}s, step_dt={step_dt:.6f}s, interp={INTERP_SUBSTEPS}")

    t_next = time.time()

    # Replay sampleova
    for i in range(1, len(samples)):
        prev_s = samples[i - 1]
        curr_s = samples[i]

        q1_prev, q2_prev = prev_s["r1"], prev_s["r2"]
        q1_next, q2_next = curr_s["r1"], curr_s["r2"]


        for k in range(1, INTERP_SUBSTEPS + 2):
            alpha = float(k) / float(INTERP_SUBSTEPS + 1)

            q1_cmd = interp_joint(q1_prev, q1_next, alpha) if INTERP_SUBSTEPS > 0 else q1_next
            q2_cmd = interp_joint(q2_prev, q2_next, alpha) if INTERP_SUBSTEPS > 0 else q2_next

            set_both_robots(robot1, robot2, q1_cmd, q2_cmd)

            now = time.time()
            if now < t_next:
                time.sleep(t_next - now)
            t_next += step_dt

        # mali status svakih 1000 sampleova
        if (i % 1000) == 0:
            print(f"Replay progress: {i}/{len(samples)-1}")


    last = samples[-1]
    set_both_robots(robot1, robot2, last["r1"], last["r2"])

    print("Replay KRAJ.")


if __name__ == "__main__":
    main()
