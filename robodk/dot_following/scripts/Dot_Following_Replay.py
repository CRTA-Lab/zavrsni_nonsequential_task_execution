from robodk.robolink import *
import time, csv, glob, os

RDK = Robolink()

robot1 = RDK.Item('Robot1', ITEM_TYPE_ROBOT)
robot2 = RDK.Item('Robot2', ITEM_TYPE_ROBOT)

if not robot1.Valid(): raise Exception("Robot1 nije pronađen")
if not robot2.Valid(): raise Exception("Robot2 nije pronađen")

# najnoviji record
pattern = r"C:\Users\xgami\Desktop\R1R2_Record_*.csv"
files = glob.glob(pattern)
if not files:
    raise Exception(f"Nema fileova: {pattern}")

CSV_FILE = max(files, key=os.path.getmtime)
print("Koristim file:", CSV_FILE)

def to_float(x):
    s = str(x).replace('[','').replace(']','').strip()
    return float(s)

# učitaj
data = []
with open(CSV_FILE, newline="") as f:
    r = csv.reader(f)
    next(r, None)
    for row in r:
        if not row:
            continue
        vals = [to_float(x) for x in row[:13]]  # t + 12 jointova
        data.append(vals)

if len(data) < 2:
    raise Exception("CSV prekratak")

print("REPLAY start (R1 + R2)")

t_start = time.time()
for row in data:
    t = row[0]
    r1 = row[1:7]
    r2 = row[7:13]

    while (time.time() - t_start) < t:
        time.sleep(0.0005)

    robot1.setJoints(r1)
    robot2.setJoints(r2)

print("REPLAY gotov.")
