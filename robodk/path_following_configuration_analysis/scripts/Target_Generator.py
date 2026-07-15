from robodk.robolink import *
from robodk.robomath import *

RDK = Robolink()

START_NAME = 'Kut_4'
END_NAME   = 'Pocetak'
PREFIX     = 'Line_'
NPTS       = 100

START_NUM  = 301    # <-- nova numeracija: 201-400
UPDATE_IF_EXISTS = True   # <-- ako postoji Line_2xx, samo ga update-a

tA = RDK.Item(START_NAME, ITEM_TYPE_TARGET)
tB = RDK.Item(END_NAME, ITEM_TYPE_TARGET)
if not tA.Valid(): raise Exception(f"Ne postoji target: {START_NAME}")
if not tB.Valid(): raise Exception(f"Ne postoji target: {END_NAME}")

parent = tA.Parent()
if not parent.Valid():
    raise Exception("Pocetak nema Parent() -> stavi Pocetak pod frame/predmet u tree-u")

TA = tA.Pose()
TB = tB.Pose()

xa, ya, za, rxa, rya, rza = pose_2_xyzrpw(TA)
xb, yb, zb, _,   _,   _   = pose_2_xyzrpw(TB)

dx, dy, dz = xb-xa, yb-ya, zb-za
print(f"Pocetak XYZ: {xa:.3f}, {ya:.3f}, {za:.3f}")
print(f"Kraj    XYZ: {xb:.3f}, {yb:.3f}, {zb:.3f}")
print(f"Delta XYZ:   {dx:.3f}, {dy:.3f}, {dz:.3f}")

adx, ady, adz = abs(dx), abs(dy), abs(dz)
if adx >= ady and adx >= adz:
    axis = 'X'; a0, a1 = xa, xb
elif ady >= adx and ady >= adz:
    axis = 'Y'; a0, a1 = ya, yb
else:
    axis = 'Z'; a0, a1 = za, zb
print(f"Interpoliram SAMO os {axis}: {a0:.3f} -> {a1:.3f}")

for i in range(NPTS):
    s = i/(NPTS-1) if NPTS > 1 else 0.0
    aval = a0 + (a1-a0)*s

    x, y, z = xa, ya, za
    if axis == 'X': x = aval
    if axis == 'Y': y = aval
    if axis == 'Z': z = aval

    Ti = xyzrpw_2_pose([x, y, z, rxa, rya, rza])

    num = START_NUM + i
    name = f"{PREFIX}{num:03d}"  # Line_201 ... Line_400

    if UPDATE_IF_EXISTS:
        existing = RDK.Item(name, ITEM_TYPE_TARGET)
        if existing.Valid():
            existing.setAsCartesianTarget()
            existing.setPose(Ti)
            continue

    tgt = RDK.AddTarget(name, parent)
    tgt.setAsCartesianTarget()
    tgt.setPose(Ti)

print("Gotovo: napravljeno/azurirano targeta:", NPTS, f"({PREFIX}{START_NUM:03d}..{PREFIX}{(START_NUM+NPTS-1):03d})")
