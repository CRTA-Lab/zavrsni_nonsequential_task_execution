import csv
import os
import math

# =========================================================
# PODESI PUTANJU DO TVOG SNIMLJENOG CSV-a
# =========================================================
INPUT_CSV = r"C:/Users/xgami/Downloads/Završni_Stanice_Ispravak/Završni_Stanice_Ispravak/configs_line_following_detailed_notime.csv"

# Pragovi za detekciju "reset" / naglih skokova
RESET_JUMP_ANY_DEG   = 120.0   # ako bilo koji joint skoci vise od ovoga
RESET_JUMP_TOTAL_DEG = 260.0   # ili ako zbroj svih skokova prede ovo



def ang_diff(a, b):
    """Kutna razlika uz wrap (-180..180)."""
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


def percentile(values, p):
    if not values:
        return 0.0
    vals = sorted(values)
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * (p / 100.0)
    f = int(math.floor(k))
    c = int(math.ceil(k))
    if f == c:
        return vals[f]
    d0 = vals[f] * (c - k)
    d1 = vals[c] * (k - f)
    return d0 + d1


def read_samples(path):
    if not os.path.isfile(path):
        raise FileNotFoundError("Ne mogu naći ulazni CSV: " + path)

    rows = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f, delimiter=';')
        if reader.fieldnames is None:
            raise Exception("CSV nema header.")

        for r in reader:
            if not r:
                continue
            if r.get("config_idx", "").strip() == "":
                continue
            if r.get("r2_j1", "").strip() == "":
                continue

            cfg_idx = fint(r.get("config_idx"))
            sample_idx = fint(r.get("sample_idx"), 0)
            ik_ok = fint(r.get("ik_ok"), 0)

            if cfg_idx is None:
                continue

            r1 = [ffloat(r.get(f"r1_j{i}"), 0.0) for i in range(1, 7)]
            r2 = [ffloat(r.get(f"r2_j{i}"), 0.0) for i in range(1, 7)]

            if any(v is None for v in r1 + r2):
                continue

            rows.append({
                "config_idx": cfg_idx,
                "tmp_name": (r.get("tmp_name") or "").strip(),
                "sample_idx": sample_idx,
                "target_name": (r.get("target_name") or "").strip(),
                "target_idx": fint(r.get("target_idx"), 0),
                "ik_ok": ik_ok,
                "r1": r1,
                "r2": r2,
                "raw": r
            })

    if len(rows) == 0:
        raise Exception("CSV je pročitan, ali nema valjanih redova podataka.")

    return rows


def group_by_config(rows):
    groups = {}
    for row in rows:
        key = (row["config_idx"], row["tmp_name"])
        if key not in groups:
            groups[key] = []
        groups[key].append(row)

    # Sort po sample_idx (za replay i analizu skokova)
    for key in groups:
        groups[key].sort(key=lambda x: x["sample_idx"])
    return groups


def analyze_group(samples):
    ik_fail_count = 0
    jumps_any = []
    jumps_total = []
    reset_events = 0

    for s in samples:
        if s["ik_ok"] != 1:
            ik_fail_count += 1

    for i in range(1, len(samples)):
        prev_q = samples[i - 1]["r2"]
        curr_q = samples[i]["r2"]

        dj = [abs(ang_diff(curr_q[j], prev_q[j])) for j in range(6)]
        max_jump = max(dj)
        total_jump = sum(dj)

        jumps_any.append(max_jump)
        jumps_total.append(total_jump)

        if (max_jump >= RESET_JUMP_ANY_DEG) or (total_jump >= RESET_JUMP_TOTAL_DEG):
            reset_events += 1

    max_joint_jump = max(jumps_any) if jumps_any else 0.0
    max_total_jump = max(jumps_total) if jumps_total else 0.0
    p95_total = percentile(jumps_total, 95) if jumps_total else 0.0
    avg_total = (sum(jumps_total) / len(jumps_total)) if jumps_total else 0.0

    # Score: ogromna kazna na reset, pa IK fail, pa skokovi
    score = (
        reset_events * 1_000_000.0 +
        ik_fail_count * 10_000.0 +
        max_total_jump * 100.0 +
        p95_total * 10.0 +
        avg_total
    )

    return {
        "samples": len(samples),
        "reset_events": reset_events,
        "ik_fail_count": ik_fail_count,
        "max_joint_jump_deg": round(max_joint_jump, 4),
        "max_total_jump_deg": round(max_total_jump, 4),
        "p95_total_jump_deg": round(p95_total, 4),
        "avg_total_jump_deg": round(avg_total, 4),
        "score": round(score, 4),
    }


def safe_write_csv(path, header, rows):
    # Ako je file otvoren u Excelu -> fallback ime
    try_paths = [path]
    base, ext = os.path.splitext(path)
    try_paths.append(base + "_new" + ext)
    try_paths.append(base + "_out" + ext)

    last_err = None
    for p in try_paths:
        try:
            with open(p, "w", encoding="utf-8", newline="") as f:
                w = csv.writer(f, delimiter=';')
                w.writerow(header)
                for row in rows:
                    w.writerow(row)
            return p
        except PermissionError as e:
            last_err = e
            continue

    raise last_err if last_err else Exception("Ne mogu zapisati CSV.")


def main():
    rows = read_samples(INPUT_CSV)
    groups = group_by_config(rows)

    if len(groups) == 0:
        raise Exception("Nema grupa konfiguracija za analizu.")

    summary = []
    best_key = None
    best_metrics = None
    best_score = None

    for (cfg_idx, tmp_name), samples in sorted(groups.items(), key=lambda x: x[0][0]):
        m = analyze_group(samples)
        rec = {
            "config_idx": cfg_idx,
            "tmp_name": tmp_name,
            **m
        }
        summary.append(rec)

        if (best_score is None) or (m["score"] < best_score):
            best_score = m["score"]
            best_key = (cfg_idx, tmp_name)
            best_metrics = m

    # Folder outputa = isti folder gdje je input CSV
    out_dir = os.path.dirname(INPUT_CSV) if os.path.dirname(INPUT_CSV) else "."

    # 1) Summary CSV
    summary_header = [
        "config_idx", "tmp_name",
        "samples", "reset_events", "ik_fail_count",
        "max_joint_jump_deg", "max_total_jump_deg",
        "p95_total_jump_deg", "avg_total_jump_deg",
        "score"
    ]
    summary_rows = []
    for s in summary:
        summary_rows.append([
            s["config_idx"], s["tmp_name"],
            s["samples"], s["reset_events"], s["ik_fail_count"],
            s["max_joint_jump_deg"], s["max_total_jump_deg"],
            s["p95_total_jump_deg"], s["avg_total_jump_deg"],
            s["score"]
        ])

    summary_path = safe_write_csv(
        os.path.join(out_dir, "analysis_summary.csv"),
        summary_header,
        summary_rows
    )

    # 2) Best config only CSV (isti header kao original!)
    best_samples = groups[best_key]

    # Uzmi original header iz prvog raw reda
    raw_header = list(best_samples[0]["raw"].keys())

    best_rows = []
    for s in best_samples:
        best_rows.append([s["raw"].get(h, "") for h in raw_header])

    best_path = safe_write_csv(
        os.path.join(out_dir, "best_config_only.csv"),
        raw_header,
        best_rows
    )

    # Print rezultat
    print("\n================ REZULTAT ================")
    print("Najbolja konfiguracija:")
    print("  config_idx:", best_key[0])
    print("  tmp_name:", best_key[1])
    print("  samples:", best_metrics["samples"])
    print("  reset_events:", best_metrics["reset_events"])
    print("  ik_fail_count:", best_metrics["ik_fail_count"])
    print("  max_joint_jump_deg:", best_metrics["max_joint_jump_deg"])
    print("  max_total_jump_deg:", best_metrics["max_total_jump_deg"])
    print("  p95_total_jump_deg:", best_metrics["p95_total_jump_deg"])
    print("  avg_total_jump_deg:", best_metrics["avg_total_jump_deg"])
    print("  score:", best_metrics["score"])
    print("==========================================")

    print("\nSpremljeno:")
    print("  Summary :", summary_path)
    print("  Best CSV:", best_path)
    print("\nReplay koristi ovaj file -> best_config_only.csv")

if __name__ == "__main__":
    main()
