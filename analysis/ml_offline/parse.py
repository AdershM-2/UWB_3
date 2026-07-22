"""Parse RTLS v3 capture lines and set up geometry for the ML offline study."""
import json, re
import numpy as np

REPO = "/home/user/UWB_3"
SENTINEL = -2147483648

def parse_capture(path):
    """RTLS,v3,<t_ms>,<tag>,<n>,then per anchor: id,d_mm,rx,fp,q [,IMU,...]"""
    sweeps = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("RTLS,v3"):
            continue
        body = line.split(",IMU,")[0]
        f = body.split(",")
        t_ms, tag, n = int(f[2]), int(f[3]), int(f[4])
        rec = {"t": t_ms / 1000.0, "tag": tag, "ids": [], "d": [], "rx": [], "fp": [], "q": []}
        k = 5
        for _ in range(n):
            aid, dmm, rx, fp, q = f[k], f[k+1], f[k+2], f[k+3], f[k+4]
            k += 5
            rec["ids"].append(int(aid))
            rec["d"].append(int(dmm) / 1000.0)
            rec["rx"].append(float(rx))
            rec["fp"].append(float(fp))
            rec["q"].append(float(q))
        imu = None
        if ",IMU," in line:
            g = line.split(",IMU,")[1].split(",")
            imu = [float(x) for x in g]
        rec["imu"] = imu
        sweeps.append(rec)
    return sweeps

def load_layout(name):
    d = json.load(open(f"{REPO}/matlab/config/{name}"))
    ids = np.array([a["id"] for a in d["anchors"]])
    pos = np.array([[a["x"], a["y"], a["z"]] for a in d["anchors"]])
    return ids, pos

def load_bias():
    d = json.load(open(f"{REPO}/matlab/config/anchor_bias.json"))
    ab = {a["id"]: a["bias_m"] for a in d["anchors"]}
    tb = {t["id"]: t["bias_m"] for t in d["tags"]}
    return ab, tb

def sweep_matrix(sweeps, ids):
    """Rows aligned to layout ids: ranges (NaN absent), rx, fp, plus tag/t."""
    M = len(ids)
    idx = {int(a): i for i, a in enumerate(ids)}
    n = len(sweeps)
    R = np.full((n, M), np.nan); RX = np.full((n, M), np.nan); FP = np.full((n, M), np.nan)
    Q = np.full((n, M), np.nan)
    tag = np.zeros(n, int); t = np.zeros(n)
    for i, s in enumerate(sweeps):
        tag[i] = s["tag"]; t[i] = s["t"]
        for a, d, rx, fp, q in zip(s["ids"], s["d"], s["rx"], s["fp"], s["q"]):
            if a not in idx:
                continue
            c = idx[a]
            if rx == SENTINEL or fp == SENTINEL or not np.isfinite(d) or d <= 0:
                continue
            R[i, c] = d; RX[i, c] = rx; FP[i, c] = fp; Q[i, c] = q
    return dict(R=R, RX=RX, FP=FP, Q=Q, tag=tag, t=t)

if __name__ == "__main__":
    for cap in ["static_capture.txt", "static_verify_capture.txt"]:
        sw = parse_capture(f"{REPO}/matlab/results/{cap}")
        tags = sorted({s["tag"] for s in sw})
        print(f"{cap}: {len(sw)} sweeps, tags {tags}, "
              f"t {sw[0]['t']:.0f}..{sw[-1]['t']:.0f}s, "
              f"anchors seen {sorted({a for s in sw for a in s['ids']})}, "
              f"imu lines {sum(1 for s in sw if s['imu'])}")
        # which layout fits better? compare median per-anchor residual at the
        # robust fix under each candidate layout
        for lay in ["anchors.json", "anchors_rect6p3x3p0_2026-07-01.json"]:
            ids, pos = load_layout(lay)
            m = sweep_matrix(sw, ids)
            print(f"   layout {lay}: ids {ids.tolist()} avail% "
                  f"{np.round(100*np.mean(np.isfinite(m['R']),0)).astype(int).tolist()}")
