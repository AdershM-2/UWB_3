"""
Professional publication-quality figures for the DUNE paper.
Run from paper/figures/: python3 gen_pro_figures.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Polygon, FancyArrowPatch
from matplotlib.lines import Line2D

# ── Design tokens ─────────────────────────────────────────────────────────────
NAVY   = "#1B2E4A"
BLUE   = "#2A6EBB"
LBLUE  = "#D6E8F7"
AMBER  = "#C8790A"
AMBERLT= "#F5E0B0"
GREEN  = "#27855A"
GREENLT= "#C8EBD9"
LGREEN = "#C8EBD9"
RED    = "#C0392B"
REDLT  = "#F5C6C0"
GRAY   = "#6B7280"
LGRAY  = "#F0F2F4"
WHITE  = "#FFFFFF"
BLACK  = "#111827"

plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 8.5,
    "text.color": BLACK,
    "figure.facecolor": WHITE,
    "axes.facecolor": WHITE,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
})

# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 1 — Isometric anchor + tag topology
# ═══════════════════════════════════════════════════════════════════════════════

def iso_pt(x, y, z, sx=1.0, sz=1.35):
    """World (x,y,z) → isometric (xi, yi)."""
    xi = (x - y) * np.cos(np.radians(30)) * sx
    yi = ((x + y) * np.sin(np.radians(30))) * sx + z * sz
    return float(xi), float(yi)

def iso_polygon(ax, corners_xyz, fc, ec, lw=0.8, zorder=3, alpha=1.0):
    """Draw a filled polygon given a list of (x,y,z) corners."""
    pts = [iso_pt(*c) for c in corners_xyz]
    poly = Polygon(pts, closed=True, fc=fc, ec=ec,
                   lw=lw, zorder=zorder, alpha=alpha)
    ax.add_patch(poly)

def iso_box(ax, ox, oy, oz, W, D, H,
            c_top, c_left, c_right, ec=WHITE, lw=0.7, zo=4):
    """Isometric box: top + left (y-face) + right (x-face) faces."""
    # top
    iso_polygon(ax, [
        (ox,   oy,   oz+H), (ox+W, oy,   oz+H),
        (ox+W, oy+D, oz+H), (ox,   oy+D, oz+H)
    ], c_top, ec, lw=lw, zorder=zo+1)
    # left face (x=const, y-varying)
    iso_polygon(ax, [
        (ox,   oy,   oz), (ox,   oy+D, oz),
        (ox,   oy+D, oz+H), (ox,   oy,   oz+H)
    ], c_left, ec, lw=lw, zorder=zo)
    # right face (y=const, x-varying)
    iso_polygon(ax, [
        (ox,   oy,   oz), (ox+W, oy,   oz),
        (ox+W, oy,   oz+H), (ox,   oy,   oz+H)
    ], c_right, ec, lw=lw, zorder=zo)

def fig_anchor_topology():
    fig, ax = plt.subplots(figsize=(7.16, 4.6))
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor(WHITE)

    # ── Floor ──────────────────────────────────────────────────────────────
    floor_corners = [(0,0,0),(5,0,0),(5,5,0),(0,5,0)]
    iso_polygon(ax, floor_corners, fc="#F6E8C4", ec=AMBER, lw=1.5, zorder=1)

    # Grid
    for v in [1,2,3,4]:
        p1=iso_pt(v,0,0); p2=iso_pt(v,5,0)
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]], color=AMBER, lw=0.3, alpha=0.5, zorder=1)
        p1=iso_pt(0,v,0); p2=iso_pt(5,v,0)
        ax.plot([p1[0],p2[0]],[p1[1],p2[1]], color=AMBER, lw=0.3, alpha=0.5, zorder=1)

    # Floor border
    fp = [iso_pt(*c) for c in floor_corners]
    for i in range(4):
        a,b = fp[i], fp[(i+1)%4]
        ax.plot([a[0],b[0]],[a[1],b[1]], color=AMBER, lw=1.6, zorder=2,
                solid_capstyle="round")

    # ── Ground anchors D1–D4 ───────────────────────────────────────────────
    BW, BD, BH = 0.22, 0.22, 0.15     # anchor box dims
    PH        = 0.18                   # pole height above floor
    ground = {"A₁":(0,0),"A₂":(5,0),"A₃":(5,5),"A₄":(0,5)}
    label_offsets = {
        "A₁":(-0.45,-0.1), "A₂":(0.12,-0.1),
        "A₃":(0.12,-0.1),  "A₄":(-0.45,-0.1),
    }

    for name,(gx,gy) in ground.items():
        # pole
        pb = iso_pt(gx+BW/2, gy+BD/2, 0)
        pt = iso_pt(gx+BW/2, gy+BD/2, PH)
        ax.plot([pb[0],pt[0]],[pb[1],pt[1]],
                color=NAVY, lw=2.2, zorder=5, solid_capstyle="round")
        # box
        iso_box(ax, gx, gy, PH, BW, BD, BH,
                c_top="#4A90D9", c_left="#1A4E8A", c_right="#2259A4",
                ec=WHITE, lw=0.5, zo=6)
        # label
        lp = iso_pt(gx+BW/2, gy+BD/2, PH+BH+0.04)
        ox,oy = label_offsets[name]
        ax.text(lp[0]+ox, lp[1]+oy, name, fontsize=8,
                fontweight="bold", color=NAVY, ha="center", va="bottom",
                bbox=dict(boxstyle="round,pad=0.12", fc="white",
                          ec=NAVY, lw=0.6, alpha=0.92), zorder=10)

    # ── Elevated anchor A ──────────────────────────────────────────────────
    AX,AY,AZ = 2.5, 5.0, 1.2
    PB = iso_pt(AX+BW/2, AY+BD/2, 0)
    PT = iso_pt(AX+BW/2, AY+BD/2, AZ)
    ax.plot([PB[0],PT[0]],[PB[1],PT[1]],
            color=AMBER, lw=2.2, zorder=5, solid_capstyle="round",
            linestyle=(0,(5,2)))
    iso_box(ax, AX, AY, AZ, BW, BD, BH,
            c_top="#F0A030", c_left="#A06010", c_right="#C07820",
            ec=WHITE, lw=0.5, zo=7)
    lp = iso_pt(AX+BW/2, AY+BD/2, AZ+BH+0.06)
    ax.text(lp[0], lp[1]+0.04, "A  [elevated]",
            fontsize=8, fontweight="bold", color=AMBER,
            ha="center", va="bottom",
            bbox=dict(boxstyle="round,pad=0.12", fc="white",
                      ec=AMBER, lw=0.6, alpha=0.92), zorder=10)

    # Height annotation
    ANNOTX = AX - 0.55
    pa = iso_pt(ANNOTX, AY, 0)
    pb2 = iso_pt(ANNOTX, AY, AZ)
    ax.annotate("", xy=(pb2[0],pb2[1]), xytext=(pa[0],pa[1]),
                arrowprops=dict(arrowstyle="<->", color=AMBER,
                                lw=1.0, mutation_scale=9), zorder=8)
    mp = iso_pt(ANNOTX, AY, AZ/2)
    ax.text(mp[0]-0.1, mp[1], "1.2 m",
            fontsize=7.5, color=AMBER, ha="right", va="center")

    # ── Rover ─────────────────────────────────────────────────────────────
    RX,RY = 2.1, 2.3
    RW,RD,RH_box = 0.85, 0.50, 0.16
    iso_box(ax, RX, RY, 0, RW, RD, RH_box,
            c_top="#374151", c_left="#1C2530", c_right="#242F3D",
            ec="#6B7280", lw=0.7, zo=9)
    rp = iso_pt(RX+RW/2, RY+RD/2, RH_box+0.04)
    ax.text(rp[0], rp[1], "Rover",
            fontsize=7.5, color=WHITE, ha="center", va="bottom",
            fontweight="bold", zorder=11,
            bbox=dict(boxstyle="round,pad=0.10", fc="#374151",
                      ec="none", alpha=0.85))

    # Tags
    T1X,T1Y = RX+0.17, RY+RD/2
    T2X,T2Y = RX+RW-0.17, RY+RD/2
    for (tx,ty,tc,tlabel) in [
        (T1X,T1Y,GREEN,"T₁ (IMU)"),
        (T2X,T2Y,RED,  "T₂"),
    ]:
        tp = iso_pt(tx, ty, RH_box)
        ax.plot(tp[0], tp[1], "o", ms=7, color=tc, zorder=12,
                markeredgecolor=WHITE, markeredgewidth=0.8)
        off = -0.22 if tc==GREEN else 0.22
        ax.text(tp[0]+off, tp[1]+0.06, tlabel,
                fontsize=7, color=tc, ha="center", va="bottom",
                fontweight="bold", zorder=13)

    # Baseline arrow T1–T2
    bt1 = iso_pt(T1X, T1Y, RH_box+0.08)
    bt2 = iso_pt(T2X, T2Y, RH_box+0.08)
    ax.annotate("", xy=bt2, xytext=bt1,
                arrowprops=dict(arrowstyle="<->", color=GRAY,
                                lw=0.8, mutation_scale=8), zorder=11)
    bm = iso_pt((T1X+T2X)/2, (T1Y+T2Y)/2, RH_box+0.14)
    ax.text(bm[0], bm[1], "50 cm",
            fontsize=7, color=GRAY, ha="center", va="bottom", zorder=12)

    # ── Ranging beams ─────────────────────────────────────────────────────
    rover_c = iso_pt(RX+RW/2, RY+RD/2, RH_box/2)
    for name,(gx,gy) in ground.items():
        anch = iso_pt(gx+BW/2, gy+BD/2, PH+BH/2)
        ax.plot([rover_c[0],anch[0]],[rover_c[1],anch[1]],
                color=BLUE, lw=0.65, alpha=0.30, ls="--", zorder=3)
    ea = iso_pt(AX+BW/2, AY+BD/2, AZ+BH/2)
    ax.plot([rover_c[0],ea[0]],[rover_c[1],ea[1]],
            color=AMBER, lw=0.8, alpha=0.45, ls=(0,(4,2)), zorder=3)

    # ── Kinect ────────────────────────────────────────────────────────────
    KX,KY,KZ = 2.5, 2.0, 2.4
    kp = iso_pt(KX,KY,KZ)
    kf = iso_pt(KX,KY,0)
    ax.plot([kp[0],kf[0]],[kp[1],kf[1]],
            color=GRAY, lw=0.6, ls=":", alpha=0.4, zorder=3)
    ax.plot(kp[0], kp[1], marker="D", ms=8, color=GRAY,
            markeredgecolor=WHITE, markeredgewidth=0.7, zorder=10)
    ax.text(kp[0]+0.16, kp[1]+0.05, "Kinect v2\n(ceiling — GT)",
            fontsize=7, color=GRAY, ha="left", va="center", zorder=11,
            bbox=dict(boxstyle="round,pad=0.10", fc="white",
                      ec="#CCCCCC", lw=0.5, alpha=0.9))

    # AprilTag
    ap = iso_pt(RX+RW/2, RY, RH_box+0.05)
    ax.plot(ap[0], ap[1], marker="s", ms=5.5, color="#8B5CF6",
            markeredgecolor=WHITE, markeredgewidth=0.5, zorder=11)
    ax.text(ap[0], ap[1]-0.10, "AprilTag",
            fontsize=6.5, color="#8B5CF6", ha="center", va="top", zorder=12)

    # ── Dimension: floor ─────────────────────────────────────────────────
    d0 = iso_pt(0,-0.3,0); d1 = iso_pt(5,-0.3,0)
    ax.annotate("", xy=d1, xytext=d0,
                arrowprops=dict(arrowstyle="<->",color=GRAY,lw=0.9,
                                mutation_scale=9), zorder=2)
    dm = iso_pt(2.5,-0.3,0)
    ax.text(dm[0], dm[1]-0.12, "5 m",
            ha="center", va="top", fontsize=7.5, color=GRAY)

    d0 = iso_pt(5.3,0,0); d1 = iso_pt(5.3,5,0)
    ax.annotate("", xy=d1, xytext=d0,
                arrowprops=dict(arrowstyle="<->",color=GRAY,lw=0.9,
                                mutation_scale=9), zorder=2)
    dm2 = iso_pt(5.3,2.5,0)
    ax.text(dm2[0]+0.14, dm2[1], "5 m",
            ha="left", va="center", fontsize=7.5, color=GRAY)

    # ── Legend ────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(fc="#4A90D9", ec="none", label="Ground anchor  (z = 0 m)"),
        mpatches.Patch(fc="#F0A030", ec="none", label="Elevated anchor  A  (z = 1.2 m)"),
        mpatches.Patch(fc=GREEN,    ec="none", label="Tag 1  —  with BNO085 IMU"),
        mpatches.Patch(fc=RED,      ec="none", label="Tag 2"),
        mpatches.Patch(fc=GRAY,     ec="none", label="Kinect v2  (ground truth)"),
        Line2D([0],[0], color=BLUE,  lw=1.2, ls="--", label="UWB ranging beam"),
        Line2D([0],[0], color=AMBER, lw=1.2, ls=(0,(4,2)), label="Beam to elevated anchor"),
    ]
    ax.legend(handles=legend_handles, loc="lower left",
              fontsize=7.2, framealpha=0.97, edgecolor="#BBBBBB",
              fancybox=False, ncol=2,
              handlelength=1.8, handleheight=0.9,
              bbox_to_anchor=(-0.01, -0.02))

    ax.set_title(
        "DUNE Testbed: Five-Anchor Two-Tag UWB Setup  (5 m × 5 m Indoor Sandbed)",
        fontsize=9.5, fontweight="bold", color=BLACK, pad=5)

    ax.autoscale_view()
    xl = ax.get_xlim(); yl = ax.get_ylim()
    ax.set_xlim(xl[0]-0.4, xl[1]+0.6)
    ax.set_ylim(yl[0]-0.55, yl[1]+0.45)
    fig.tight_layout(pad=0.3)
    fig.savefig("anchor_topology.pdf", bbox_inches="tight", dpi=300)
    plt.close()
    print("  anchor_topology.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE 2 — System architecture pipeline (professional block diagram)
# ═══════════════════════════════════════════════════════════════════════════════

def bbox(ax, cx, cy, w, h, label, sublabel="",
         fc=LBLUE, ec=BLUE, tc=NAVY, tsize=8, lw=1.2,
         radius=0.08, zorder=4):
    rect = FancyBboxPatch(
        (cx-w/2, cy-h/2), w, h,
        boxstyle=f"round,pad={radius}",
        fc=fc, ec=ec, lw=lw, zorder=zorder)
    ax.add_patch(rect)
    if sublabel:
        ax.text(cx, cy+0.12, label, ha="center", va="center",
                fontsize=tsize, fontweight="bold", color=tc, zorder=zorder+1)
        ax.text(cx, cy-0.18, sublabel, ha="center", va="center",
                fontsize=6.8, color=tc, style="italic", zorder=zorder+1)
    else:
        ax.text(cx, cy, label, ha="center", va="center",
                fontsize=tsize, fontweight="bold", color=tc, zorder=zorder+1,
                multialignment="center")

def harrow(ax, x1, x2, y, color=NAVY, lw=1.3, label="", label_y_off=0.14):
    ax.annotate("", xy=(x2, y), xytext=(x1, y),
                arrowprops=dict(arrowstyle="-|>", color=color,
                                lw=lw, mutation_scale=10),
                zorder=5)
    if label:
        ax.text((x1+x2)/2, y+label_y_off, label,
                ha="center", va="bottom", fontsize=6.8,
                color=color, style="italic")

def varrow(ax, x, y1, y2, color=NAVY, lw=1.3, label="", label_x_off=0.12):
    ax.annotate("", xy=(x, y2), xytext=(x, y1),
                arrowprops=dict(arrowstyle="-|>", color=color,
                                lw=lw, mutation_scale=10),
                zorder=5)
    if label:
        ax.text(x+label_x_off, (y1+y2)/2, label,
                ha="left", va="center", fontsize=6.8,
                color=color, style="italic")

def layer_bg(ax, x0, x1, y0, y1, fc, ec, label, lw=1.0, zorder=1):
    rect = FancyBboxPatch((x0, y0), x1-x0, y1-y0,
                          boxstyle="round,pad=0.04",
                          fc=fc, ec=ec, lw=lw, zorder=zorder)
    ax.add_patch(rect)
    ax.text((x0+x1)/2, y1-0.13, label,
            ha="center", va="top", fontsize=8, fontweight="bold",
            color=ec, zorder=zorder+1)

def fig_system_pipeline():
    fig, ax = plt.subplots(figsize=(7.16, 4.0))
    ax.set_xlim(0, 14); ax.set_ylim(0, 8)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.patch.set_facecolor(WHITE)

    # ── Layer 1: Firmware ─────────────────────────────────────────────────
    layer_bg(ax, 0.2, 5.6, 4.7, 7.9,
             fc="#EEF4FB", ec=NAVY,
             label="FIRMWARE  (ESP32 + DW1000)", lw=1.1)

    BW, BH = 1.30, 0.88

    # DW1000 chip
    bbox(ax, 0.95, 6.3, BW, BH,
         "DW1000", "UWB Radio",
         fc=NAVY, ec=NAVY, tc=WHITE, tsize=8, zorder=4)

    # TWR Engine
    bbox(ax, 2.45, 6.3, BW, BH,
         "ADS-TWR", "Engine",
         fc=LBLUE, ec=BLUE, tc=NAVY, tsize=8, zorder=4)

    # NLOS power on same frame
    bbox(ax, 3.95, 5.38, BW, 0.70,
         "FP / RX Power\n(same frame)",
         fc="#FFF3E0", ec=AMBER, tc="#7A4800", tsize=7.5, zorder=4)

    # HostLink formatter
    bbox(ax, 2.45, 5.22, BW, 0.70,
         "HostLink\nFormatter",
         fc=LBLUE, ec=BLUE, tc=NAVY, tsize=7.5, zorder=4)

    # Arrows inside firmware
    harrow(ax, 0.95+BW/2, 2.45-BW/2, 6.3, color=BLUE, lw=1.1,
           label="ranges")
    # FP power feeds to formatter
    ax.annotate("", xy=(3.15, 5.22), xytext=(3.95-BW/2, 5.38),
                arrowprops=dict(arrowstyle="-|>", color=AMBER, lw=1.0,
                                mutation_scale=9), zorder=5)
    harrow(ax, 1.80, 2.45-BW/2, 5.22, color=BLUE, lw=1.0)

    # WiFi badge
    wifi_x, wifi_y = 4.90, 5.22
    wifi_rect = FancyBboxPatch((wifi_x-0.42, wifi_y-0.28), 0.84, 0.56,
                               boxstyle="round,pad=0.06",
                               fc="#E8F5E9", ec=GREEN, lw=1.0, zorder=4)
    ax.add_patch(wifi_rect)
    ax.text(wifi_x, wifi_y, "Wi-Fi\nUDP",
            ha="center", va="center", fontsize=7, color=GREEN,
            fontweight="bold", zorder=5)
    harrow(ax, 2.45+BW/2, wifi_x-0.42, 5.22, color=GREEN, lw=1.0,
           label="RTLS,v3 packet")

    # ── Transmission arrow ─────────────────────────────────────────────────
    ax.annotate("", xy=(5.9, 5.22), xytext=(5.33, 5.22),
                arrowprops=dict(arrowstyle="-|>", color=GREEN,
                                lw=1.3, mutation_scale=11), zorder=5)

    # ── Layer 2: Host Python ──────────────────────────────────────────────
    layer_bg(ax, 5.8, 13.8, 0.3, 7.9,
             fc="#F0F8F4", ec=GREEN,
             label="HOST PIPELINE  (Python)", lw=1.1)

    # Pipeline boxes — top row
    PY = 6.2
    boxes_top = [
        (6.75, PY, "Tier-2\nSpatial Map",   "#FFF8E7", AMBER,   "#7A4800"),
        (8.35, PY, "NLOS\nWeighting",        "#FEF0F0", RED,     "#8B0000"),
        (9.95, PY, "Weighted LM\nTrilatera.", LBLUE,   BLUE,    NAVY),
        (11.55, PY, "FusionEKF\n+ ZUPT/NIS",LGREEN,  GREEN,    "#1A5235"),
    ]
    BW2, BH2 = 1.30, 0.90
    for (bx, by, lbl, fc, ec, tc) in boxes_top:
        bbox(ax, bx, by, BW2, BH2, lbl,
             fc=fc, ec=ec, tc=tc, tsize=7.8, zorder=4)

    # top-row arrows
    for i in range(len(boxes_top)-1):
        x1 = boxes_top[i][0] + BW2/2
        x2 = boxes_top[i+1][0] - BW2/2
        labels = ["corrected\nranges", "gap + ranges", r"$\hat{p}_{ML}$, $R$"]
        harrow(ax, x1, x2, PY, color=GRAY, lw=1.1,
               label=labels[i], label_y_off=0.12)

    # ── IMU branch ────────────────────────────────────────────────────────
    IMU_X, IMU_Y = 11.55, 4.85
    bbox(ax, IMU_X, IMU_Y, BW2, 0.72,
         "IMU  (BNO085)\nRoll · Pitch",
         fc="#F3E8FF", ec="#7C3AED", tc="#4C1D95", tsize=7.5, zorder=4)
    varrow(ax, IMU_X, IMU_Y+0.36, PY-BH2/2,
           color="#7C3AED", lw=1.0, label="  φ, θ")

    # ── Dual-tag yaw branch ───────────────────────────────────────────────
    YAW_X, YAW_Y = 9.75, 4.25
    bbox(ax, YAW_X, YAW_Y, BW2*1.1, 0.72,
         "Dual-Tag Yaw\nψ = atan2(Δp₂−Δp₁)",
         fc="#EDE9FE", ec="#6D28D9", tc="#3B0764", tsize=7.5, zorder=4)

    # Arrow from EKF to dual-tag yaw
    ax.annotate("", xy=(9.95+BW2/2+0.10, YAW_Y),
                xytext=(11.55-BW2/2, YAW_Y),
                arrowprops=dict(arrowstyle="<-", color="#6D28D9",
                                lw=0.9, mutation_scale=9), zorder=5)
    ax.text(10.75, YAW_Y-0.02, r"$\hat{p}_{T1}, \hat{p}_{T2}$",
            ha="center", va="top", fontsize=6.8, color="#6D28D9", style="italic")

    # ── 6-DOF output ──────────────────────────────────────────────────────
    OUT_X, OUT_Y = 13.15, 5.22
    bbox(ax, OUT_X, OUT_Y, 1.20, 2.50,
         "6-DOF\nPose\nOutput",
         fc=GREEN, ec=GREEN, tc=WHITE, tsize=8.5, zorder=4)
    ax.text(OUT_X, OUT_Y-0.80,
            "(x, y, z)\n(φ, θ, ψ)\n@ 10 Hz",
            ha="center", va="center", fontsize=7.0,
            color=WHITE, style="italic", zorder=5)

    # Arrow from EKF to output
    harrow(ax, 11.55+BW2/2, OUT_X-0.60, PY,
           color=GREEN, lw=1.4, label="full 6-DOF")

    # Yaw → output vertical
    ax.annotate("", xy=(OUT_X-0.60, YAW_Y),
                xytext=(YAW_X+BW2*1.1/2, YAW_Y),
                arrowprops=dict(arrowstyle="-|>", color="#6D28D9",
                                lw=0.9, mutation_scale=9), zorder=5)
    ax.annotate("", xy=(OUT_X-0.60, PY-0.3),
                xytext=(OUT_X-0.60, YAW_Y),
                arrowprops=dict(arrowstyle="-|>", color="#6D28D9",
                                lw=0.9, mutation_scale=9), zorder=5)

    # ── Bottom: Tier-2 Map + Median labels ────────────────────────────────
    bbox(ax, 6.75, 4.5, BW2, 0.72,
         "Median\nPre-filter",
         fc=LGRAY, ec=GRAY, tc=BLACK, tsize=7.5, zorder=4)
    varrow(ax, 6.75, 4.86, PY-BH2/2, color=GRAY, lw=0.9, label="  smooth")

    # ── Section labels ────────────────────────────────────────────────────
    ax.text(2.9, 7.65, "§ IV – UWB Protocol",
            ha="center", fontsize=7, color=NAVY, style="italic")
    ax.text(9.8, 7.65, "§ V – NLOS   §VI – Calibration   §VII – State Estimation",
            ha="center", fontsize=7, color=GREEN, style="italic")

    ax.set_title("DUNE System Architecture: From DW1000 Radio to 6-DOF Pose",
                 fontsize=9.5, fontweight="bold", color=BLACK, pad=5)
    fig.tight_layout(pad=0.3)
    fig.savefig("system_pipeline.pdf", bbox_inches="tight", dpi=300)
    plt.close()
    print("  system_pipeline.pdf")


# ── Run ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("Generating professional figures...")
    fig_anchor_topology()
    fig_system_pipeline()
    print("Done.")


# ═══════════════════════════════════════════════════════════════════════════════
# FIGURE: NLOS Concept Sketch
# ═══════════════════════════════════════════════════════════════════════════════

def fig_nlos_concept():
    """Two-panel NLOS explanation:
    (a) Physical geometry — tag, obstacle, anchor, LOS vs. NLOS paths
    (b) CIR power profiles — how the gap is read from the DW1000 registers
    """
    fig = plt.figure(figsize=(7.16, 3.4))
    # manual layout: left panel wider, right panel has two stacked sub-axes
    ax_geo  = fig.add_axes([0.03, 0.10, 0.44, 0.80])   # geometry
    ax_los  = fig.add_axes([0.55, 0.55, 0.43, 0.36])   # CIR LOS
    ax_nlos = fig.add_axes([0.55, 0.10, 0.43, 0.36])   # CIR NLOS

    for a in [ax_geo, ax_los, ax_nlos]:
        a.set_facecolor(WHITE)

    # ── (a) Physical geometry ──────────────────────────────────────────────
    ax = ax_geo
    ax.set_xlim(0, 10); ax.set_ylim(0, 6)
    ax.axis("off")

    # --- Floor / ceiling lines ---
    ax.axhline(0.3,  color=LGRAY, lw=1.5, zorder=0)
    ax.axhline(5.7, color=LGRAY, lw=1.5, zorder=0)
    ax.text(9.5, 0.05, "floor", fontsize=7, color=GRAY, ha="right")

    # --- Tag (left) ---
    TAG_X, TAG_Y = 1.2, 3.0
    tag_circle = mpatches.Circle((TAG_X, TAG_Y), 0.38,
                                  fc=LBLUE, ec=BLUE, lw=1.5, zorder=5)
    ax.add_patch(tag_circle)
    ax.text(TAG_X, TAG_Y, "Tag", ha="center", va="center",
            fontsize=8, fontweight="bold", color=NAVY, zorder=6)
    ax.text(TAG_X, TAG_Y-0.72, "(UWB Rx)", ha="center",
            fontsize=7, color=GRAY, zorder=6)

    # --- Anchor (right) ---
    ANC_X, ANC_Y = 8.8, 3.0
    anc_rect = FancyBboxPatch((ANC_X-0.42, ANC_Y-0.42), 0.84, 0.84,
                               boxstyle="round,pad=0.06",
                               fc="#FFF0D0", ec=AMBER, lw=1.5, zorder=5)
    ax.add_patch(anc_rect)
    ax.text(ANC_X, ANC_Y, "Anchor", ha="center", va="center",
            fontsize=8, fontweight="bold", color="#7A4800", zorder=6)
    ax.text(ANC_X, ANC_Y-0.78, "(UWB Tx)", ha="center",
            fontsize=7, color=GRAY, zorder=6)

    # --- Obstacle wall ---
    WALL_X = 5.0
    wall = mpatches.Rectangle((WALL_X-0.22, 0.5), 0.44, 5.0,
                                fc="#D1D5DB", ec="#6B7280", lw=1.5, zorder=4)
    ax.add_patch(wall)
    ax.text(WALL_X, 5.35, "Obstacle", ha="center",
            fontsize=7.5, color=GRAY, fontweight="bold")
    ax.text(WALL_X, 5.05, "(wall / rover body)", ha="center",
            fontsize=6.5, color=GRAY)

    # --- True distance annotation ---
    ax.annotate("", xy=(ANC_X-0.42, TAG_Y-0.5),
                xytext=(TAG_X+0.38, TAG_Y-0.5),
                arrowprops=dict(arrowstyle="<->", color=GRAY,
                                lw=0.9, mutation_scale=8), zorder=3)
    ax.text((TAG_X+ANC_X)/2, TAG_Y-0.82, r"$d$  (true distance)",
            ha="center", va="top", fontsize=8, color=GRAY)

    # --- LOS path (blocked — dashed gray) ---
    ax.annotate("", xy=(ANC_X-0.42, TAG_Y),
                xytext=(TAG_X+0.38, TAG_Y),
                arrowprops=dict(arrowstyle="-|>", color="#9CA3AF",
                                lw=1.4, mutation_scale=10,
                                linestyle="dashed"), zorder=3)
    # "BLOCKED" badge on wall
    ax.text(WALL_X, TAG_Y, "✕", ha="center", va="center",
            fontsize=14, color=RED, fontweight="bold", zorder=7)
    ax.text(WALL_X, TAG_Y+0.55, "LOS blocked", ha="center",
            fontsize=7, color=RED)

    # --- NLOS path (reflected — solid colored curve) ---
    # Goes: tag → floor → anchor
    NLOS_DIP = 0.9   # dip point y
    nlos_xs = np.array([TAG_X+0.38, 3.5,  WALL_X,  6.5, ANC_X-0.42])
    nlos_ys = np.array([TAG_Y,      1.5,  1.0,     1.5,  ANC_Y])
    from scipy.interpolate import make_interp_spline
    t = np.linspace(0, 1, len(nlos_xs))
    t_fine = np.linspace(0, 1, 200)
    spl_x = make_interp_spline(t, nlos_xs, k=3)(t_fine)
    spl_y = make_interp_spline(t, nlos_ys, k=3)(t_fine)
    ax.plot(spl_x, spl_y, color=GREEN, lw=2.0, zorder=3,
            solid_capstyle="round")
    ax.annotate("", xy=(spl_x[-1], spl_y[-1]),
                xytext=(spl_x[-2], spl_y[-2]),
                arrowprops=dict(arrowstyle="-|>", color=GREEN,
                                lw=1.8, mutation_scale=12), zorder=4)

    # Reflection point marker
    refl_x, refl_y = spl_x[len(spl_x)//2], spl_y[len(spl_y)//2]
    ax.plot(refl_x, refl_y, "o", ms=5, color=GREEN,
            markeredgecolor=WHITE, markeredgewidth=0.7, zorder=6)

    # NLOS path label
    ax.text(4.5, 0.58, r"NLOS path:  $\rho_i = d + \Delta_i$",
            ha="center", fontsize=8, color=GREEN, fontweight="bold")
    ax.text(4.5, 0.20, r"$\Delta_i > 0$  (always positive bias)",
            ha="center", fontsize=7.5, color=GREEN)

    # Δ excess path annotation
    ax.annotate("", xy=(ANC_X-0.42, ANC_Y+0.65),
                xytext=(TAG_X+0.38, TAG_Y+0.65),
                arrowprops=dict(arrowstyle="<->", color=GREEN,
                                lw=0.9, mutation_scale=8,
                                linestyle="dashed"), zorder=3)
    ax.text((TAG_X+ANC_X)/2, TAG_Y+0.85,
            r"$\rho_i$  (measured, biased long)",
            ha="center", fontsize=7.5, color=GREEN)

    ax.set_title("(a)  Physical NLOS Scenario",
                 fontsize=9, fontweight="bold", color=BLACK, pad=3)

    # ── (b) CIR — LOS ──────────────────────────────────────────────────────
    t = np.linspace(0, 12, 600)

    def gaussian(t, mu, sig, amp):
        return amp * np.exp(-0.5*((t-mu)/sig)**2)

    def cir_los():
        # Strong first path, small echoes
        s  = gaussian(t, 4.0, 0.35, 1.0)
        s += gaussian(t, 6.5, 0.40, 0.18)
        s += gaussian(t, 8.0, 0.45, 0.10)
        return s

    def cir_nlos():
        # Weak first path, dominant multipath
        s  = gaussian(t, 4.0, 0.35, 0.22)
        s += gaussian(t, 6.8, 0.70, 0.95)
        s += gaussian(t, 8.5, 0.55, 0.45)
        return s

    # LOS panel
    ax2 = ax_los
    cir_l = cir_los()
    ax2.fill_between(t, 0, cir_l, color=LBLUE, alpha=0.55)
    ax2.plot(t, cir_l, color=BLUE, lw=1.4)

    FP_L  = cir_l.max()
    RX_L  = cir_l.max()          # for LOS they're nearly equal
    ax2.axhline(FP_L, color=GREEN, lw=1.1, ls="--",
                label=r"$P_\mathrm{fp}$")
    ax2.axhline(RX_L*1.05, color=RED, lw=1.1, ls=":",
                label=r"$P_\mathrm{rx}$")
    ax2.annotate("", xy=(11.5, RX_L*1.05), xytext=(11.5, FP_L),
                arrowprops=dict(arrowstyle="<->", color=GRAY,
                                lw=0.9, mutation_scale=7))
    ax2.text(11.7, (RX_L + RX_L*1.05)/2, "gap ≈ 0",
             fontsize=7, color=GRAY, va="center")
    ax2.text(4.0, FP_L*1.02, r"$F_1, F_2, F_3$  (first-path)",
             fontsize=7, color=BLUE, ha="center", va="bottom")
    ax2.set_title("(b)  LOS: $P_{\\rm rx} \\approx P_{\\rm fp}$",
                  fontsize=8.5, fontweight="bold", color=BLUE, pad=2)
    ax2.set_xlim(0,13); ax2.set_ylim(-0.05, 1.25)
    ax2.set_ylabel("CIR Power", fontsize=7.5)
    ax2.tick_params(labelsize=7)
    ax2.set_xticklabels([])
    ax2.legend(fontsize=7, loc="upper right", framealpha=0.9,
               edgecolor="#CCCCCC")
    ax2.spines[["top","right"]].set_visible(False)

    # NLOS panel
    ax3 = ax_nlos
    cir_n = cir_nlos()
    ax3.fill_between(t, 0, cir_n, color="#FFE8E8", alpha=0.65)
    ax3.plot(t, cir_n, color=RED, lw=1.4)

    FP_N  = gaussian(np.array([4.0]), 4.0, 0.35, 0.22)[0]
    RX_N  = cir_n.max()
    ax3.axhline(FP_N, color=GREEN, lw=1.1, ls="--",
                label=r"$P_\mathrm{fp}$ (attenuated)")
    ax3.axhline(RX_N, color=RED, lw=1.1, ls=":",
                label=r"$P_\mathrm{rx}$ (multipath)")
    # Gap double-arrow
    ax3.annotate("", xy=(11.5, RX_N), xytext=(11.5, FP_N),
                arrowprops=dict(arrowstyle="<->", color=RED,
                                lw=1.1, mutation_scale=7))
    ax3.text(11.7, (RX_N+FP_N)/2,
             r"$\mathrm{gap}_i$" "\n> 3 dB",
             fontsize=7, color=RED, va="center", fontweight="bold")
    # First-path label
    ax3.text(4.0, FP_N+0.06, "weak\nfirst path",
             fontsize=6.5, color=GREEN, ha="center", va="bottom")
    # Multipath hump label
    ax3.text(7.2, RX_N+0.06, "dominant\nmultipath",
             fontsize=6.5, color=RED, ha="center", va="bottom")
    ax3.set_title(r"(c)  NLOS: $P_{\rm rx} \gg P_{\rm fp}$  $\Rightarrow$  "
                  r"$\mathrm{gap}_i = P_{\rm rx} - P_{\rm fp} > 3\,\mathrm{dB}$",
                  fontsize=8.5, fontweight="bold", color=RED, pad=2)
    ax3.set_xlim(0,13); ax3.set_ylim(-0.05, 1.25)
    ax3.set_xlabel("Time of Arrival  →", fontsize=7.5)
    ax3.set_ylabel("CIR Power", fontsize=7.5)
    ax3.tick_params(labelsize=7)
    ax3.set_xticklabels([])
    ax3.legend(fontsize=7, loc="upper right", framealpha=0.9,
               edgecolor="#CCCCCC")
    ax3.spines[["top","right"]].set_visible(False)

    # Shared note
    fig.text(0.785, 0.03,
             "DW1000 registers: $F_1,F_2,F_3$ → $P_{\\rm fp}$;  "
             r"$P_{\rm rx}$ from total received power accumulator.",
             ha="center", fontsize=7, color=GRAY, style="italic")

    fig.savefig("nlos_concept.pdf", bbox_inches="tight", dpi=300)
    plt.close()
    print("  nlos_concept.pdf")


if __name__ == "__main__":
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("Generating NLOS concept figure...")
    fig_nlos_concept()
    print("Done.")
