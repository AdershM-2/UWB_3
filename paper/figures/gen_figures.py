"""
Generate all technical figures for the DUNE paper.
Run from paper/figures/:  python3 gen_figures.py
Outputs: anchor_topology.pdf, dstwr_timing.pdf, nlos_diagram.pdf,
         system_pipeline.pdf, calibration_effect.pdf
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.lines import Line2D

# ── shared style ──────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "figure.dpi": 150,
    "pdf.fonttype": 42,
})
GRAY   = "#555555"
BLUE   = "#2166AC"
RED    = "#D6604D"
GREEN  = "#4DAC26"
ORANGE = "#F4A582"
LIGHT  = "#F7F7F7"

# ═══════════════════════════════════════════════════════════════════════════════
# 1.  ANCHOR TOPOLOGY  (top view + side view)
# ═══════════════════════════════════════════════════════════════════════════════
def fig_anchor_topology():
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.4),
                             gridspec_kw={"width_ratios": [1.15, 0.85]})

    # --- TOP VIEW ---
    ax = axes[0]
    ax.set_aspect("equal")
    ax.set_xlim(-0.4, 5.4); ax.set_ylim(-0.4, 5.6)
    ax.set_facecolor("#FAFAF5")

    # sandbed outline
    sand = mpatches.FancyBboxPatch((0, 0), 5, 5,
        boxstyle="round,pad=0.05", linewidth=1.5,
        edgecolor="#8B6914", facecolor="#F5DEB3", zorder=0)
    ax.add_patch(sand)

    # grid ticks
    for v in [1, 2, 3, 4]:
        ax.axvline(v, color="#C8A96E", lw=0.4, zorder=1)
        ax.axhline(v, color="#C8A96E", lw=0.4, zorder=1)

    # anchor positions: D1(0,0), D2(0,5), D3(5,0), D4(5,5) ground; A at (2.5,5) top; D5(5,5) elevated
    anchors_ground = {
        "D1\n(0x01)": (0.0, 0.0),
        "D2\n(0x02)": (0.0, 5.0),
        "D3\n(0x03)": (5.0, 0.0),
        "D4\n(0x04)": (5.0, 5.0),
    }
    anchor_high = {"A\n(0x05)": (2.5, 5.0)}   # elevated — plotted differently

    for name, (x, y) in anchors_ground.items():
        ax.plot(x, y, "s", ms=9, color=BLUE, zorder=5,
                markeredgecolor="white", markeredgewidth=0.8)
        offset = {"D1\n(0x01)": (-0.42, -0.30),
                  "D2\n(0x02)": (-0.42,  0.12),
                  "D3\n(0x03)": ( 0.08, -0.30),
                  "D4\n(0x04)": ( 0.08,  0.12)}[name]
        ax.text(x + offset[0], y + offset[1], name.replace("\n", " "),
                fontsize=7, color=BLUE, va="top")

    # elevated anchor A at top-center (show with different marker)
    ax.plot(2.5, 5.3, "^", ms=11, color=RED, zorder=5,
            markeredgecolor="white", markeredgewidth=0.8)
    ax.text(2.5, 5.45, "A (0x05)\n[elevated, 1.2 m]",
            fontsize=7, color=RED, ha="center", va="bottom")

    # rover at centre with two tags
    rover_x, rover_y = 2.5, 2.5
    tag1 = (2.5 - 0.25, 2.5)
    tag2 = (2.5 + 0.25, 2.5)

    rover_rect = mpatches.FancyBboxPatch(
        (rover_x - 0.45, rover_y - 0.35), 0.9, 0.7,
        boxstyle="round,pad=0.04", linewidth=1.2,
        edgecolor=GRAY, facecolor="white", zorder=4)
    ax.add_patch(rover_rect)
    ax.plot(*tag1, "o", ms=7, color=GREEN, zorder=6,
            markeredgecolor="white", markeredgewidth=0.7)
    ax.plot(*tag2, "o", ms=7, color=ORANGE, zorder=6,
            markeredgecolor="white", markeredgewidth=0.7)
    ax.text(tag1[0], tag1[1] + 0.18, "T1", fontsize=6.5,
            ha="center", color=GREEN, fontweight="bold")
    ax.text(tag2[0], tag2[1] + 0.18, "T2", fontsize=6.5,
            ha="center", color="#C45900", fontweight="bold")
    ax.annotate("", xy=tag2, xytext=tag1,
                arrowprops=dict(arrowstyle="<->", color=GRAY, lw=0.8))
    ax.text(2.5, 2.22, "50 cm", fontsize=6.5, ha="center", color=GRAY)

    # ranging lines from rover to each anchor
    for name, (x, y) in anchors_ground.items():
        ax.plot([rover_x, x], [rover_y, y], "--",
                color=BLUE, lw=0.6, alpha=0.5, zorder=2)
    ax.plot([rover_x, 2.5], [rover_y, 5.3], "--",
            color=RED, lw=0.6, alpha=0.5, zorder=2)

    # dimension annotations
    ax.annotate("", xy=(5, -0.35), xytext=(0, -0.35),
                arrowprops=dict(arrowstyle="<->", color=GRAY, lw=0.8))
    ax.text(2.5, -0.38, "5 m", ha="center", va="top", fontsize=7, color=GRAY)
    ax.annotate("", xy=(-0.35, 5), xytext=(-0.35, 0),
                arrowprops=dict(arrowstyle="<->", color=GRAY, lw=0.8))
    ax.text(-0.38, 2.5, "5 m", ha="right", va="center", fontsize=7,
            color=GRAY, rotation=90)

    ax.set_xticks([0, 1, 2, 3, 4, 5])
    ax.set_yticks([0, 1, 2, 3, 4, 5])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("(a) Top view — 5 m × 5 m sandbed", fontweight="bold")

    legend_elements = [
        Line2D([0], [0], marker="s", color="w", markerfacecolor=BLUE,
               markersize=8, label="Ground anchor (z = 0 m)"),
        Line2D([0], [0], marker="^", color="w", markerfacecolor=RED,
               markersize=8, label="Elevated anchor A (z = 1.2 m)"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=GREEN,
               markersize=7, label="Tag 1 (IMU)"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=ORANGE,
               markersize=7, label="Tag 2"),
    ]
    ax.legend(handles=legend_elements, loc="lower center",
              bbox_to_anchor=(0.5, -0.38), ncol=2, framealpha=0.9,
              edgecolor="#CCCCCC", fontsize=7)

    # --- SIDE VIEW ---
    ax2 = axes[1]
    ax2.set_aspect("equal")
    ax2.set_xlim(-0.3, 5.3); ax2.set_ylim(-0.2, 1.8)
    ax2.set_facecolor("#FAFAF5")

    # sandbed floor
    floor = mpatches.FancyBboxPatch(
        (0, -0.15), 5, 0.15,
        boxstyle="square,pad=0", linewidth=1,
        edgecolor="#8B6914", facecolor="#F5DEB3")
    ax2.add_patch(floor)
    ax2.text(2.5, -0.28, "Sandy floor", ha="center", fontsize=7, color="#8B6914")

    # ground anchors at 4 corners (project to side: x=0 and x=5)
    for xpos, label in [(0, "D1/D2"), (5, "D3/D4")]:
        ax2.plot(xpos, 0.0, "s", ms=9, color=BLUE, zorder=5,
                 markeredgecolor="white", markeredgewidth=0.8)
        ax2.annotate(f"z = 0 m\n{label}", xy=(xpos, 0.02),
                     xytext=(xpos + (0.3 if xpos == 0 else -0.3), 0.25),
                     fontsize=6.5, color=BLUE, ha="center",
                     arrowprops=dict(arrowstyle="-", color=BLUE, lw=0.6))

    # elevated anchor A
    ax2.plot(2.5, 1.2, "^", ms=11, color=RED, zorder=5,
             markeredgecolor="white", markeredgewidth=0.8)
    ax2.annotate("A  z = 1.2 m", xy=(2.5, 1.22),
                 xytext=(3.6, 1.35), fontsize=7, color=RED,
                 arrowprops=dict(arrowstyle="-", color=RED, lw=0.6))
    ax2.plot([2.5, 2.5], [0, 1.2], ":", color=RED, lw=0.8, alpha=0.6)
    ax2.annotate("", xy=(2.7, 1.2), xytext=(2.7, 0.0),
                 arrowprops=dict(arrowstyle="<->", color=RED, lw=0.8))
    ax2.text(2.75, 0.6, "1.2 m", fontsize=6.5, color=RED, va="center")

    # rover
    rover_h = 0.22
    rover_body = mpatches.FancyBboxPatch(
        (2.0, 0.0), 1.0, rover_h,
        boxstyle="round,pad=0.03", linewidth=1,
        edgecolor=GRAY, facecolor="white", zorder=3)
    ax2.add_patch(rover_body)
    ax2.text(2.5, rover_h / 2, "Rover", ha="center", va="center",
             fontsize=7, color=GRAY)

    # ranging lines
    for xpos in [0, 5]:
        ax2.plot([2.5, xpos], [rover_h, 0.0], "--",
                 color=BLUE, lw=0.7, alpha=0.5)
    ax2.plot([2.5, 2.5], [rover_h, 1.2], "--",
             color=RED, lw=0.7, alpha=0.5)

    # kinect
    ax2.plot(2.5, 1.6, "D", ms=8, color="#555555", zorder=5,
             markeredgecolor="white", markeredgewidth=0.8)
    ax2.text(3.3, 1.60, "Kinect v2\n(ceiling, GT)", fontsize=6.5, color=GRAY,
             va="center")

    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")
    ax2.set_xticks([0, 1, 2, 3, 4, 5])
    ax2.set_yticks([0.0, 0.4, 0.8, 1.2, 1.6])
    ax2.set_title("(b) Side view — anchor heights", fontweight="bold")

    fig.tight_layout(pad=0.8)
    fig.savefig("anchor_topology.pdf", bbox_inches="tight")
    plt.close(fig)
    print("  anchor_topology.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
# 2.  DS-TWR TIMING LADDER DIAGRAM
# ═══════════════════════════════════════════════════════════════════════════════
def fig_dstwr_timing():
    fig, ax = plt.subplots(figsize=(6.5, 4.0))
    ax.set_xlim(0, 10); ax.set_ylim(0, 10)
    ax.axis("off")

    # timeline bars
    TAG_X, ANC_X = 1.5, 8.5
    ax.plot([TAG_X, TAG_X], [0.5, 9.5], color=BLUE,   lw=2.5, solid_capstyle="round")
    ax.plot([ANC_X, ANC_X], [0.5, 9.5], color=GREEN, lw=2.5, solid_capstyle="round")
    ax.text(TAG_X, 9.8, "TAG", ha="center", fontsize=10,
            fontweight="bold", color=BLUE)
    ax.text(ANC_X, 9.8, "ANCHOR", ha="center", fontsize=10,
            fontweight="bold", color=GREEN)

    # events: (y_tag, y_anc, label_tag, label_anc, msg_label, msg_color)
    events = [
        (8.5, 7.0, r"$T_{PS}$", r"$T_{PR}$", "[1] POLL", BLUE),
        (7.0, 5.5, r"$T_{AS}$", r"$T_{PR}$", "[2] POLL ACK", GREEN),
        (5.5, 4.0, r"$T_{RS}$", r"$T_{AR}$", r"[3] RANGE $(T_{PS},T_{PR},T_{RS})$", BLUE),
        (4.0, 2.8, None,       None,         "[4] RANGE REPORT (distance $d$)", GREEN),
    ]

    for (y1, y2, lbl_tag, lbl_anc, msg, col) in events:
        if col == BLUE:   # tag → anchor
            ax.annotate("", xy=(ANC_X - 0.15, y2), xytext=(TAG_X + 0.15, y1),
                        arrowprops=dict(arrowstyle="-|>", color=col,
                                        lw=1.5, mutation_scale=14))
            mid_x = (TAG_X + ANC_X) / 2
            mid_y = (y1 + y2) / 2
            ax.text(mid_x, mid_y + 0.18, msg, ha="center", fontsize=7.5,
                    color=col, fontweight="bold", rotation=-15)
        else:             # anchor → tag
            ax.annotate("", xy=(TAG_X + 0.15, y2), xytext=(ANC_X - 0.15, y1),
                        arrowprops=dict(arrowstyle="-|>", color=col,
                                        lw=1.5, mutation_scale=14))
            mid_x = (TAG_X + ANC_X) / 2
            mid_y = (y1 + y2) / 2
            ax.text(mid_x, mid_y + 0.18, msg, ha="center", fontsize=7.5,
                    color=col, fontweight="bold", rotation=15)

        if lbl_tag:
            ax.plot(TAG_X, y1, "o", ms=6, color=BLUE, zorder=5)
            ax.text(TAG_X - 0.25, y1, lbl_tag, ha="right", va="center",
                    fontsize=8.5, color=BLUE)
        if lbl_anc:
            ax.plot(ANC_X, y2, "o", ms=6, color=GREEN, zorder=5)
            ax.text(ANC_X + 0.25, y2, lbl_anc, ha="left", va="center",
                    fontsize=8.5, color=GREEN)

    # interval braces on tag side
    brace_x = TAG_X - 0.7
    for (y_start, y_end, lbl) in [
        (8.5, 7.0, r"$R_1 = T_{PR}-T_{PS}$"),
        (7.0, 5.5, r"$P_1 = T_{AS}-T_{PR_A}$"),
        (5.5, 4.0, r"$R_2 = T_{AR}-T_{AS}$"),
    ]:
        ax.annotate("", xy=(brace_x, y_end), xytext=(brace_x, y_start),
                    arrowprops=dict(arrowstyle="<->", color=GRAY, lw=0.8))
        ax.text(brace_x - 0.1, (y_start + y_end) / 2, lbl,
                ha="right", va="center", fontsize=7, color=GRAY)

    # ToF formula at bottom
    ax.text(5, 1.5,
            r"$\hat{\tau} = \dfrac{R_1 R_2 - P_1 P_2}{R_1 + R_2 + P_1 + P_2}$"
            "\n(clock-skew cancels to first order)",
            ha="center", va="center", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#EEF4FF",
                      edgecolor=BLUE, lw=1.0))

    ax.set_title("Double-Sided Two-Way Ranging (DS-TWR) message exchange",
                 fontsize=9.5, fontweight="bold", pad=4)
    fig.tight_layout()
    fig.savefig("dstwr_timing.pdf", bbox_inches="tight")
    plt.close(fig)
    print("  dstwr_timing.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
# 3.  NLOS GAP DIAGRAM  (conceptual FP vs RX power histogram)
# ═══════════════════════════════════════════════════════════════════════════════
def fig_nlos_diagram():
    rng = np.random.default_rng(42)
    # Synthetic gap distributions (dB)
    los_gaps  = rng.normal(1.2, 0.6, 800).clip(0, 6)
    nlos_gaps = rng.normal(8.5, 2.0, 400).clip(2, 18)

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.0))

    # --- Histogram ---
    ax = axes[0]
    bins = np.linspace(0, 18, 36)
    ax.hist(los_gaps,  bins=bins, color=BLUE,  alpha=0.75, label="LOS links",  density=True)
    ax.hist(nlos_gaps, bins=bins, color=RED,   alpha=0.75, label="NLOS links", density=True)
    ax.axvline(3, color=GRAY,   lw=1.5, ls="--", label=r"$\theta_\mathrm{soft}=3\,\mathrm{dB}$")
    ax.axvline(6, color="black", lw=1.5, ls=":",  label=r"$\theta_\mathrm{hard}=6\,\mathrm{dB}$")
    ax.set_xlabel(r"NLOS gap $g_i = P_\mathrm{rx} - P_\mathrm{fp}$  (dB)")
    ax.set_ylabel("Probability density")
    ax.set_title("(a) LOS vs NLOS gap distribution\n(synthetic — replace with hardware data)",
                 fontsize=8)
    ax.legend(fontsize=7)
    ax.set_xlim(0, 18)

    # --- Weight curve ---
    ax2 = axes[1]
    g = np.linspace(0, 18, 300)
    w = np.maximum(0.05, 10 ** (-np.maximum(0, g - 3) / 10))
    ax2.plot(g, w, color=BLUE, lw=2.0, label=r"$w_i = 10^{-\max(0,\,g_i-3)/10}$")
    ax2.axhline(0.05, color=GRAY, lw=1.2, ls="--", label=r"$w_\min = 0.05$")
    ax2.axvline(3, color=GRAY,   lw=1.2, ls="--")
    ax2.axvline(6, color="black", lw=1.2, ls=":")
    ax2.fill_between(g, w, 0.05, where=(w > 0.05), alpha=0.12, color=BLUE)
    ax2.set_xlabel(r"NLOS gap $g_i$ (dB)")
    ax2.set_ylabel(r"Anchor weight $w_i$")
    ax2.set_title("(b) LM weight as function of NLOS gap", fontsize=8)
    ax2.set_xlim(0, 18); ax2.set_ylim(0, 1.05)
    ax2.legend(fontsize=7)

    fig.tight_layout(pad=0.8)
    fig.savefig("nlos_diagram.pdf", bbox_inches="tight")
    plt.close(fig)
    print("  nlos_diagram.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
# 4.  SYSTEM PIPELINE BLOCK DIAGRAM
# ═══════════════════════════════════════════════════════════════════════════════
def fig_system_pipeline():
    fig, ax = plt.subplots(figsize=(7.0, 2.8))
    ax.set_xlim(0, 14); ax.set_ylim(0, 4)
    ax.axis("off")

    def block(x, y, w, h, text, color, tsize=8):
        rect = FancyBboxPatch((x - w/2, y - h/2), w, h,
            boxstyle="round,pad=0.12", linewidth=1.2,
            edgecolor=color, facecolor=color + "22")
        ax.add_patch(rect)
        ax.text(x, y, text, ha="center", va="center",
                fontsize=tsize, color=color, fontweight="bold",
                multialignment="center")

    def arrow(x1, x2, y=2.0, label="", color=GRAY):
        ax.annotate("", xy=(x2, y), xytext=(x1, y),
                    arrowprops=dict(arrowstyle="-|>", color=color,
                                    lw=1.3, mutation_scale=12))
        if label:
            ax.text((x1 + x2) / 2, y + 0.25, label,
                    ha="center", fontsize=6.5, color=color)

    # blocks
    block(1.3, 2.0, 2.2, 1.4, "DW1000\nFirmware\n(ESP32)", BLUE)
    block(3.8, 2.0, 2.2, 1.4, "Wire format\nRTLS,v3\nUDP / serial", GRAY)
    block(6.3, 3.2, 2.2, 0.9, "Tier-2 map\n+ Median filter", GREEN)
    block(6.3, 0.9, 2.2, 0.9, "IMU\n(BNO085)", "#9C5E00")
    block(8.8, 2.0, 2.2, 1.4, "Weighted LM\nMultilateration\n+ MAD gate", RED)
    block(11.4, 2.0, 2.2, 1.4, "Singer EKF\n+ ZUPT\n+ EMA", "#6B2D8B")
    block(13.5, 2.0, 0.8, 1.4, "6-DOF\nPose", GREEN)

    # arrows
    arrow(2.4, 2.7, y=2.0, label="Raw ranges\n+ FP/RX power")
    arrow(4.9, 5.2, y=2.0, label="Per-anchor\nranges")
    arrow(5.2, 5.2, y=3.2, color=GREEN)  # vertical into LM
    ax.annotate("", xy=(7.7, 2.5), xytext=(6.3, 2.7),
                arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=1.0))
    ax.annotate("", xy=(7.7, 1.5), xytext=(6.3, 1.0),
                arrowprops=dict(arrowstyle="-|>", color="#9C5E00", lw=1.0))
    arrow(9.9,  10.2, y=2.0, label=r"$\hat{p}_{ML}$, $R$")
    arrow(12.5, 13.1, y=2.0, label=r"$(x,y,z,\phi,\theta,\psi)$")

    # labels
    ax.text(1.3, 3.0, "Firmware layer", ha="center", fontsize=6.5, color=BLUE, style="italic")
    ax.text(8.0, 3.8, "Host Python pipeline", ha="center", fontsize=7.5,
            color=GRAY, fontweight="bold")
    ax.plot([2.75, 2.75], [0.2, 3.8], ":", color=GRAY, lw=0.8)

    ax.set_title("DUNE system architecture: firmware → Python host → 6-DOF pose",
                 fontsize=9, fontweight="bold", pad=4)
    fig.tight_layout()
    fig.savefig("system_pipeline.pdf", bbox_inches="tight")
    plt.close(fig)
    print("  system_pipeline.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
# 5.  CALIBRATION EFFECT  (before/after bar chart — synthetic placeholder)
# ═══════════════════════════════════════════════════════════════════════════════
def fig_calibration_effect():
    rng = np.random.default_rng(7)
    labels = ["No calib.\n(factory)", "Tier-1\n(antenna delay)", "Tier-1 + Tier-2\n(+ spatial map)"]
    # Synthetic RMSE values — replace with hardware data
    means  = [35.2, 8.4, 4.1]   # cm
    stds   = [ 8.1, 2.1, 0.9]   # cm
    colors = [RED, ORANGE, GREEN]

    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    x = np.arange(len(labels))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color=colors,
                  edgecolor="white", linewidth=1.2, alpha=0.85,
                  error_kw=dict(elinewidth=1.2, ecolor=GRAY))

    for bar, m, s in zip(bars, means, stds):
        ax.text(bar.get_x() + bar.get_width()/2, m + s + 0.8,
                f"{m:.1f} cm", ha="center", fontsize=8, fontweight="bold")

    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("2D Position RMSE (cm)")
    ax.set_title("Calibration ablation — 2D RMSE\n"
                 r"{\small (values are \textbf{placeholder} — replace with hardware data)}",
                 fontsize=9)
    ax.set_ylim(0, 52)
    ax.axhline(5, color=GRAY, lw=1.0, ls="--", label="5 cm target")
    ax.legend(fontsize=8)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    fig.savefig("calibration_effect.pdf", bbox_inches="tight")
    plt.close(fig)
    print("  calibration_effect.pdf")


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import os
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("Generating figures...")
    fig_anchor_topology()
    fig_dstwr_timing()
    fig_nlos_diagram()
    fig_system_pipeline()
    fig_calibration_effect()
    print("Done. All figures written to paper/figures/")
