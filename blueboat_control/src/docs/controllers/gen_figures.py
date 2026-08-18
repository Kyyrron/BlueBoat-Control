"""Render the controller-comparison figures from the cached simulations."""
import math
import os
import pickle

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import sim

CACHE = os.path.join(os.path.dirname(__file__), "cache")
OUT = r"c:\Users\killi\Desktop\Research Kyutech\BlueBoat\BlueBoat-SideScanSonar\blueboat_control\src\docs\controllers"
os.makedirs(OUT, exist_ok=True)

# ── validated palette (dataviz skill reference instance, light mode) ───────────
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#8a8983"
GRID = "#e6e5e1"
C = {"MPC": "#2a78d6", "PID": "#eb6834", "LoS": "#1baf7a", "Point-LoS": "#4a3aa7"}
REF = "#9a998f"
RAMP = ["#86b6ef", "#2a78d6", "#104281"]        # ordinal, validated
GOOD, BAD = "#1baf7a", "#e34948"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "font.family": "DejaVu Sans", "font.size": 9,
    "axes.edgecolor": "#c9c8c3", "axes.labelcolor": INK2, "axes.labelsize": 9.5,
    "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.titlecolor": INK,
    "axes.titlelocation": "left", "axes.titlepad": 8,
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelsize": 8.5,
    "ytick.labelsize": 8.5, "legend.frameon": False, "legend.fontsize": 9,
    "grid.color": GRID, "grid.linewidth": 0.8, "lines.linewidth": 1.8,
    "lines.solid_capstyle": "round", "figure.dpi": 150,
})


def style(ax, grid="both"):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_axisbelow(True)
    if grid:
        ax.grid(True, axis=grid if grid != "both" else "both")
    return ax


def load(key):
    with open(os.path.join(CACHE, key + ".pkl"), "rb") as f:
        return pickle.load(f)


def label_end(ax, x, y, text, color, dx=6, dy=0):
    ax.annotate(text, (x, y), textcoords="offset points", xytext=(dx, dy),
                color=color, fontsize=9, fontweight="bold", va="center")


def save(fig, name):
    fig.savefig(os.path.join(OUT, name), bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)
    print("wrote", name)


# ══ Fig 1 — LoS geometry + the lookahead curve ════════════════════════════════
def fig_geometry():
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.0),
                               gridspec_kw={"width_ratios": [1.15, 1]})

    # (a) geometry
    a.axhline(0, color=REF, lw=1.6, ls=(0, (6, 4)))
    a.annotate("planned path", (9.8, 0.28), color=REF, fontsize=9, va="bottom", ha="right")
    bx, by, bpsi = 2.0, -2.6, 0.35
    a.plot([bx], [by], "o", ms=10, color=INK, zorder=5)
    a.arrow(bx, by, 1.5 * math.cos(bpsi), 1.5 * math.sin(bpsi), head_width=0.28,
            color=INK, lw=1.6, zorder=5, length_includes_head=True)
    a.annotate("boat  ψ", (bx - 0.15, by - 0.55), color=INK, fontsize=9.5,
               ha="center", va="top", fontweight="bold")
    # cross-track
    a.plot([bx, bx], [by, 0], color=BAD, lw=1.8)
    a.annotate("$e_y$\ncross-track\nerror", (bx - 0.25, by / 2), color=BAD, fontsize=9,
               ha="right", va="center", fontweight="bold")
    # target at tau
    a.plot([bx], [0], "o", ms=8, color=REF, zorder=4)
    a.annotate("virtual target\n(pose at τ)", (bx - 0.25, 0.3), color=INK2, fontsize=9,
               ha="right", va="bottom")
    # lookahead
    lx = bx + 4.0
    a.plot([bx, lx], [0, 0], color=C["MPC"], lw=2.4)
    a.plot([lx], [0], "o", ms=9, color=C["MPC"], zorder=5)
    a.annotate("Δ  lookahead", ((bx + lx) / 2, 0.3), color=C["MPC"], fontsize=9.5,
               ha="center", va="bottom", fontweight="bold")
    a.annotate("aim point", (lx, -0.35), color=C["MPC"], fontsize=9,
               ha="center", va="top")
    # desired heading
    a.plot([bx, lx], [by, 0], color=C["MPC"], lw=2.0, ls=(0, (5, 3)))
    a.annotate("$ψ_d$ = aim here", ((bx + lx) / 2 + 0.4, by / 2 - 0.25),
               color=C["MPC"], fontsize=9.5, ha="left", va="center", fontweight="bold")
    a.set_xlim(0.2, 10); a.set_ylim(-4.2, 1.6)
    a.set_aspect("equal"); a.set_xticks([]); a.set_yticks([])
    for sp in a.spines.values():
        sp.set_visible(False)
    a.set_title("a.  The lookahead law:  $ψ_d = γ_p + \\arctan(-e_y / Δ)$")

    # (b) correction curve
    ey = np.linspace(-8, 8, 400)
    for D, col in zip((1.0, 2.5, 6.0), RAMP):
        corr = np.degrees(np.arctan2(-ey, D))
        b.plot(ey, corr, color=col, label=f"Δ = {D:g} m")
        label_end(b, ey[-1], corr[-1], f"Δ={D:g} m", col, dx=5)
    b.axhline(0, color=MUTED, lw=0.9)
    b.axvline(0, color=MUTED, lw=0.9)
    b.set_xlabel("cross-track error $e_y$  (m)   ← left of path | right of path →")
    b.set_ylabel("heading correction  (deg)")
    b.set_yticks([-90, -45, 0, 45, 90])
    b.set_title("b.  Smaller Δ = harder correction")
    b.legend(loc="upper right", bbox_to_anchor=(1.0, 1.0))
    b.set_xlim(-8, 11.5)
    style(b)
    fig.suptitle("Line-of-sight guidance — shared by the LoS and PID controllers",
                 x=0.005, ha="left", fontsize=12.5, fontweight="bold", color=INK)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    save(fig, "fig1_los_geometry.png")


# ══ Fig 2 — acquisition from a 5 m offset ═════════════════════════════════════
def fig_acquisition():
    runs = {n: load(f"acquire_{n}") for n in ("MPC", "PID", "LoS")}
    fig = plt.figure(figsize=(11, 6.6))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], hspace=0.42, wspace=0.24)
    ax = fig.add_subplot(gs[0, :])
    rx, ry = sim.reference_track("straight_line", 60)
    ax.plot(rx, ry, color=REF, ls=(0, (6, 4)), lw=1.6, label="planned path (0.50 m/s)")
    for n, d in runs.items():
        ax.plot(d["x"], d["y"], color=C[n], label=n)
        label_end(ax, d["x"][-1], d["y"][-1], n, C[n])
    ax.plot([0], [-4], "o", ms=9, color=INK, zorder=6)
    ax.annotate("start\n(5 m off the path)", (0, -4), textcoords="offset points",
                xytext=(10, 4), fontsize=9, color=INK, va="bottom")
    ax.set_aspect("equal")
    ax.set_xlabel("x  (m)"); ax.set_ylabel("y  (m)")
    ax.set_title("a.  Acquiring a straight path from a 5 m cross-track offset  (60 s)")
    ax.legend(loc="lower right", ncol=2)
    style(ax)

    b = fig.add_subplot(gs[1, 0])
    for n, d in runs.items():
        b.plot(d["t"], d["e_y"], color=C[n])
        label_end(b, d["t"][-1], d["e_y"][-1], n, C[n], dx=4)
    b.axhline(0, color=MUTED, lw=0.9)
    b.set_xlabel("time  (s)"); b.set_ylabel("cross-track error  (m)")
    b.set_title("b.  Convergence onto the path")
    b.set_xlim(0, 66)
    style(b)

    c = fig.add_subplot(gs[1, 1])
    c.axhline(0.5, color=REF, ls=(0, (6, 4)), lw=1.4)
    c.annotate("authored 0.50 m/s", (30, 0.52), color=REF, fontsize=8.5,
               ha="center", va="bottom")
    c.axhline(0, color=MUTED, lw=0.9)
    for n, d in runs.items():
        c.plot(d["t"], d["u"], color=C[n])
        label_end(c, d["t"][-1], d["u"][-1], n, C[n], dx=4)
    c.annotate("MPC sprints to 1.0 m/s,\nthen brakes (full reverse)", (10, 0.95),
               textcoords="offset points", xytext=(14, 6), fontsize=8.5,
               color=C["MPC"], va="bottom")
    c.set_xlabel("time  (s)"); c.set_ylabel("surge speed $u$  (m/s)")
    c.set_title("c.  Speed tracking")
    c.set_xlim(0, 66); c.set_ylim(-0.25, 1.18)
    style(c)
    save(fig, "fig2_acquisition.png")


# ══ Fig 3/4 — curved and cornered paths ═══════════════════════════════════════
def fig_path(key, shape, tmax, title, fname, xlim=None, ylim=None, legend_loc="best"):
    runs = {n: load(f"{key}_{n}") for n in ("MPC", "PID", "LoS")}
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.6),
                               gridspec_kw={"width_ratios": [1.05, 1]})
    rx, ry = sim.reference_track(shape, tmax)
    a.plot(rx, ry, color=REF, ls=(0, (6, 4)), lw=1.6, label="planned path")
    for n, d in runs.items():
        a.plot(d["x"], d["y"], color=C[n], label=n)
        label_end(a, d["x"][-1], d["y"][-1], n, C[n])
    a.plot([0], [0], "o", ms=8, color=INK, zorder=6)
    a.annotate("start", (0, 0), textcoords="offset points", xytext=(6, -10),
               fontsize=9, color=INK)
    a.set_aspect("equal"); a.set_xlabel("x  (m)"); a.set_ylabel("y  (m)")
    a.set_title(f"a.  {title}")
    if xlim:
        a.set_xlim(*xlim)
    if ylim:
        a.set_ylim(*ylim)
    if legend_loc == "below":
        a.legend(loc="upper center", bbox_to_anchor=(0.5, -0.40), ncol=4, fontsize=8.5)
    else:
        a.legend(loc=legend_loc, ncol=2, fontsize=8.5)
    style(a)
    for n, d in runs.items():
        b.plot(d["t"], np.abs(d["e_y"]), color=C[n])
        label_end(b, d["t"][-1], abs(d["e_y"][-1]), n, C[n], dx=4)
    b.set_xlabel("time  (s)"); b.set_ylabel("|cross-track error|  (m)")
    b.set_title("b.  Tracking error")
    style(b)
    save(fig, fname)


# ══ Fig 5 — steady lateral current ════════════════════════════════════════════
def fig_current():
    runs = {n: load(f"current_{n}") for n in ("MPC", "PID", "LoS")}
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.2),
                               gridspec_kw={"width_ratios": [1.15, 1]})
    a.axhline(1.0, color=REF, ls=(0, (6, 4)), lw=1.6, label="planned path")
    for n, d in runs.items():
        a.plot(d["x"], d["y"], color=C[n], label=n)
        label_end(a, d["x"][-1], d["y"][-1], n, C[n])
    a.annotate("10 N current", (10.5, -8.0), color=BAD, fontsize=9.5, fontweight="bold",
               ha="left", va="center")
    a.arrow(9.0, -6.6, 0, -2.6, head_width=1.1, head_length=0.7, color=BAD, lw=1.8,
            length_includes_head=True)
    a.set_xlabel("x  (m)"); a.set_ylabel("y  (m)")
    a.set_title("a.  Holding a line against a steady 10 N side current  (90 s)")
    a.legend(loc="lower right", ncol=2)
    style(a)
    for n, d in runs.items():
        b.plot(d["t"], d["e_y"], color=C[n])
        label_end(b, d["t"][-1], d["e_y"][-1], n, C[n], dx=4)
    b.axhline(0, color=MUTED, lw=0.9)
    b.set_xlabel("time  (s)"); b.set_ylabel("cross-track error  (m)")
    b.set_title("b.  Steady-state offset  (0 = on the line)")
    style(b)
    save(fig, "fig5_current.png")


# ══ Fig 6 — lookahead sweep ═══════════════════════════════════════════════════
def fig_lookahead():
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.4),
                               gridspec_kw={"width_ratios": [1.0, 1.1]})
    a.axhline(1.0, color=REF, ls=(0, (6, 4)), lw=1.6, label="planned path")
    for D, col in zip((0.5, 2.5, 12.0), RAMP):
        d = load(f"lookF_{D}")
        a.plot(d["x"], d["y"], color=col, label=f"Δ = {D:g} m")
        b.plot(d["t"], d["e_y"], color=col)
        label_end(b, d["t"][-1], d["e_y"][-1], f"Δ={D:g} m", col, dx=5)
    a.plot([0], [-4], "o", ms=8, color=INK, zorder=6)
    a.annotate("start", (0, -4), textcoords="offset points", xytext=(8, 2),
               fontsize=9, color=INK)
    a.set_aspect("equal"); a.set_xlim(-1, 22); a.set_ylim(-5, 2.6)
    a.set_xlabel("x  (m)"); a.set_ylabel("y  (m)")
    a.set_title("a.  Same manoeuvre, three lookahead distances")
    a.legend(loc="lower right", fontsize=8.5)
    style(a)
    b.axhline(0, color=MUTED, lw=0.9)
    b.annotate("Δ=0.5 m overshoots\nby 0.29 m", (30, 0.29), textcoords="offset points",
               xytext=(6, 30), fontsize=8.5, color=RAMP[0], fontweight="bold",
               arrowprops=dict(arrowstyle="-", color=RAMP[0], lw=1.0))
    b.set_xlim(0, 104)
    b.set_xlabel("time  (s)"); b.set_ylabel("cross-track error  (m)")
    b.set_title("b.  Small Δ: quick but overshoots · large Δ: smooth but slow")
    style(b)
    fig.suptitle("Lookahead distance Δ is the aggressiveness dial   "
                 "(LoS controller, surge gain corrected per §6)",
                 x=0.005, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    save(fig, "fig6_lookahead.png")


# ══ Fig 7 — the surge-gain finding ════════════════════════════════════════════
def fig_surge():
    s = load("surge_sweep")
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    ax.axhline(0.5, color=REF, ls=(0, (6, 4)), lw=1.5)
    ax.annotate("authored speed 0.50 m/s", (0.55, 0.505), color=REF, fontsize=9,
                ha="left", va="bottom")
    ax.plot(s["los_k"], s["los_u"], "-o", ms=6, color=C["LoS"])
    ax.plot(s["pid_k"], s["pid_u"], "-o", ms=6, color=C["PID"])
    label_end(ax, s["los_k"][-1], s["los_u"][-1], "LoS  (los_ku)", C["LoS"], dx=6)
    label_end(ax, s["pid_k"][-1], s["pid_u"][-1], "PID  (inner u gain)", C["PID"], dx=6)
    for k, u, col in ((8.0, s["los_u"][s["los_k"].index(8)], C["LoS"]),
                      (1.0, s["pid_u"][s["pid_k"].index(1.0)], C["PID"])):
        ax.plot([k], [u], "o", ms=11, mfc="none", mec=BAD, mew=2.0, zorder=6)
    ax.annotate("shipped default\nlos_ku = 8", (8, 0.107), textcoords="offset points",
                xytext=(10, 22), fontsize=9, color=BAD, fontweight="bold",
                arrowprops=dict(arrowstyle="-", color=BAD, lw=1.2))
    ax.annotate("shipped default\ninner u gain = 1.0", (1.0, 0.285),
                textcoords="offset points", xytext=(6, 26), fontsize=9, color=BAD,
                fontweight="bold", arrowprops=dict(arrowstyle="-", color=BAD, lw=1.2))
    ax.set_xscale("log")
    ax.set_xlim(0.4, 1500); ax.set_ylim(0, 0.58)
    ax.set_xlabel("surge gain  (log scale)")
    ax.set_ylabel("achieved cruise speed  (m/s)")
    ax.set_title("Both controllers ship with a surge gain far below the drag coefficient")
    style(ax)
    save(fig, "fig7_surge_gain.png")


# ══ Fig 8 — point LoS ═════════════════════════════════════════════════════════
def fig_point():
    real, simg = load("point_real"), load("point_sim")
    VIO, RED = C["Point-LoS"], "#e34948"
    names = ["start heading 0°", "90°", "180° (facing away)", "−90°"]
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.6),
                               gridspec_kw={"width_ratios": [1.0, 1.1]})
    for d, nm in zip(real, names):
        a.plot(d["x"], d["y"], color=VIO, alpha=0.9)
        i = len(d["x"]) // 5
        a.annotate(nm, (d["x"][i], d["y"][i]), textcoords="offset points",
                   xytext=(5, 5), fontsize=8.5, color=VIO)
    a.plot([12], [6], "*", ms=20, color=RED, zorder=6)
    a.annotate("target", (12, 6), textcoords="offset points", xytext=(10, 2),
               fontsize=9.5, color=RED, fontweight="bold")
    a.plot([0], [0], "o", ms=8, color=INK, zorder=6)
    a.annotate("start", (0, 0), textcoords="offset points", xytext=(-6, -12),
               fontsize=9, color=INK, ha="right")
    a.set_aspect("equal"); a.set_xlabel("x  (m)"); a.set_ylabel("y  (m)")
    a.set_title("a.  Real-boat gains: converges from any heading")
    style(a)

    for k, d in enumerate(simg):
        b.plot(d["t"], d["d"], color=RED, alpha=0.9,
               label="simulation gains  $k_v$=2.0, $k_ψ$=16" if k == 0 else None)
    for k, d in enumerate(real):
        b.plot(d["t"], d["d"], color=VIO, alpha=0.9,
               label="real-boat gains  $k_v$=0.15, $k_ψ$=10" if k == 0 else None)
    b.axhline(0, color=MUTED, lw=0.9)
    b.annotate("overshoots, then runs away\nfaster the further it gets", (86, 62),
               fontsize=9, color=RED, fontweight="bold", ha="right", va="top")
    b.annotate("arrives", (150, 3.5), fontsize=9, color=VIO, fontweight="bold",
               ha="right", va="bottom")
    b.set_xlabel("time  (s)"); b.set_ylabel("distance to target  (m)")
    b.set_title("b.  The shipped simulation gains diverge on this hull model")
    b.legend(loc="upper left", fontsize=8.5)
    style(b)
    save(fig, "fig8_point_los.png")


# ══ Fig 9 — the governor ══════════════════════════════════════════════════════
def fig_governor():
    slow, ok = load("gov_slow"), load("gov_ok")
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4.0))
    OKC, SLOWC = C["MPC"], C["PID"]
    L_OK = "boat reaches\nthe authored speed"
    L_SLOW = "boat cannot reach it\n(held back by a headwind)"
    a.plot(ok["t"], ok["t"], color=REF, ls=(0, (6, 4)), lw=1.4)
    a.plot(ok["t"], ok["tau"], color=OKC, label=L_OK.replace("\n", " "))
    a.plot(slow["t"], slow["tau"], color=SLOWC, label=L_SLOW.replace("\n", " "))
    a.annotate("τ = wall clock", (148, 104), color=REF, fontsize=9, ha="right", va="top")
    a.set_xlabel("real time  (s)"); a.set_ylabel("path parameter τ  (path seconds)")
    a.set_title("a.  τ advances only as fast as the boat earns it")
    a.legend(loc="upper left", fontsize=8.5)
    style(a)
    b.plot(ok["t"], ok["e_along"], color=OKC)
    b.plot(slow["t"], slow["e_along"], color=SLOWC)
    label_end(b, 150, ok["e_along"][-1], "", OKC)
    b.axhspan(3.0, 4.2, color=BAD, alpha=0.10)
    b.axhline(3.0, color=BAD, lw=1.2, ls=(0, (4, 3)))
    b.annotate("gov_Lmax = 3 m → τ freezes", (2, 3.12), color=BAD, fontsize=9,
               ha="left", va="bottom", fontweight="bold")
    b.axhline(0.5, color=GOOD, lw=1.2, ls=(0, (4, 3)))
    b.annotate("gov_Lmin = 0.5 m → full speed", (2, 0.62), color=GOOD, fontsize=9,
               ha="left", va="bottom", fontweight="bold")
    b.annotate("τ crawls at 0.19×", (52, 2.32), fontsize=9, color=SLOWC,
               fontweight="bold", ha="left", va="top")
    b.annotate("τ at full speed", (100, 0.24), fontsize=9, color=OKC,
               fontweight="bold", ha="left", va="top")
    b.set_ylim(-0.6, 4.2); b.set_xlim(-4, 152)
    b.set_xlabel("real time  (s)"); b.set_ylabel("along-track gap $e_{along}$  (m)")
    b.set_title("b.  The gap settles where target speed = boat speed")
    style(b)
    save(fig, "fig9_governor.png")


if __name__ == "__main__":
    fig_geometry()
    fig_acquisition()
    fig_path("circle", "circle", 160, "4 m circle, boat starts 90° off heading  (160 s)",
             "fig3_circle.png")
    fig_path("square", "kin_square", 160, "Zig-zag with 90° corners  (160 s)",
             "fig4_square.png")
    fig_current()
    fig_lookahead()
    fig_surge()
    fig_point()
    fig_governor()
