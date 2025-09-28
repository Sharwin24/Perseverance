# End-to-end Ackermann-style planner demo with Dubins + curvature ramps + time-scaling.
# Generates a few example paths and plots them with matplotlib (one chart per figure).
#
# Notes:
# - Self-contained: defines a simple rover geometry and limits inline.
# - Uses a minimal Dubins shortest-path implementation (LSL, RSR, LSR, RSL, RLR, LRL).
# - Inserts linear curvature ramps to respect steering-rate-like bounds.
# - Time-parameterizes trajectory to respect per-wheel max speed.
#
# You can tweak geometry/limits at the CONFIG section and re-run.

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt
from rover_constants import WHEEL_LOCATIONS_SCALED as WHEEL_LOCATIONS
from rover_constants import STEERABLE, ROVER_BODY_LENGTH, ROVER_BODY_WIDTH

# ---------------------- CONFIG: Rover geometry & limits ----------------------
ROVER_BODY_LENGTH_SCALED = ROVER_BODY_LENGTH / 100  # for plotting
ROVER_BODY_WIDTH_SCALED = ROVER_BODY_WIDTH / 100  # for plotting


@dataclass
class RoverLimits:
    wheel_diameter: float         # m
    wheel_speed_max: float        # m/s tread speed
    steer_angle_max: float        # rad (per steerable)
    # rad/s (unused here; we map to kappa'(s) below)
    steer_rate_max: float
    accel_max: float              # m/s^2 centerline accel
    decel_max: float              # m/s^2 centerline decel
    kappa_prime_max: float        # 1/m^2 max |dκ/ds|


LIMITS = RoverLimits(
    wheel_diameter=0.15,
    wheel_speed_max=1.2,
    steer_angle_max=math.radians(30.0),
    steer_rate_max=math.radians(90.0),
    accel_max=0.8,
    decel_max=1.2,
    kappa_prime_max=0.8
)

# ---------------------- Utility helpers ----------------------


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2*math.pi) - math.pi


def steer_angles_from_kappa(kappa: float) -> Dict[str, float]:
    ang = {w: 0.0 for w in WHEEL_LOCATIONS.keys()}
    if abs(kappa) < 1e-9:
        return ang
    R = 1.0 / kappa
    for name in STEERABLE:
        x_i, y_i = WHEEL_LOCATIONS[name]
        ang[name] = math.atan2(x_i, (R - y_i))
    return ang


def per_wheel_speed(v: float, kappa: float, wheel_angles: Dict[str, float]) -> Dict[str, float]:
    omega = v * kappa
    speeds = {}
    for name, (x_i, y_i) in WHEEL_LOCATIONS.items():
        vx = v - omega * y_i
        vy = omega * x_i
        delta = wheel_angles.get(name, 0.0)
        ux, uy = math.cos(delta), math.sin(delta)
        speeds[name] = vx*ux + vy*uy
    return speeds


def kappa_max_from_geometry(steer_angle_max: float) -> float:
    # small numeric search for tightest kappa allowed by any steerable wheel
    # scan up to a reasonable curvature
    max_scan = 10.0 / max(ROVER_BODY_LENGTH_SCALED, 1e-3)
    ks = np.linspace(0.0, max_scan, 5000)

    def ok(k):
        if k < 1e-9:
            return True
        R = 1.0 / k
        for name in STEERABLE:
            x_i, y_i = WHEEL_LOCATIONS[name]
            delta = math.atan2(x_i, (R - y_i))
            if abs(delta) > steer_angle_max + 1e-9:
                return False
        return True
    kmax = 0.0
    for k in ks:
        if ok(k):
            kmax = k
        else:
            break
    return float(kmax)

# ---------------------- Minimal Dubins implementation ----------------------
# Based on standard formulas (Shkel & Lumelsky). We compute the shortest of 6 cases.


def dubins_shortest_path(q0: Tuple[float, float, float],
                         q1: Tuple[float, float, float],
                         Rmin: float):
    """
    Return a list of segments: (type, length), where type in {'L','R','S'} and
    length is arc length for turns (in meters) or straight length (meters).
    """
    x0, y0, th0 = q0
    x1, y1, th1 = q1

    # transform q1 into q0's frame scaled by Rmin
    dx = x1 - x0
    dy = y1 - y0
    c, s = math.cos(th0), math.sin(th0)
    x = (c*dx + s*dy) / Rmin
    y = (-s*dx + c*dy) / Rmin
    th = wrap_to_pi(th1 - th0)

    def polar(x, y):
        r = math.hypot(x, y)
        t = math.atan2(y, x)
        return r, t

    def mod2pi(a):
        return (a % (2*math.pi))

    sols = []  # (cost, [("L", L1), ("S", L2), ("L", L3)])

    # LSL
    r, t = polar(x - math.sin(th), y - 1 + math.cos(th))
    if r >= 0:
        t1 = mod2pi(t - math.atan2(0, 1))  # just t
        t2 = r
        t3 = mod2pi(th - t)
        cost = (t1 + t2 + t3)
        sols.append((cost, [("L", t1*Rmin), ("S", t2*Rmin), ("L", t3*Rmin)]))

    # RSR
    r, t = polar(x + math.sin(th), y + 1 - math.cos(th))
    if r >= 0:
        t1 = mod2pi(-t)
        t2 = r
        t3 = mod2pi(-wrap_to_pi(th) + t)
        cost = (t1 + t2 + t3)
        sols.append((cost, [("R", t1*Rmin), ("S", t2*Rmin), ("R", t3*Rmin)]))

    # LSR
    r, t = polar(x - math.sin(th), y - 1 + math.cos(th))
    r = math.hypot(r, 2)  # distance between circles
    if r >= 2:
        # angle between circles
        theta = math.atan2(2.0, math.hypot(
            x - math.sin(th), y - 1 + math.cos(th)))
        t1 = mod2pi(t + theta)
        t2 = math.sqrt(r**2 - 4.0)
        t3 = mod2pi(-wrap_to_pi(th) - t + theta)
        sols.append(
            (t1 + t2 + t3, [("L", t1*Rmin), ("S", t2*Rmin), ("R", t3*Rmin)]))

    # RSL
    r, t = polar(x + math.sin(th), y + 1 - math.cos(th))
    r = math.hypot(r, 2)
    if r >= 2:
        theta = math.atan2(2.0, math.hypot(
            x + math.sin(th), y + 1 - math.cos(th)))
        t1 = mod2pi(-t + theta)
        t2 = math.sqrt(r**2 - 4.0)
        t3 = mod2pi(wrap_to_pi(th) - t + theta)
        sols.append(
            (t1 + t2 + t3, [("R", t1*Rmin), ("S", t2*Rmin), ("L", t3*Rmin)]))

    # RLR
    r, t = polar(x - math.sin(th), y - 1 + math.cos(th))
    d = 0.25*(2 + r**2)
    if d <= 1:
        phi = math.acos(d)
        t1 = mod2pi(-t + phi + math.pi/2)
        t2 = mod2pi(2*math.pi - 2*phi)
        t3 = mod2pi(wrap_to_pi(th) - t + phi + math.pi/2)
        sols.append(
            (t1 + t2 + t3, [("R", t1*Rmin), ("L", t2*Rmin), ("R", t3*Rmin)]))

    # LRL
    r, t = polar(x + math.sin(th), y + 1 - math.cos(th))
    d = 0.25*(2 + r**2)
    if d <= 1:
        phi = math.acos(d)
        t1 = mod2pi(t - phi - math.pi/2)
        t2 = mod2pi(2*math.pi - 2*phi)
        t3 = mod2pi(-wrap_to_pi(th) + t - phi - math.pi/2)
        sols.append(
            (t1 + t2 + t3, [("L", t1*Rmin), ("R", t2*Rmin), ("L", t3*Rmin)]))

    if not sols:
        raise RuntimeError("Dubins solver failed (no solutions)")

    best = min(sols, key=lambda x: x[0])
    # transform back: lengths are already in meters (scaled by Rmin).
    return best[1]

# Turn a Dubins sequence into piecewise constant curvature segments


@dataclass
class PathSeg:
    length: float
    kappa0: float
    kappa1: float
    kind: str  # "S" or "L" or "R"


def dubins_to_segments(q0, q1, Rmin):
    segs_raw = dubins_shortest_path(q0, q1, Rmin)
    segs: List[PathSeg] = []
    for kind, L in segs_raw:
        if kind == "S":
            segs.append(PathSeg(L, 0.0, 0.0, "S"))
        elif kind == "L":
            k = 1.0 / Rmin
            segs.append(PathSeg(L, k, k, "L"))
        elif kind == "R":
            k = -1.0 / Rmin
            segs.append(PathSeg(L, k, k, "R"))
    return segs

# ---------------------- Curvature ramps & integration ----------------------


def insert_ramps(segments: List[PathSeg], kappa_prime_max: float, ds_sample=0.02):
    """Return (s, kappa) samples after inserting linear ramps between curvatures."""
    s = 0.0
    out = [(0.0, segments[0].kappa0 if segments else 0.0)]
    k_prev = out[0][1]
    for seg in segments:
        k_target = seg.kappa0
        # ramp from k_prev to k_target
        if abs(k_target - k_prev) > 1e-12:
            Lr = abs(k_target - k_prev) / max(kappa_prime_max, 1e-9)
            n = max(1, int(math.ceil(Lr / ds_sample)))
            for j in range(1, n+1):
                s += Lr / n
                k = k_prev + (k_target - k_prev) * (j / n)
                out.append((s, k))
            k_prev = k_target
        # flat segment at k_target
        if seg.length > 1e-9:
            n = max(1, int(math.ceil(seg.length / ds_sample)))
            for j in range(1, n+1):
                s += seg.length / n
                out.append((s, k_target))
            k_prev = k_target
    return out


def integrate_centerline(kappa_profile: List[Tuple[float, float]], start_pose: Tuple[float, float, float]):
    xs, ys, thetas, ss, ks = [], [], [], [], []
    x, y, th = start_pose
    s_prev, k_prev = kappa_profile[0]
    xs.append(x)
    ys.append(y)
    thetas.append(th)
    ss.append(0.0)
    ks.append(k_prev)
    for s, k in kappa_profile[1:]:
        ds = s - s_prev
        # second-order midpoint integration
        th_mid = th + 0.5*(k_prev + k)*ds
        x += math.cos(th_mid) * ds
        y += math.sin(th_mid) * ds
        th += 0.5*(k_prev + k)*ds
        xs.append(x)
        ys.append(y)
        thetas.append(th)
        ss.append(s)
        ks.append(k)
        s_prev, k_prev = s, k
    return np.array(ss), np.array(xs), np.array(ys), np.array(thetas), np.array(ks)

# ---------------------- Time-scaling with wheel-speed limit ----------------------


def time_scale_with_wheels(ss, kappas, limits: RoverLimits):
    N = len(ss)
    v = np.full(N, np.inf)
    for i in range(N):
        k = float(kappas[i])
        ang = steer_angles_from_kappa(k)
        # binary search v so that max |s_i| <= wheel_speed_max
        v_lo, v_hi = 0.0, limits.wheel_speed_max * 2.0 + 1e-6
        for _ in range(30):
            vm = 0.5*(v_lo + v_hi)
            speeds = per_wheel_speed(vm, k, ang)
            if max(abs(s) for s in speeds.values()) <= limits.wheel_speed_max:
                v_lo = vm
            else:
                v_hi = vm
        v[i] = v_lo

    # forward accel limit in s-domain: v_i^2 - v_{i-1}^2 <= 2*a*ds
    for i in range(1, N):
        ds = ss[i] - ss[i-1]
        vmax = math.sqrt(max(0.0, v[i-1]**2 + 2*limits.accel_max*ds))
        v[i] = min(v[i], vmax)
    # backward decel limit
    for i in range(N-2, -1, -1):
        ds = ss[i+1] - ss[i]
        vmax = math.sqrt(max(0.0, v[i+1]**2 + 2*limits.decel_max*ds))
        v[i] = min(v[i], vmax)

    # steering-rate -> limit v via |kappa'(s)| * v <= kappa_prime_t_max (optional)
    # Here we conservatively reuse kappa_prime_max as κ'(s) cap in time domain:
    # v <= kappa_prime_t_max / |κ'(s)|. We skip for simplicity; could add if needed.

    # build time stamps
    t = np.zeros(N)
    for i in range(1, N):
        ds = ss[i] - ss[i-1]
        v_mid = max(1e-4, 0.5*(v[i] + v[i-1]))
        t[i] = t[i-1] + ds / v_mid
    return t, v

# ---------------------- Plotting helpers ----------------------


def plot_path(xs, ys, ths, kappas, start_pose, goal_pose, title):
    plt.figure(figsize=(6, 6))
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.title(title)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.plot(xs, ys, linewidth=2, label="centerline")

    # Draw start/goal headings
    def draw_pose(pose, label):
        x, y, th = pose
        plt.plot([x], [y], marker="o")
        L = 0.25
        plt.arrow(x, y, L*math.cos(th), L*math.sin(th),
                  head_width=0.06, length_includes_head=True)
        plt.text(x+0.05, y+0.05, label)

    draw_pose(start_pose, "start")
    draw_pose(goal_pose, "goal")

    # sample a few frames and draw the rover footprint rectangle
    idxs = np.linspace(0, len(xs)-1, 8, dtype=int)
    for i in idxs:
        x, y, th = xs[i], ys[i], ths[i]
        # rectangle corners in body frame
        Lb, Wb = ROVER_BODY_LENGTH_SCALED, ROVER_BODY_WIDTH_SCALED
        corners = np.array([
            [Lb/2,  Wb/2],
            [Lb/2, -Wb/2],
            [-Lb/2, -Wb/2],
            [-Lb/2,  Wb/2],
            [Lb/2,  Wb/2],
        ])
        R = np.array([[math.cos(th), -math.sin(th)],
                      [math.sin(th),  math.cos(th)]])
        pts = (R @ corners.T).T + np.array([x, y])
        plt.plot(pts[:, 0], pts[:, 1], linewidth=1)

    plt.legend()
    plt.show()

# ---------------------- End-to-end convenience ----------------------


def plan_and_plot(start_pose, goal_pose, limits: RoverLimits, title: str):
    # curvature bound from geometry
    kappa_max = kappa_max_from_geometry(limits.steer_angle_max)
    Rmin = 1.0 / max(kappa_max, 1e-6)

    # 1) geometric (Dubins)
    segs = dubins_to_segments(start_pose, goal_pose, Rmin)

    # 2) curvature ramps
    kprof = insert_ramps(segs, limits.kappa_prime_max, ds_sample=0.02)

    # 3) integrate
    ss, xs, ys, ths, ks = integrate_centerline(kprof, start_pose)

    # 4) time-scaling (wheel limits)
    t, v = time_scale_with_wheels(ss, ks, limits)

    # 5) plot
    plot_path(xs, ys, ths, ks, start_pose, goal_pose, title)

    # return for further use if needed
    return {"s": ss, "t": t, "x": xs, "y": ys, "theta": ths, "kappa": ks, "v": v}


# ---------------------- Examples ----------------------
examples = [
    ((0.0, 0.0, 0.0),  (4.0, 0.5, math.radians(10)),
     "Slight offset & small heading change"),
    ((0.0, 0.0, 0.0),  (3.0, 2.0, math.radians(90)),  "Left turn to face north"),
    ((0.0, 0.0, 0.0),  (5.0, -1.5, math.radians(-45)), "Rightward S-curve style goal"),
]

results = []
for (s, g, name) in examples:
    results.append(plan_and_plot(s, g, LIMITS, f"Planned path: {name}"))
