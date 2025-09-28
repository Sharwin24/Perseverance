# ackermann_planner.py
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple, Set
import numpy as np
from rover_constants import WHEEL_DIAMETER


@dataclass
class RoverLimits:
    wheel_diameter: float         # [m]
    wheel_speed_max: float        # [rpm] rotational speed
    steer_angle_max: float        # [rad]
    accel_max: float              # [m/s^2] centerline
    decel_max: float              # [m/s^2] centerline

    def linear_speed_max(self) -> float:
        """Uses the max rotational speed to convert to linear speed [m/s]"""
        return (self.wheel_diameter * math.pi / 60) * self.wheel_speed_max


class AckermannPlanner:
    """
    Dubins-based (no reverse) Ackermann planner with wheel-limit time-scaling.
    Geometry is provided as body-frame wheel contact positions (x forward, y left).
    The 4 steerable wheels define curvature limits.
    """

    def __init__(
        self,
        wheel_locations: Dict[str, Tuple[float, float]],
        steerable: List[str] | Set[str],
        limits: RoverLimits,
    ):
        self.wheel_locations = dict(wheel_locations)
        self.steerable = set(steerable)
        self.limits = limits

        # Precompute curvature bound from geometry + steer limit
        self.kappa_max = self._kappa_max_from_geometry(
            self.limits.steer_angle_max)
        if self.kappa_max <= 0:
            raise ValueError(
                "Could not compute positive kappa_max from geometry")
        self.Rmin = 1.0 / self.kappa_max

    # -------------------- Public API --------------------

    def plan(
        self,
        start_pose: Tuple[float, float, float],
        goal_pose: Tuple[float, float, float],
        dt: float = 0.02,
        ds_prop: float = 0.01,
    ) -> Dict:
        """
        Returns:
          {
            "t": [N], "x":[N], "y":[N], "theta":[N], "kappa":[N], "v":[N],
            "wheel_angles": {name: [N]},       # radians
            "wheel_speeds": {name: [N]},       # m/s
            "wheel_omegas": {name: [N]},       # rad/s
          }
        """
        # 1) Geometric plan: shortest Dubins sequence (L/R/S with lengths in meters)
        segs = self._dubins_shortest_path(start_pose, goal_pose, self.Rmin)

        # 2) Exact propagation to get centerline samples in s (no smoothing)
        ss, xs, ys, ths, kappas = self._propagate_exact(
            start_pose, segs, self.Rmin, ds_prop)

        # 3) Time-scaling with per-wheel tread speed + accel/decel limits
        t, v = self._time_scale_with_wheels(ss, kappas)

        # 4) Sample at fixed dt and compute wheel angles/speeds for each sample
        out = self._sample_uniform_dt(t, ss, xs, ys, ths, kappas, v, dt)

        return out

    # -------------------- Geometry / kinematics --------------------

    def _kappa_max_from_geometry(self, steer_angle_max: float) -> float:
        # find tightest κ such that all steerables satisfy |delta_i(κ)| <= steer_angle_max
        Lref = max(
            1e-3,
            max(abs(x) for x, _ in self.wheel_locations.values()),
        )
        ks = np.linspace(0.0, 10.0 / Lref, 6000)  # generous scan

        def ok(k):
            if k < 1e-9:
                return True
            R = 1.0 / k
            for name in self.steerable:
                x_i, y_i = self.wheel_locations[name]
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

    def _steer_angles_from_kappa(self, kappa: float) -> Dict[str, float]:
        ang = {w: 0.0 for w in self.wheel_locations.keys()}
        if abs(kappa) < 1e-12:
            return ang
        R = 1.0 / kappa
        for name in self.steerable:
            x_i, y_i = self.wheel_locations[name]
            ang[name] = float(math.atan2(x_i, (R - y_i)))
        return ang

    def _per_wheel_speed(self, v_center: float, kappa: float, wheel_angles: Dict[str, float]) -> Dict[str, float]:
        """Wheel rolling speed (projection of contact velocity onto wheel heading)."""
        omega = v_center * kappa
        speeds = {}
        for name, (x_i, y_i) in self.wheel_locations.items():
            vx = v_center - omega * y_i   # body vy = 0 (nonholonomic)
            vy = omega * x_i
            delta = float(wheel_angles.get(name, 0.0))
            ux, uy = math.cos(delta), math.sin(delta)
            speeds[name] = float(vx * ux + vy * uy)
        return speeds

    # -------------------- Dubins (no reverse) --------------------

    @staticmethod
    def _wrap_to_pi(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _dubins_shortest_path(self, q0, q1, Rmin):
        """
        Returns list of segments: [('L'|'R'|'S', length_in_meters), ...]
        Shortest of the 6 standard Dubins families.
        """
        x0, y0, th0 = q0
        x1, y1, th1 = q1
        dx, dy = x1 - x0, y1 - y0
        c, s = math.cos(th0), math.sin(th0)
        # transform goal into start frame, scaled by Rmin
        x = (c * dx + s * dy) / Rmin
        y = (-s * dx + c * dy) / Rmin
        th = self._wrap_to_pi(th1 - th0)

        def mod2pi(a): return a % (2 * math.pi)
        def polar(X, Y): return math.hypot(X, Y), math.atan2(Y, X)

        sols = []

        # LSL
        r, t = polar(x - math.sin(th), y - 1 + math.cos(th))
        if r >= 0:
            t1 = mod2pi(t)
            t2 = r
            t3 = mod2pi(th - t)
            sols.append(
                (t1 + t2 + t3, [("L", t1 * Rmin), ("S", t2 * Rmin), ("L", t3 * Rmin)]))

        # RSR
        r, t = polar(x + math.sin(th), y + 1 - math.cos(th))
        if r >= 0:
            t1 = mod2pi(-t)
            t2 = r
            t3 = mod2pi(-self._wrap_to_pi(th) + t)
            sols.append(
                (t1 + t2 + t3, [("R", t1 * Rmin), ("S", t2 * Rmin), ("R", t3 * Rmin)]))

        # LSR
        r0, t0 = polar(x - math.sin(th), y - 1 + math.cos(th))
        r = math.hypot(r0, 2.0)
        if r >= 2.0:
            theta = math.atan2(2.0, r0)
            t1 = mod2pi(t0 + theta)
            t2 = math.sqrt(r * r - 4.0)
            t3 = mod2pi(-self._wrap_to_pi(th) - t0 + theta)
            sols.append(
                (t1 + t2 + t3, [("L", t1 * Rmin), ("S", t2 * Rmin), ("R", t3 * Rmin)]))

        # RSL
        r0, t0 = polar(x + math.sin(th), y + 1 - math.cos(th))
        r = math.hypot(r0, 2.0)
        if r >= 2.0:
            theta = math.atan2(2.0, r0)
            t1 = mod2pi(-t0 + theta)
            t2 = math.sqrt(r * r - 4.0)
            t3 = mod2pi(self._wrap_to_pi(th) - t0 + theta)
            sols.append(
                (t1 + t2 + t3, [("R", t1 * Rmin), ("S", t2 * Rmin), ("L", t3 * Rmin)]))

        # RLR
        r0, t0 = polar(x - math.sin(th), y - 1 + math.cos(th))
        d = 0.25 * (2.0 + r0 * r0)
        if d <= 1.0:
            phi = math.acos(d)
            t1 = mod2pi(-t0 + phi + math.pi / 2.0)
            t2 = mod2pi(2 * math.pi - 2 * phi)
            t3 = mod2pi(self._wrap_to_pi(th) - t0 + phi + math.pi / 2.0)
            sols.append(
                (t1 + t2 + t3, [("R", t1 * Rmin), ("L", t2 * Rmin), ("R", t3 * Rmin)]))

        # LRL
        r0, t0 = polar(x + math.sin(th), y + 1 - math.cos(th))
        d = 0.25 * (2.0 + r0 * r0)
        if d <= 1.0:
            phi = math.acos(d)
            t1 = mod2pi(t0 - phi - math.pi / 2.0)
            t2 = mod2pi(2 * math.pi - 2 * phi)
            t3 = mod2pi(-self._wrap_to_pi(th) + t0 - phi - math.pi / 2.0)
            sols.append(
                (t1 + t2 + t3, [("L", t1 * Rmin), ("R", t2 * Rmin), ("L", t3 * Rmin)]))

        if not sols:
            raise RuntimeError("Dubins failed: no solutions")

        return min(sols, key=lambda x: x[0])[1]  # [(kind, length_m), ...]

    def _propagate_exact(self, q0, segs, Rmin, ds=0.01):
        """
        Integrate the Dubins path exactly (so the endpoint equals goal).
        Returns arrays indexed by s: ss, xs, ys, thetas, kappas
        """
        x, y, th = q0
        xs, ys, ths, ss, ks = [x], [y], [th], [0.0], [0.0]

        for kind, L in segs:
            if kind == "S":
                n = max(1, int(math.ceil(L / ds)))
                step = L / n
                for _ in range(n):
                    x += math.cos(th) * step
                    y += math.sin(th) * step
                    xs.append(x)
                    ys.append(y)
                    ths.append(th)
                    ss.append(ss[-1] + step)
                    ks.append(0.0)
            else:
                # arc on circle of radius Rmin; L = arc_length
                sgn = +1 if kind == "L" else -1
                dth_total = sgn * (L / Rmin)
                # choose steps so heading steps align with ds along arc
                n = max(1, int(math.ceil(abs(dth_total) * Rmin / max(ds, 1e-6))))
                dth = dth_total / n
                for _ in range(n):
                    # rotate around instantaneous center
                    xc = x - sgn * Rmin * math.sin(th)
                    yc = y + sgn * Rmin * math.cos(th)
                    th_new = th + dth
                    x = xc + sgn * Rmin * math.sin(th_new)
                    y = yc - sgn * Rmin * math.cos(th_new)
                    th = th_new
                    xs.append(x)
                    ys.append(y)
                    ths.append(th)
                    ss.append(ss[-1] + abs(dth) * Rmin)
                    ks.append(sgn / Rmin)

        return np.asarray(ss), np.asarray(xs), np.asarray(ys), np.asarray(ths), np.asarray(ks)

    # -------------------- Time-scaling --------------------

    def _time_scale_with_wheels(self, ss, kappas):
        """
        Build a speed profile v(s) that respects per-wheel tread speed and accel/decel.
        Returns t(s) and v(s) sampled at the same indices as ss.
        """
        N = len(ss)
        v = np.full(N, np.inf, dtype=float)

        # Speed envelope from wheel-speed limit (binary search per sample)
        for i in range(N):
            k = float(kappas[i])
            ang = self._steer_angles_from_kappa(k)
            # find largest v such that all |s_i| <= wheel_speed_max
            vlo, vhi = 0.0, self.limits.linear_speed_max() * 2.0 + 1e-6
            for _ in range(28):
                vm = 0.5 * (vlo + vhi)
                speeds = self._per_wheel_speed(vm, k, ang)
                if max(abs(s) for s in speeds.values()) <= self.limits.linear_speed_max():
                    vlo = vm
                else:
                    vhi = vm
            v[i] = vlo

        # Forward accel pass
        for i in range(1, N):
            ds = ss[i] - ss[i - 1]
            vmax = math.sqrt(
                max(0.0, v[i - 1] ** 2 + 2.0 * self.limits.accel_max * ds))
            v[i] = min(v[i], vmax)

        # Backward decel pass
        for i in range(N - 2, -1, -1):
            ds = ss[i + 1] - ss[i]
            vmax = math.sqrt(
                max(0.0, v[i + 1] ** 2 + 2.0 * self.limits.decel_max * ds))
            v[i] = min(v[i], vmax)

        # Integrate time along s: dt = ds / v_mid
        t = np.zeros(N, dtype=float)
        for i in range(1, N):
            ds = ss[i] - ss[i - 1]
            v_mid = max(1e-6, 0.5 * (v[i] + v[i - 1]))
            t[i] = t[i - 1] + ds / v_mid

        return t, v

    # -------------------- Sampling to wheel commands --------------------

    def _sample_uniform_dt(self, t, ss, xs, ys, ths, kappas, v, dt: float):
        if t[-1] <= 0:
            # Degenerate (start==goal)
            return {
                "t": np.array([0.0]),
                "x": np.array([xs[0]]),
                "y": np.array([ys[0]]),
                "theta": np.array([ths[0]]),
                "kappa": np.array([0.0]),
                "v": np.array([0.0]),
                "wheel_angles": {k: np.array([0.0]) for k in self.wheel_locations},
                "wheel_speeds": {k: np.array([0.0]) for k in self.wheel_locations},
                "wheel_omegas": {k: np.array([0.0]) for k in self.wheel_locations},
            }

        T = float(t[-1])
        times = np.arange(0.0, T + 1e-9, dt)
        # index into original arrays by time
        idx = np.searchsorted(t, times, side="left")
        idx = np.clip(idx, 1, len(t) - 1)

        # simple linear interpolation between samples (s, x, y, theta, kappa, v)
        def lerp(a, b, alpha): return (1.0 - alpha) * a + alpha * b

        xs_out = np.empty_like(times)
        ys_out = np.empty_like(times)
        th_out = np.empty_like(times)
        k_out = np.empty_like(times)
        v_out = np.empty_like(times)

        wheel_angles = {name: np.empty_like(times)
                        for name in self.wheel_locations}
        wheel_speeds = {name: np.empty_like(times)
                        for name in self.wheel_locations}
        wheel_omegas = {name: np.empty_like(times)
                        for name in self.wheel_locations}

        for j, (tj, i1) in enumerate(zip(times, idx)):
            i0 = i1 - 1
            # fractional position between i0 and i1
            alpha = (tj - t[i0]) / max(t[i1] - t[i0], 1e-12)

            xj = lerp(xs[i0], xs[i1], alpha)
            yj = lerp(ys[i0], ys[i1], alpha)
            # handle theta wrap
            dth = self._wrap_to_pi(ths[i1] - ths[i0])
            thj = self._wrap_to_pi(ths[i0] + alpha * dth)
            kj = lerp(kappas[i0], kappas[i1], alpha)
            vj = max(0.0, lerp(v[i0], v[i1], alpha))

            xs_out[j] = xj
            ys_out[j] = yj
            th_out[j] = thj
            k_out[j] = kj
            v_out[j] = vj

            # per-wheel angles and speeds
            ang = self._steer_angles_from_kappa(kj)
            spd = self._per_wheel_speed(vj, kj, ang)
            for name in self.wheel_locations:
                a = float(ang.get(name, 0.0))
                s = float(spd[name])
                wheel_angles[name][j] = a
                wheel_speeds[name][j] = s
                wheel_omegas[name][j] = (
                    2.0 * s) / max(self.limits.wheel_diameter, 1e-9)  # rad/s

        return {
            "t": times,
            "x": xs_out,
            "y": ys_out,
            "theta": th_out,
            "kappa": k_out,
            "v": v_out,
            "wheel_angles": wheel_angles,
            "wheel_speeds": wheel_speeds,
            "wheel_omegas": wheel_omegas,
        }


# -------------------- Example usage --------------------
if __name__ == "__main__":
    from rover_constants import WHEEL_LOCATIONS, STEERABLE_WHEELS
    limits = RoverLimits(
        wheel_diameter=WHEEL_DIAMETER / 1000,  # Convert mm to m
        wheel_speed_max=60,            # [rpm]
        steer_angle_max=math.radians(30.0),  # [rad]
        accel_max=0.8,                  # [m/s^2]
        decel_max=1.2,                  # [m/s^2]
    )

    # Convert wheel locations from mm to m
    wheel_locations_m = {name: (x/1000.0, y/1000.0)
                         for name, (x, y) in WHEEL_LOCATIONS.items()}

    planner = AckermannPlanner(
        wheel_locations_m, STEERABLE_WHEELS, limits)

    start = (0.0, 0.0, 0.0)
    goal = (3.0, 2.0, math.radians(90.0))

    traj = planner.plan(start, goal, dt=0.02)

    # Quick peek
    print(f"Total time: {traj['t'][-1]:.2f} s, samples: {len(traj['t'])}")
    print("At t=0.0: steer angles (deg):",
          {k: math.degrees(traj['wheel_angles'][k][0]) for k in STEERABLE_WHEELS})
    print("At t=end: steer angles (deg):",
          {k: math.degrees(traj['wheel_angles'][k][-1]) for k in STEERABLE_WHEELS})

    # Plot the path
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(traj['x'], traj['y'], label='Path')
    plt.quiver(traj['x'], traj['y'], np.cos(traj['theta']), np.sin(traj['theta']),
               angles='xy', scale_units='xy', scale=1, color='r', label='Heading')
    plt.xlim(-1, 4)
    plt.ylim(-1, 3)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Rover Path')
    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()
