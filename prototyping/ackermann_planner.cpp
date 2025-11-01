// ackermann_planner.cpp
#include "ackermann_planner.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace {
  inline double mod2pi(double a) {
    double two_pi = 2.0 * M_PI;
    double r = std::fmod(a, two_pi);
    if (r < 0) r += two_pi;
    return r;
  }
}

AckermannPlanner::AckermannPlanner(const WheelMap& wheel_locations,
  const NameSet& steerable,
  const RoverLimits& limits)
  : wheels_(wheel_locations), steerable_(steerable), limits_(limits) {
  kappa_max_ = kappa_max_from_geometry(limits_.steer_angle_max);
  if (kappa_max_ <= 0.0) {
    throw std::runtime_error("Could not compute positive kappa_max from geometry");
  }
  Rmin_ = 1.0 / kappa_max_;
}

double AckermannPlanner::wrap_to_pi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

double AckermannPlanner::kappa_max_from_geometry(double steer_angle_max) const {
  double Lref = 1e-3;
  for (const auto& kv : wheels_) {
    Lref = std::max(Lref, std::abs(kv.second.x()));
  }
  int K = 6000;
  double kmax = 0.0;
  for (int i = 0; i < K; ++i) {
    double k = (10.0 / Lref) * (static_cast<double>(i) / static_cast<double>(K - 1));
    bool ok = true;
    if (k >= 1e-9) {
      double R = 1.0 / k;
      for (const auto& name : steerable_) {
        auto it = wheels_.find(name);
        if (it == wheels_.end()) continue;
        double x_i = it->second.x();
        double y_i = it->second.y();
        double delta = std::atan2(x_i, (R - y_i));
        if (std::abs(delta) > steer_angle_max + 1e-9) { ok = false; break; }
      }
    }
    if (ok) kmax = k; else break;
  }
  return kmax;
}

std::unordered_map<std::string, double>
AckermannPlanner::steer_angles_from_kappa(double kappa) const {
  std::unordered_map<std::string, double> ang;
  for (const auto& kv : wheels_) ang[kv.first] = 0.0;
  if (std::abs(kappa) < 1e-12) return ang;
  double R = 1.0 / kappa;
  for (const auto& name : steerable_) {
    auto it = wheels_.find(name);
    if (it == wheels_.end()) continue;
    double x_i = it->second.x();
    double y_i = it->second.y();
    ang[name] = std::atan2(x_i, (R - y_i));
  }
  return ang;
}

std::unordered_map<std::string, double>
AckermannPlanner::per_wheel_speed(double v_center,
  double kappa,
  const std::unordered_map<std::string, double>& wheel_angles) const {
  double omega = v_center * kappa; // yaw rate
  std::unordered_map<std::string, double> speeds;
  for (const auto& kv : wheels_) {
    const std::string& name = kv.first;
    const Eigen::Vector2d& p = kv.second;
    double x_i = p.x();
    double y_i = p.y();
    double vx = v_center - omega * y_i;
    double vy = omega * x_i;
    auto it = wheel_angles.find(name);
    double delta = (it != wheel_angles.end()) ? it->second : 0.0;
    double ux = std::cos(delta), uy = std::sin(delta);
    speeds[name] = vx * ux + vy * uy;
  }
  return speeds;
}

std::vector<AckermannPlanner::Seg>
AckermannPlanner::dubins_shortest_path(const Eigen::Vector3d& q0,
  const Eigen::Vector3d& q1,
  double Rmin) const {
  double x0 = q0.x(), y0 = q0.y(), th0 = q0.z();
  double x1 = q1.x(), y1 = q1.y(), th1 = q1.z();
  double dx = x1 - x0, dy = y1 - y0;
  double c = std::cos(th0), s = std::sin(th0);

  double x = (c * dx + s * dy) / Rmin;
  double y = (-s * dx + c * dy) / Rmin;
  double th = wrap_to_pi(th1 - th0);

  auto polar = [](double X, double Y) { return std::make_pair(std::hypot(X, Y), std::atan2(Y, X)); };
  auto clamp = [](double a, double lo, double hi) { return (a < lo) ? lo : ((a > hi) ? hi : a); };

  const double EPS = 1e-9;
  using Sol = std::pair<double, std::vector<Seg>>;
  std::vector<Sol> sols;

  // LSL
  {
    auto pr = polar(x - std::sin(th), y - 1.0 + std::cos(th));
    double r = pr.first, t = pr.second;
    double t1 = mod2pi(t);
    double t2 = r;
    double t3 = mod2pi(th - t);
    sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'L', t1* Rmin}, {'S', t2 * Rmin}, {'L', t3 * Rmin}});
  }
  // RSR
  {
    auto pr = polar(x + std::sin(th), y + 1.0 - std::cos(th));
    double r = pr.first, t = pr.second;
    double t1 = mod2pi(-t);
    double t2 = r;
    double t3 = mod2pi(-wrap_to_pi(th) + t);
    sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'R', t1* Rmin}, {'S', t2 * Rmin}, {'R', t3 * Rmin}});
  }
  // LSR
  {
    auto pr = polar(x - std::sin(th), y - 1.0 + std::cos(th));
    double r0 = pr.first, t0 = pr.second;
    double rr = std::hypot(r0, 2.0);
    if (rr >= 2.0 - EPS) {
      double t2 = std::sqrt(std::max(0.0, rr * rr - 4.0));
      double theta = std::atan2(2.0, r0);
      double t1 = mod2pi(t0 + theta);
      double t3 = mod2pi(-wrap_to_pi(th) - t0 + theta);
      sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'L', t1* Rmin}, {'S', t2 * Rmin}, {'R', t3 * Rmin}});
    }
  }
  // RSL
  {
    auto pr = polar(x + std::sin(th), y + 1.0 - std::cos(th));
    double r0 = pr.first, t0 = pr.second;
    double rr = std::hypot(r0, 2.0);
    if (rr >= 2.0 - EPS) {
      double t2 = std::sqrt(std::max(0.0, rr * rr - 4.0));
      double theta = std::atan2(2.0, r0);
      double t1 = mod2pi(-t0 + theta);
      double t3 = mod2pi(wrap_to_pi(th) - t0 + theta);
      sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'R', t1* Rmin}, {'S', t2 * Rmin}, {'L', t3 * Rmin}});
    }
  }
  // RLR
  {
    auto pr = polar(x - std::sin(th), y - 1.0 + std::cos(th));
    double r0 = pr.first, t0 = pr.second;
    double d = 0.25 * (2.0 + r0 * r0);
    if (d <= 1.0 + EPS) {
      double phi = std::acos(clamp(d, -1.0, 1.0));
      double t1 = mod2pi(-t0 + phi + M_PI / 2.0);
      double t2 = mod2pi(2 * M_PI - 2 * phi);
      double t3 = mod2pi(wrap_to_pi(th) - t0 + phi + M_PI / 2.0);
      sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'R', t1* Rmin}, {'L', t2 * Rmin}, {'R', t3 * Rmin}});
    }
  }
  // LRL
  {
    auto pr = polar(x + std::sin(th), y + 1.0 - std::cos(th));
    double r0 = pr.first, t0 = pr.second;
    double d = 0.25 * (2.0 + r0 * r0);
    if (d <= 1.0 + EPS) {
      double phi = std::acos(clamp(d, -1.0, 1.0));
      double t1 = mod2pi(t0 - phi - M_PI / 2.0);
      double t2 = mod2pi(2 * M_PI - 2 * phi);
      double t3 = mod2pi(-wrap_to_pi(th) + t0 - phi - M_PI / 2.0);
      sols.emplace_back(t1 + t2 + t3, std::vector<Seg>{{'L', t1* Rmin}, {'R', t2 * Rmin}, {'L', t3 * Rmin}});
    }
  }

  if (sols.empty()) {
    // Should not happen with clamping; fallback: small perturbation
    Eigen::Vector3d q1p = q1;
    q1p.z() += 1e-9;
    return dubins_shortest_path(q0, q1p, Rmin);
  }

  auto best = std::min_element(sols.begin(), sols.end(),
    [](const Sol& a, const Sol& b) { return a.first < b.first; });
  return best->second;
}

void AckermannPlanner::propagate_exact(const Eigen::Vector3d& q0,
  const std::vector<Seg>& segs,
  double Rmin,
  double ds,
  std::vector<double>& ss,
  std::vector<double>& xs,
  std::vector<double>& ys,
  std::vector<double>& ths,
  std::vector<double>& ks) const {
  double x = q0.x(), y = q0.y(), th = q0.z();
  xs.clear(); ys.clear(); ths.clear(); ss.clear(); ks.clear();
  xs.push_back(x); ys.push_back(y); ths.push_back(th); ss.push_back(0.0); ks.push_back(0.0);

  for (const auto& seg : segs) {
    char kind = seg.first; double L = seg.second;
    if (kind == 'S') {
      int n = std::max(1, static_cast<int>(std::ceil(L / ds)));
      double step = L / static_cast<double>(n);
      for (int i = 0; i < n; ++i) {
        x += std::cos(th) * step;
        y += std::sin(th) * step;
        xs.push_back(x); ys.push_back(y); ths.push_back(th);
        ss.push_back(ss.back() + step); ks.push_back(0.0);
      }
    }
    else {
      int sgn = (kind == 'L') ? +1 : -1;
      double dth_total = sgn * (L / Rmin);
      int n = std::max(1, static_cast<int>(std::ceil(std::abs(dth_total) * Rmin / std::max(ds, 1e-6))))
        ;
      double dth = dth_total / static_cast<double>(n);
      for (int i = 0; i < n; ++i) {
        double xc = x - sgn * Rmin * std::sin(th);
        double yc = y + sgn * Rmin * std::cos(th);
        double th_new = th + dth;
        x = xc + sgn * Rmin * std::sin(th_new);
        y = yc - sgn * Rmin * std::cos(th_new);
        th = th_new;
        xs.push_back(x); ys.push_back(y); ths.push_back(th);
        ss.push_back(ss.back() + std::abs(dth) * Rmin);
        ks.push_back(static_cast<double>(sgn) / Rmin);
      }
    }
  }
}

void AckermannPlanner::time_scale_with_wheels(const std::vector<double>& ss,
  const std::vector<double>& kappas,
  std::vector<double>& t,
  std::vector<double>& v) const {
  size_t N = ss.size();
  v.assign(N, std::numeric_limits<double>::infinity());

  // Wheel speed envelope (binary search per sample)
  for (size_t i = 0; i < N; ++i) {
    double k = kappas[i];
    auto ang = steer_angles_from_kappa(k);
    double vlo = 0.0, vhi = limits_.linear_speed_max() * 2.0 + 1e-6;
    for (int it = 0; it < 28; ++it) {
      double vm = 0.5 * (vlo + vhi);
      auto speeds = per_wheel_speed(vm, k, ang);
      double smax = 0.0;
      for (const auto& kv : speeds) smax = std::max(smax, std::abs(kv.second));
      if (smax <= limits_.linear_speed_max()) vlo = vm; else vhi = vm;
    }
    v[i] = vlo;
  }

  // Forward accel pass
  for (size_t i = 1; i < N; ++i) {
    double ds = ss[i] - ss[i - 1];
    double vmax = std::sqrt(std::max(0.0, v[i - 1] * v[i - 1] + 2.0 * limits_.accel_max * ds));
    v[i] = std::min(v[i], vmax);
  }
  // Backward decel pass
  for (size_t i = N - 1; i-- > 0; ) {
    double ds = ss[i + 1] - ss[i];
    double vmax = std::sqrt(std::max(0.0, v[i + 1] * v[i + 1] + 2.0 * limits_.decel_max * ds));
    v[i] = std::min(v[i], vmax);
  }

  // Integrate time: dt = ds / v_mid
  t.assign(N, 0.0);
  for (size_t i = 1; i < N; ++i) {
    double ds = ss[i] - ss[i - 1];
    double v_mid = std::max(1e-6, 0.5 * (v[i] + v[i - 1]));
    t[i] = t[i - 1] + ds / v_mid;
  }
}

PlanResult AckermannPlanner::sample_uniform_dt(const std::vector<double>& t,
  const std::vector<double>& ss,
  const std::vector<double>& xs,
  const std::vector<double>& ys,
  const std::vector<double>& ths,
  const std::vector<double>& kappas,
  const std::vector<double>& v,
  double dt) const {
  PlanResult out;
  if (t.back() <= 0.0) {
    out.t = Eigen::VectorXd::Constant(1, 0.0);
    out.x = Eigen::VectorXd::Constant(1, xs.front());
    out.y = Eigen::VectorXd::Constant(1, ys.front());
    out.theta = Eigen::VectorXd::Constant(1, ths.front());
    out.kappa = Eigen::VectorXd::Constant(1, 0.0);
    out.v = Eigen::VectorXd::Constant(1, 0.0);
    for (const auto& w : wheels_) {
      out.wheel_angles[w.first] = Eigen::VectorXd::Constant(1, 0.0);
      out.wheel_speeds[w.first] = Eigen::VectorXd::Constant(1, 0.0);
      out.wheel_omegas[w.first] = Eigen::VectorXd::Constant(1, 0.0);
    }
    return out;
  }

  double T = t.back();
  int M = static_cast<int>(std::floor(T / dt)) + 1;
  std::vector<double> times; times.reserve(M);
  for (int i = 0; i <= M; ++i) times.push_back(std::min(T, i * dt));

  auto lerp = [](double a, double b, double alpha) { return (1.0 - alpha) * a + alpha * b; };

  std::vector<double> x_out, y_out, th_out, k_out, v_out;
  x_out.resize(times.size()); y_out.resize(times.size()); th_out.resize(times.size());
  k_out.resize(times.size()); v_out.resize(times.size());

  // Prepare per-wheel arrays
  for (const auto& kv : wheels_) {
    out.wheel_angles[kv.first] = Eigen::VectorXd::Zero(times.size());
    out.wheel_speeds[kv.first] = Eigen::VectorXd::Zero(times.size());
    out.wheel_omegas[kv.first] = Eigen::VectorXd::Zero(times.size());
  }

  // For searchsorted behavior
  auto lower_bound_idx = [&](double value) {
    return static_cast<int>(std::lower_bound(t.begin(), t.end(), value) - t.begin());
  };

  for (size_t j = 0; j < times.size(); ++j) {
    double tj = times[j];
    int i1 = std::min(static_cast<int>(t.size()) - 1, std::max(1, lower_bound_idx(tj)));
    int i0 = i1 - 1;
    double alpha = (tj - t[i0]) / std::max(t[i1] - t[i0], 1e-12);

    double xj = lerp(xs[i0], xs[i1], alpha);
    double yj = lerp(ys[i0], ys[i1], alpha);
    double dth = wrap_to_pi(ths[i1] - ths[i0]);
    double thj = wrap_to_pi(ths[i0] + alpha * dth);
    double kj = lerp(kappas[i0], kappas[i1], alpha);
    double vj = std::max(0.0, lerp(v[i0], v[i1], alpha));

    x_out[j] = xj; y_out[j] = yj; th_out[j] = thj; k_out[j] = kj; v_out[j] = vj;

    auto ang = steer_angles_from_kappa(kj);
    auto spd = per_wheel_speed(vj, kj, ang);
    for (const auto& kv : wheels_) {
      const std::string& name = kv.first;
      double a = ang.count(name) ? ang[name] : 0.0;
      double s = spd[name];
      out.wheel_angles[name](j) = a;
      out.wheel_speeds[name](j) = s;
      out.wheel_omegas[name](j) = (2.0 * s) / std::max(limits_.wheel_diameter, 1e-9);
    }
  }

  // Move to Eigen
  out.t = Eigen::Map<const Eigen::VectorXd>(times.data(), static_cast<int>(times.size()));
  out.x = Eigen::Map<const Eigen::VectorXd>(x_out.data(), static_cast<int>(x_out.size()));
  out.y = Eigen::Map<const Eigen::VectorXd>(y_out.data(), static_cast<int>(y_out.size()));
  out.theta = Eigen::Map<const Eigen::VectorXd>(th_out.data(), static_cast<int>(th_out.size()));
  out.kappa = Eigen::Map<const Eigen::VectorXd>(k_out.data(), static_cast<int>(k_out.size()));
  out.v = Eigen::Map<const Eigen::VectorXd>(v_out.data(), static_cast<int>(v_out.size()));
  return out;
}

PlanResult AckermannPlanner::plan(const Eigen::Vector3d& start_pose,
  const Eigen::Vector3d& goal_pose,
  double dt,
  double ds_prop) const {
  // 1) Shortest Dubins sequence
  auto segs = dubins_shortest_path(start_pose, goal_pose, Rmin_);

  // 2) Exact propagation along centerline
  std::vector<double> ss, xs, ys, ths, kappas;
  propagate_exact(start_pose, segs, Rmin_, ds_prop, ss, xs, ys, ths, kappas);

  // 3) Time-scaling with wheel speed and accel/decel limits
  std::vector<double> t, v;
  time_scale_with_wheels(ss, kappas, t, v);

  // 4) Uniform dt sampling + wheel angles/speeds
  return sample_uniform_dt(t, ss, xs, ys, ths, kappas, v, dt);
}
