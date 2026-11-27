#pragma once
#include <math.h>

// ══════════════════════════════════════════════════════════
// ServoProfile — 시간 기반 사다리꼴 위치 궤적 (서보 위치 루프의 기준값)
//
// 거리 D(step, 부호 있음)를 정확히 T초에 끝내도록 계획한다.
//   가속 r초 → 정속 vPeak → 감속 r초, r ≤ T/2
// 가속도 한계 A를 지키는 최소 램프:  r(T−r) ≥ |D|/A
//   → r_min = (T − √(T² − 4|D|/A)) / 2
// 실제 램프는 max(r_min, 원하는 램프시간)이며, 정속 속도는
//   vPeak = |D| / (T − r)
// 이 값이 최고 속도를 넘으면 계획 자체를 거부한다(움직이기 전에 실패).
//
// Motor.cpp가 이 기준 위치와 엔코더 실측 위치의 차이로 속도 명령을 보정한다.
// ══════════════════════════════════════════════════════════
struct ServoProfile {
  double dist  = 0;  // step, 부호 있음
  double total = 0;  // s
  double ramp  = 0;  // s
  double vPeak = 0;  // step/s (크기)
  double accel = 0;  // step/s² (크기)

  static bool make(double distance, double seconds, double rampSec,
                   double maxSpeed, double maxAccel, ServoProfile &out) {
    if (!isfinite(distance) || !isfinite(seconds) || !isfinite(rampSec) ||
        seconds <= 0 || seconds > 3600 || maxSpeed <= 0 || maxAccel <= 0 ||
        rampSec < 0) return false;
    const double d = fabs(distance);
    if (d < 0.5) return false;                 // 1 step 미만은 계획하지 않음
    const double disc = seconds * seconds - 4.0 * d / maxAccel;
    if (disc < 0) return false;                // 가속도 한계로 시간 안에 불가능
    double r = (seconds - sqrt(disc)) / 2.0;
    const double want = rampSec < seconds / 2 ? rampSec : seconds / 2;
    if (r < want) r = want;
    if (r <= 0) r = seconds / 2;
    if (r > seconds / 2) r = seconds / 2;
    const double v = d / (seconds - r);
    const double a = v / r;
    if (v > maxSpeed * (1 + 1e-9) || a > maxAccel * (1 + 1e-6)) return false;
    out.dist = distance; out.total = seconds; out.ramp = r; out.vPeak = v; out.accel = a;
    return true;
  }

  // 시작 후 t초 시점의 (시작점 기준) 위치와 속도
  void sample(double t, double &pos, double &vel) const {
    const double s = dist < 0 ? -1.0 : 1.0;
    const double d = fabs(dist);
    double p, v;
    if (t <= 0)                 { p = 0; v = 0; }
    else if (t < ramp)          { p = 0.5 * accel * t * t; v = accel * t; }
    else if (t <= total - ramp) { p = 0.5 * accel * ramp * ramp + vPeak * (t - ramp); v = vPeak; }
    else if (t < total)         { const double q = total - t; p = d - 0.5 * accel * q * q; v = accel * q; }
    else                        { p = d; v = 0; }
    pos = s * p; vel = s * v;
  }
};
