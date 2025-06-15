#pragma once

#include <cmath>

#include "cvec.h"
#include "quat.h"
#include "asst5.h"

inline Cvec3 cvecCRS(Cvec3 c0, Cvec3 c1, Cvec3 c2, Cvec3 c3, float alpha) {
  const Cvec3 d = (c2 - c0) / 6 + c1;
  const Cvec3 e = -(c3 - c1) / 6 + c2;
  return c1 * pow(1 - alpha, 3) + d * 3 * alpha * pow(1 - alpha, 2) +
  	e * 3 * pow(alpha, 2) * (1 - alpha) + c2 * pow(alpha, 3);
}

inline Quat quatCRS(Quat q0, Quat q1, Quat q2, Quat q3, float alpha) {
  const Quat d = pow(q2 * inv(q0), 1.0 / 6) * q1;
  const Quat e = pow(q3 * inv(q1), -1.0 / 6) * q2;
  const Quat p01 = slerp(q1, d, alpha);
  const Quat p12 = slerp(d, e, alpha);
  const Quat p23 = slerp(e, q2, alpha);
  const Quat p012 = slerp(p01, p12, alpha);
  const Quat p123 = slerp(p12, p23, alpha);
  return slerp(p012, p123, alpha);
}
