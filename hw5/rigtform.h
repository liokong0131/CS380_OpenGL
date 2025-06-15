#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    for (int i = 0; i < 3; i++)
      t_[i] = t[i];
    
    for (int i = 0; i < 4; i++)
      r_[i] = r[i];
  }

  explicit RigTForm(const Cvec3& t) {
    for (int i = 0; i < 3; i++)
      t_[i] = t[i];
  }

  explicit RigTForm(const Quat& r) {
    for (int i = 0; i < 4; i++)
      r_[i] = r[i];
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    return r_ * a + Cvec4(t_, 0);
  }

  RigTForm operator * (const RigTForm& a) const {
    return RigTForm(t_ + Cvec3(r_ * Cvec4(a.t_, 0)), r_ * a.r_);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  const Quat inv_r = inv(tform.getRotation());
  const Cvec3 t = tform.getTranslation();
  return RigTForm(Cvec3(inv_r * Cvec4(t, 0) * (-1)), inv_r);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  return Matrix4::makeTranslation(tform.getTranslation()) * quatToMatrix(tform.getRotation());
}
inline RigTForm doMtoOwrtA_r(const RigTForm& m, const RigTForm& A, const RigTForm& o) {
  return A * m * inv(A) * o;
}

inline RigTForm makeMixedFrame_r(const RigTForm& o, const RigTForm& e) {
  return RigTForm(o.getTranslation(), e.getRotation());
}
#endif
