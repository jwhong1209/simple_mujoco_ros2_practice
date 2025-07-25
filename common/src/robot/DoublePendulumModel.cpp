#include "DoublePendulumModel.hpp"

#include <cmath>

using namespace std;

template <typename T>
DoublePendulumModel<T>::DoublePendulumModel()
{
  l1_ = l2_ = T(1.0);
  d1_ = d2_ = T(0.5);
  m1_ = m2_ = T(1.0);
  I1_ = T(0.03);
  I2_ = T(0.07);

  /* compute links' momentum of inertia w.r.t rotating axis */
  J1_ = I1_ + m1_ * (d1_ * d1_);
  J2_ = I2_ + m2_ * (d2_ * d2_);
}

//* ----- SETTERS ----------------------------------------------------------------------------------
template <typename T>
void DoublePendulumModel<T>::setJointStates(const Vec2<T> & q, const Vec2<T> & dq)
{
  q_ = q;
  dq_ = dq;
}

//* ----- GETTERS ----------------------------------------------------------------------------------
/* Kinematics */
template <typename T>
Vec2<T> DoublePendulumModel<T>::position()
{
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  p_(0) = l1_ * cos(q1) + l2_ * cos(q12);  // x-coordinate
  p_(1) = l1_ * sin(q1) + l2_ * sin(q12);  // y-coordinate
  return p_;
}

template <typename T>
Vec2<T> DoublePendulumModel<T>::velocity()
{
  v_ = J_ * dq_;
  return v_;
}

template <typename T>
Mat2<T> DoublePendulumModel<T>::jacobian()
{
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  J_(0, 0) = -l1_ * sin(q1) - l2_ * sin(q12);
  J_(0, 1) = -l2_ * sin(q12);
  J_(1, 0) = l1_ * cos(q1) + l2_ * cos(q12);
  J_(1, 1) = l2_ * cos(q12);

  return J_;
}

/* Dynamics */
template <typename T>
Mat2<T> DoublePendulumModel<T>::inertia()
{
  const T q2 = q_(1);

  M_(0, 0) = J1_ + J2_ + m2_ * (l1_ * l1_) + 2 * m2_ * l1_ * d2_ * cos(q2);
  M_(0, 1) = M_(1, 0) = J2_ + m2_ * l1_ * d2_ * cos(q2);
  M_(1, 1) = J2_;

  return M_;
}

template <typename T>
Vec2<T> DoublePendulumModel<T>::coriolis()
{
  const T q1 = q_(0);
  const T q2 = q_(1);

  tau_c_(0) = -m2_ * l1_ * d2_ * sin(q2) * (q2 * q2 + 2 * q1 * q2);
  tau_c_(1) = m2_ * l1_ * d2_ * sin(q2) * (q1 * q1);

  return tau_c_;
}

template <typename T>
Vec2<T> DoublePendulumModel<T>::gravity()
{
  const T g = 9.81;
  const T q1 = q_(0);
  const T q12 = q_(0) + q_(1);

  tau_g_(0) = g * (m1_ * d1_ + m2_ * l1_) * cos(q1) + g * m2_ * d2_ * cos(q12);
  tau_g_(1) = g * m2_ * d2_ * cos(q12);

  return tau_g_;
}

template class DoublePendulumModel<float>;
template class DoublePendulumModel<double>;