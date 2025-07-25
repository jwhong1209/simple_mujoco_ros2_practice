#include "TrajectoryGenerator.hpp"

#include <cmath>

using namespace std;

template <typename T>
Vec2<T> TrajectoryGenerator<T>::cubic(const T t,           // current time
                                      const T t0,          // initial time
                                      const T tf,          // final time
                                      const Vec2<T> & p0,  // initial position
                                      const Vec2<T> & pf,  // final position
                                      const Vec2<T> & v0,  // initial velocity
                                      const Vec2<T> & vf)  // final velocity
{
  T dt = tf - t0;
  if (dt == 0)
  {
    return p0;
  }

  Vec2<T> dp = pf - p0;

  Vec2<T> a0 = p0;
  Vec2<T> a1 = v0;
  Vec2<T> a2 = (3 * dp) / (dt * dt) - (2 * v0) / dt - vf / dt;
  Vec2<T> a3 = -(2 * dp) / (dt * dt * dt) + (v0 + vf) / (dt * dt);

  return a0 + a1 * (t - t0) + a2 * (t - t0) * (t - t0) + a3 * (t - t0) * (t - t0) * (t - t0);
}

template <typename T>
Vec2<T> TrajectoryGenerator<T>::circular(const T t,           // current time
                                         const T t0,          // initial time
                                         const T r,           // radius
                                         const T Hz,          // frequency
                                         const Vec2<T> & p0)  // initial position
{
  const T w = 2 * M_PI * Hz;
  Vec2<T> p_des = p0;

  p_des(0) += r * (1 - cos(w * (t - t0)));
  p_des(1) += r * sin(w * (t - t0));
  return p_des;
}

template <typename T>
Vec2<T> TrajectoryGenerator<T>::circularDDot(const T t,   // current time
                                             const T t0,  // initial time
                                             const T r,   // radius
                                             const T Hz)  // frequency
{
  const T w = 2 * M_PI * Hz;
  Vec2<T> a_des = Vec2<T>::Zero();

  a_des(0) = (r * w * w) * cos(w * (t - t0));
  a_des(1) = -(r * w * w) * sin(w * (t - t0));
  return a_des;
}

template class TrajectoryGenerator<float>;
template class TrajectoryGenerator<double>;