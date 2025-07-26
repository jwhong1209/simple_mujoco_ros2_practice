#ifndef DOUBLE_PENDULUM_MODEL_HPP_
#define DOUBLE_PENDULUM_MODEL_HPP_

#include "eigen_types.hpp"

template <typename T>
class DoublePendulumModel
{
private:
  T l1_, l2_;  // link length
  T d1_, d2_;  // distance from joint axis to link's CoM
  T m1_, m2_;  // link's mass
  T I1_, I2_;  // link's momentum of inertia w.r.t CoM
  T J1_, J2_;  // link's momentum of inertia w.r.t axis

  Vec2<T> q_;    // joint pos
  Vec2<T> dq_;   // joint vel
  Vec2<T> ddq_;  // joint acc

  Vec2<T> p_;  // Cartesian position
  Vec2<T> v_;  // Cartesian velocity
  Vec2<T> a_;  // Cartesian acceleration
  Mat2<T> J_;  // Jacobian matrix

  Mat2<T> M_;      // inertia matrix
  Vec2<T> tau_c_;  // coriolis
  Vec2<T> tau_g_;  // gravity

public:
  DoublePendulumModel();

  //* ----- SETTERS --------------------------------------------------------------------------------
  void setJointStates(const Vec2<T> & q, const Vec2<T> & dq);

  //* ----- METHODS --------------------------------------------------------------------------------
  /* Kinematics */
  Vec2<T> position();
  Vec2<T> velocity();
  Mat2<T> jacobian();

  Vec2<T> inverseKinematics(const Vec2<T> & p_des);

  /* Dynamics */
  // https://robotics.stackexchange.com/questions/19861/model-of-two-link-rigid-manipulator
  Mat2<T> inertia();
  Vec2<T> coriolis();
  Vec2<T> gravity();

  //* ----- PRINTER --------------------------------------------------------------------------------
};

#endif  // DOUBLE_PENDULUM_MODEL_HPP_