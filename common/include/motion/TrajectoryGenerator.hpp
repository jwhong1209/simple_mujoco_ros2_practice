#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include "eigen_types.hpp"

// TODO: Add simple trajectory e.g. sinusoidal or circular

template <typename T>
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(){};

  // void step();
  // void sinusoidal();

  Vec2<T> cubic(const T t,                              // current time
                const T t0,                             // initial time
                const T tf,                             // final time
                const Vec2<T> & p0,                     // initial position
                const Vec2<T> & pf,                     // final position
                const Vec2<T> & v0,                     // initial velocity
                const Vec2<T> & vf = Vec2<T>::Zero());  // final velocity

  Vec2<T> circular(const T t,            // current time
                   const T t0,           // initial time
                   const T r,            // radius
                   const T Hz,           // frequency
                   const Vec2<T> & p0);  // initial position

  Vec2<T> circularDDot(const T t,    // current time
                       const T t0,   // initial time
                       const T r,    // radius
                       const T Hz);  // frequency
};

#endif  // TRAJECTORY_GENERATOR_HPP_