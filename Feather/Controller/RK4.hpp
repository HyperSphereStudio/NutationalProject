/**********************************************************************
   NAME: Rk4.hpp
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023

   C++ extension of the julia Rk4
*********************************************************************/

#ifndef RK4_H
#define RK4_H

template<typename T, int N>
struct RK4{
  T u[N];
  T udot[N];
  T k1[N];
  T k2[N];
  T k3[N];
  T k4[N];
  T U[N];

  typedef void Fcn(T* s, T* sdot, int n);

  T t;
  int i;
  Fcn* xdot;

  RK4(Fcn* x_dot_f) : xdot(x_dot_f), t(0), i(0){}

  void step(T dt){
    auto dth = dt/2;

    xdot(&u, &k1, N);

    for(auto i = 0; i < N; i++)
      U[i] = u[i] + dth * k1[i];
    xdot(&U, &k2, N);

    for(auto i = 0; i < N; i++)
      U[i] = u[i] + dth * k2[i];
    xdot(&U, &k3, N);

    for(auto i = 0; i < N; i++)
      U[i] = u[i] + dt * k3[i];
    xdot(&U, &k4, N);

    for(auto i = 0; i < N; i++)
      u[i] += dt/6 * (k1[i] + k2[i] + k3[i] + k4[i]);

    t += dt;
    i += 1;
  }

};

#endif //RK4_H