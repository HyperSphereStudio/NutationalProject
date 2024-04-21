#ifndef SYSTEM_H
#define SYSTEM_H

#include "RK4.hpp"
#include "vector_datatype/src/quaternion_type.h"

#define DATA_SIZE 4

struct System{
    //Constants
    float mass;                                  //Mass of the system
    float pivot_to_com_bf[3];                    //Pivot point to center of mass fixed frame
    float J_bf[3];                               //Diagonal Matrix Moment of Inertia of Body as vector in body fixed frame
    float Jr_bf[3];                              //Diagonal Matrix Moment of Inertia of Reaction as vector in body fixed frame
    float u_h_dot_bf[3];                         //Unit direction of control torque in the body frame          

    float a;                                     //Desired Angle cos(θ)
    float b;                                     //Reaction Angular Moment Coefficient
    float t0;                                         

    float* s;                                    //Data Array
};

System Sy;
RK4<float, DATA_SIZE> rk4 = RK4<float, DATA_SIZE>(nullptr);

//Intertial to Fixed Frame.      Derived from accelerometer
quat_t Q(float* s){ return quat_t(&s[0]); }
void Q(float* s, quat_t& q){ q.copyArray(&s[0]); }

vec3_t H_bf(float* s){ return vec3_t(&s[4]); }
void H_bf(float* s, vec3_t& v){ v.copyArray(&s[4]); }

void xdot(sdot, s, hdot){
  //q̇ = 1/2qΩ 
  q!(ṡ, q * Quat(ω_bf(s)..., 0) / 2)
}

void init_system(){
  auto r_o = 10E-2;
  auto r_i = 0E-2;
  auto h = 10E-2;

  Sy.mass = .25;

  Sy.u_h_dot_bf[0] = 1;
  Sy.u_h_dot_bf[1] = 0;
  Sy.u_h_dot_bf[2] = 0;

  Sy.pivot_to_com_bf[0] = 0;
  Sy.pivot_to_com_bf[1] = 0;
  Sy.pivot_to_com_bf[2] = h / 2;
  
  Sy.b = 1;

  Sy.s = &rk4.u;    //So dont have to copy the values over. Direct ref
}

void update_system(float dt){
  Q(Sy.s, Q(Sy.s).norm());    //Prevent numeric drifting
  rk4.step(dt);  
}