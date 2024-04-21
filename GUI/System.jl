#=
   NAME: System.jl
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
=#

#=
An Introduction to Physically Based Modeling:
Rigid Body Simulation I—Unconstrained Rigid
Body Dynamics

(2) The Quaternions with an application to Rigid
Body Dynamics
Evangelos A. Coutsias† and Louis Romero‡
Department of Mathematics and Statistics,
University of New Mexico
=#
using LinearAlgebra

const Eps::Float32 = 1E-3

#Intertial to Fixed Frame.      Derived from accelerometer
q(x) = Quat(x[1:4]...)
q!(x, v::Quat) = x[1:4] .= v.data

#Measuered by Gyroscope.        Should be relatively constant
ω_bf(x) = SVector(x[5:7]...)
ω_bf!(x, v) = x[5:7] .= v

#Angular Momentum of the reaction wheel in the body fixed frame
h_bf(x) = SVector(x[8:10]...)
h_bf!(x, v) = x[8:10] .= v

mutable struct System
    #Constants
    mass::Float32                           #Mass of the system
    pivot_to_com_bf::SVec                   #Pivot point to center of mass fixed frame
    J_bf::SVec                              #Diagonal Matrix Moment of Inertia of Body as vector in body fixed frame
    Jr_bf::SVec                             #Diagonal Matrix Moment of Inertia of Reaction as vector in body fixed frame
    u_ḣ_bf::SVec                            #Unit direction of control torque in the body frame          

    α::Float32                              #Desired Angle cos(θ)
    β::Float32                              #Reaction Angular Moment Coefficient
    t0::Int64                                         

    s::Vector{Float32}                      #Data Array

    System() = new()
end

const Sy = System()
const ODE = RK4{Float32}(println, 10)   
const EmptyS = MVector{10, Float32}

#Eulers Rotational Equation
#Differential Equation governing angular velocity in fixed frame
#Differential State of the System

function ẋ!(ṡ, s, ḣ)
    #T_ext = T_g = q^c * (q * pivot × F_g_sp)
    Fg_sp = Sy.mass * SVec(0, 0, -9.8)
    R_sp = q(s) * Sy.pivot_to_com_bf                   #Contact point to com in the inertial frame
    τ_sp = cross(R_sp, Fg_sp)                           #Torque due to gravity in the intertial frame
    τ_bf = conj(q(s)) * τ_sp                            #Torque due to gravity in the body frame
    
    #Eulers Rotational Equation for a body-fixed 3 vector ω
    #Jα + ω × (J ω) = T_ext + u
    f = τ_bf - cross(ω_bf(s), Sy.J_bf .* ω_bf(s))

    #Eulers Rotational Equation for a body-fixed 3 vector ω
    ḣ_bf = Sy.u_ḣ_bf * ḣ                               #Torque in the inertial frame
    β = -(ḣ_bf + cross(ω_bf(s), h_bf(s)))

    #=q̇ = 1/2qΩ =#
    q!(ṡ, q(s) * Quat(ω_bf(s)..., 0) / 2)

    #J-1Jωdot = wdot
    ω_bf!(ṡ, (f + β) ./ Sy.J_bf)   

    h_bf!(ṡ, ḣ_bf)
end        

#Control that forces the leypanov time derivative to negative semi-definite
function control_step!(s)
    function v(s)
        ẑp_bf = SVec(0, 0, 1)
        ẑp_sp = normalize(q(s) * ẑp_bf)
        uω_bf = normalize(ω_bf(s))

        #ẑ_z_s -> α
        #1 = ẑ_x_s^2 + ẑ_y_s^2 + ẑ_z_s^2
        #sqrt(1 - ẑ_x_s^2 + ẑ_y_s^2) = ẑ_z_s -> α
        #(sqrt(1 - (ẑ_x_s^2 + ẑ_y_s^2)) - α)^2
        min_attitude_goal = (sqrt(1 - (ẑp_sp[1]^2 + ẑp_sp[2]^2)) - Sy.α)^2
        min_rotation_goal = uω_bf[1]^2 + uω_bf[2]^2     #Only rotate about z axis

        (min_attitude_goal + min_rotation_goal)/ 2
    end 

    v_0 = v(s)

    #v̇ = Δv ⋅ ẋ(ḣ)

    #Doesnt contain ḣ
    #Gradient
    Δv = zero(MVector{7, Float32})   
    for i in 1:7        #State Array
        s[i] += Eps
        Δv[i] = (v(s) - v_0) / Eps
        s[i] -= Eps
    end

    #Not constant in ḣ
    ẋ_0 = zero(EmptyS)
    ẋ!(ẋ_0, s, 0)                   
    v̇_0 = dot(Δv, ẋ_0[1:7])

    #v̇ = dot(Δv, ẋ) = αḣ + γ = (dẋdḣ)ḣ + v̇(0)
    #ḣ = -β(v̇(0) / dẋdḣ)    β >= 1
    #dv̇dḣ = ddot(Δv, ẋ)dḣ = dot(Δv, dẋdḣ)
    dẋdḣ = zero(EmptyS)
    ẋ!(dẋdḣ, s, Eps)
    dẋdḣ -= ẋ_0
    dẋdḣ /= Eps
    
    dv̇dḣ = dot(Δv, dẋdḣ[1:7])

    ḣ = -Sy.β * v̇_0 / dv̇dḣ
    
    ḣ * 1E-3
end

J_cylinder_about_vertical(m, r, h) = @SVector [
    m*(3*(r^2) + h^2)/12  
    m*(3*(r^2) + h^2)/12 
    m*(3*(r^2))/2
]

const www = Ref(0)

function initialize_system()
    ODE.ẋ! = function f(t, s, ṡ)
        ḣ = if www[] > 25
            0 #control_step!(s)
        else 
            www[] += 1
            0
        end
        ẋ!(ṡ, s, ḣ)    
    end
    ODE.u = Sy.s 
end

#Update in realtime
function model_step!(dt)
    rk4_step!(ODE, dt/2)
    rk4_step!(ODE, dt/2)
    q!(Sy.s, normalize(q(Sy.s)))            #Minimize numerical drift
    ODE.i % 1000 == 0 && println(ODE.i)
end