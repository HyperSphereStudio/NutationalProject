"""
m_reaction = .1             #kg
r_reaction_o = 2E-2         #m
width_reaction = 5E-3       #m
h_x_from_com  = 6E-2        #m
   
J_rx_l = 2 * J_cylinder_about_vertical(m_reaction, r_reaction_o, width_reaction)  
θ_1_l = SVec(1, 0, 0)
q_1 = qrotation(SVec(0, 1, 0), pi/2)

"""

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

mutable struct SystemState
    #Constants
    mass::Float32                       #Mass of the system
    pivot_to_com_bf::SVec               #Pivot point to center of mass fixed frame
    J_bf::SVec                          #Diagonal Matrix Moment of Inertia of Body as vector in body fixed frame
    Jr_bf::SVec                         #Diagonal Matrix Moment of Inertia of Reaction as vector in body fixed frame
    u_τ_bf::SVec                        #Unit direction of control torque in the body frame

    #Interpolated / Measured

    #x_sp = q * x_bf -> 8.1 The Euler equations for rigid body dynamics (2)
    q::Quaternionf                      #Intertial to Fixed Frame.      Derived from accelerometer
    ω_bf::Float32                       #Measuered by Gyroscope.        Should be relatively constant
    h_bf::SVec                          #Angular momentum of reaction in body fixed frame. Can measure?  
                            
    h_dot_bf::MVec                      #
    ω_dot_bf::MVec                      #Angular Acceleration in the Body Fixed Frame

    α::Float32                          #Desired Angle (cos(θ))

    array::Array{Float32}               #Data array to integrate over

    SystemState() = new()
end

function expand_state_from_array(s::SystemState)
    v = s.array
    ptr = 1
    s.q, ptr = expand!(s.q, v, ptr)
    s.ω_bf, ptr = expand!(s.ω_bf, v, ptr)
    s.h_bf, ptr = expand!(s.h_bf, v, ptr)
    s.h_dot_bf .= 0
    s.ω_dot_bf .= 0
end 

function apply_state_to_array(s::SystemState)
    v = s.array
    ptr = 1
    ptr = flatten!(s.q, v, ptr)
    ptr = flatten!(s.ω_bf, v, ptr)
    ptr = flatten!(s.h_bf, v, ptr)
end

#Eulers Rotational Equation
#Differential Equation governing angular velocity in fixed frame

"Jα + ω × (J ω) = T_ext + u"
xdot(s) = (f(s) + β(s)) ./ s.J_bf        #J-1Jωdot = wdot


#System Dynamics
#Eulers Rotational Equation for a body-fixed 3 vector ω
"ω × (J ω) = T_ext"
function f(s::SystemState)
    τ_bf = SVec(0, 0, 0)
    return τ_bf - cross(s.ω_bf, s.J_bf .* s.ω_bf)           
end

#Reaction Rotational Dynamics
#Eulers Rotational Equation for a body-fixed 3 vector ω
β(s::SystemState) = -(s.h_dot_bf + cross(s.ω_bf, s.h_bf))

#Control that forces the leypanov time derivative 0
#J1α1 + ω × (J1 ω1) = -u1
function u(s::SystemState)
    c = [1, 1, 1]       #Coefficient Vector Weight the 

    -(f(s) + β(s)) + c .* [s.ω_l[1], s.ω_l[2], -(1 - s.ω_l[3] / s.ω * α)]
end

"ḣ = (ĥ ⋅ u)ĥ = Proj_h(u)"
reaction_control(s::SystemState) = s.h_dot_bf = dot(s.u_τ_bf, u(s)) * u_τ_bf

function apply_derivative_state(s::SystemState)
    xdot = s.array

    #=dqdt = 1/2qΩ =#
    qdot = s.q * Quaternionf(s.ω_l..., 0.0)  / 2                  #q_dot = q * omega_l / 2

    ptr = 1
    ptr = flatten!(qdot, xdot, ptr)
    ptr = flatten!(s.ω_dot_bf, xdot, ptr)
    ptr = flatten!(s.h_dot_bf, xdot, ptr)
end

function state(s::SystemState)
    expand_state_from_array(s)

   # s.h_dot_l .= reaction_control(s)
    s.ω_dot_bf .= xdot(s)

    apply_derivative_state(s)
end

lerp(x, y, t) = t*x + (1-t)*y
Base.eachindex(q::Quaternion) = eachindex(q.data)
expand!(::Quaternionf, v::AbstractVector, ptr::Integer) = (Quaternionf(v[ptr], v[ptr+1], v[ptr+2], v[ptr+3]), ptr+4)
function flatten!(data, v::AbstractVector, ptr::Integer)
    for i in eachindex(data)
        v[ptr] = data[i]
        ptr += 1
    end
    ptr
end

function expand!(data, v::AbstractVector, ptr::Integer)
    for i in eachindex(data)
        data[i] = v[ptr]
        ptr += 1
    end
    (data, ptr)
end