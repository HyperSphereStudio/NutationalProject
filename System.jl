"""
m_reaction = .1             #kg
r_reaction_o = 2E-2         #m
width_reaction = 5E-3       #m
h_x_from_com  = 6E-2        #m
   
J_rx_l = 2 * J_cylinder_about_vertical(m_reaction, r_reaction_o, width_reaction)  
θ_1_l = SVec(1, 0, 0)
q_1 = qrotation(SVec(0, 1, 0), pi/2)

"""

mutable struct SystemState
    #Constants
    mass::Float64                       #Mass of the system
    pivot_to_com_l::SVec                #Pivot point to center of Mass Fixed Frame
    q_r_l::Quaternionf                  #q to reaction wheel frame
    J_l::SVec                           #Diagonal Matrix Moment of Inertia of Body as vector in fixed frame
    Jr_l::Svec                          #Diagonal Matrix Moment of Inertia of Reaction as vector in fixed frame

    #Measured
    direction_of_gravity_l::SVec        #Accelerometer.    Will be oscilliating along the pivot. Used to derive q

    #Interpolated / Measured
    q::Quaternionf                      #Intertial to Fixed Frame
    ω::Float32                          #Gyroscope.        Should be relatively constant

    #Calculated
    ω_l::MVec                           #Angular Velocity Intertial Frame
    h_l::MVec                           #Reaction Angular Momentum Intertial Frame
    pivot_to_com_g::SVec                #Pivot point to center of Mass Intertial Frame

    τ_g::MVec                           #External Torques in the Intertial Frame
    h_dot_g::MVec                       #Reaction Torque in the Intertial Frame
end

function compute_and_expand_state_from_array(s::SystemState, v, t)
    ptr = 1
    rb.q, ptr = expand!(rb.q, v, ptr)
    rb.ω_g, ptr = expand!(rb.ω_g, v, ptr)
    rb.h_g, ptr = expand!(rb.h_g, v, ptr)
end 

function apply_state_to_array(rb::RotationalRigidBody, v)
    ptr = 1
    ptr = flatten!(rb.q, v, ptr)
    ptr = flatten!(rb.ω_g, v, ptr)
    ptr = flatten!(rb.h_g, v, ptr)
end

function f(s::SystemState)
    #Eulers Rotational Equation from
    (rb.τ_g - cross(rb.ω_l, rb.J_l .* rb.ω_l)
end

function compute_derivative_state(rb::RotationalRigidBody, vdot)
    qdot = Quaternionf(rb.ω_g..., 0.0) * rb.q  / 2                  #q_dot = omega_g * q / 2

    ptr = 1
    ptr = flatten!(qdot, vdot, ptr)

    #Jα + ω × (J ω) = T_ext + u1 + u2
    ωdot_g = rb.J_i_l * ) 
    ptr = flatten!(ωdot_g, vdot, ptr)
    ptr = flatten!(rb.h_dot_g, vdot, ptr)
end

#Rigid Body System
function f(q, x, J)

end

function state(x, h, hdot)


    (x, h)
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