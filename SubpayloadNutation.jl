using LinearAlgebra
using GLMakie
using GLMakie: rotate!, translate!
using GeometryBasics
using DifferentialEquations
using StaticArrays

include("Control.jl")

J_cylinder_about_vertical(m, r, h) = @SMatrix [
    m * (3 * (r^2) + h^2)/12      0       0
    0   m * (3 * (r^2) + h^2)/12          0
    0   0                                m * (3 * (r^2))/2
]

const SVec = SVector{3, Float32}
const MVec = MVector{3, Float32}

mutable struct RotationalRigidBody
    # Constants
    mass::Float64
    J_l::Mat3f
    J_i_l::Mat3f

    # State Vars
    q::Quaternionf
    ω_g::MVec
    h_g::MVec           #Reaction Angular Momentum

    #Computed
    τ_g::MVec
    h_dot_g::MVec      #Reaction Torque

    t::Float64

    function RotationalRigidBody(m, J_l, q_0, ω_0)
        return new(m, J_l, inv(J_l), q_0, ω_0, zero(MVec), zero(MVec), zero(MVec), 0.0)
    end
end



function model()
    fig = Figure()
    axis3 = Axis3(fig[1, 1])
    hidedecorations!(axis3)

    g = [0, 0, -9.8]

    #Define the frame of the payload to be at the com of the cylinder
    r_o = 10E-2
    r_i = 0E-2
    m = .25
    h = 10E-2
    J_l = J_cylinder_about_vertical(m, r_o, h) - J_cylinder_about_vertical(m, r_i, h)
    θ_l = SVec(0, 0, 1)
    ω_l_0 = 12 * θ_l
    pivot_to_com_body_l = h/2 * θ_l
    q_0 = qrotation(SVec(1, 0, 0), 0.02)

    ω_g_0 = q_0 * ω_l_0    

    payload_shape = Cylinder3{Float64}(Point3f(0, 0, 0), Point3f(0, 0, h), r_o)
    S = mesh!(axis3, payload_shape, color=(:green, .2), transparency=true)
    SF = wireframe!(axis3, payload_shape, color=:black)

    rb = RotationalRigidBody(m, J_l, q_0, ω_g_0)


    S1 = mesh!(axis3, Cylinder3{Float64}(Point3f(0, 0, h_x_from_com), Point3f(width_reaction, 0, h_x_from_com), r_reaction_o), color=(:red, .3), transparency=true)

    autolimits!(axis3)

    """
    Jα + ω × (J ω) = T_ext + u1 + u2
    J1α1 + ω × (J1 ω1) = -u1
    J2α2 + ω × (J2 ω2) = -u2
    """

    function newton_step(f, x)
        f_x = f(x)
        f_dx = (f(x + eps()) - f_x) / eps() 
        x - f_x / (f_dx)
    end

    function f(u, p, t)
        compute_state(rb, u, t)
        du = similar(u)
        du[:] .= 0

        #Calculate torques
            rb.τ_g .= 0
            rb.h_dot_g .= 0

            #Calculate Gravity Torque
            pivot_to_com_g = rb.q * pivot_to_com_body_l
            F_g = g * m
            τ_gravity_g = cross(pivot_to_com_g, F_g)
            rb.τ_g += τ_gravity_g

            τ_ext_g = τ_gravity_g

            rb.h_dot_g .= -(τ_ext_g - cross(rb.ω_g, rb.J_l * rb.ω_g + rb.h_g))
            rb.τ_g += rb.h_dot_g
        #End Torques

        compute_derivative_state(rb, du)
   
        return du
    end

    v = zeros(10)
    set_state(rb, v)
    prob = ODEProblem(f, v, (0.0, 10.0); saveat=.025)
    sol = solve(prob, DP5())

    record(fig, "simulation.mp4", eachindex(sol.u); framerate=30) do i
        t = .1 * i
        compute_state(rb, sol.u[i], t)
       
        rotate!(SF, rb.q)
        rotate!(S, rb.q)

        rotate!(S1, rb.q)
    end

end

model()