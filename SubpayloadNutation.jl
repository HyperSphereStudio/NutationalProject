using LinearAlgebra
using GLMakie
using GLMakie: rotate!, translate!
using GeometryBasics
using DifferentialEquations
using StaticArrays

J_cylinder_about_vertical(m, r, h) = @SVector [
    m*(3*(r^2) + h^2)/12  
    m*(3*(r^2) + h^2)/12 
    m*(3*(r^2))/2
]

const SVec = SVector{3, Float32}
const MVec = MVector{3, Float32}

include("System.jl")

function model()
    fig = Figure()
    axis3 = Axis3(fig[1, 1])
    hidedecorations!(axis3)

    #Define the frame of the payload to be at the com of the cylinder
    r_o = 10E-2
    r_i = 0E-2
    h = 10E-2

    s = SystemState()
    s.mass = .25
    s.J_bf = J_cylinder_about_vertical(s.mass, r_o, h) - J_cylinder_about_vertical(s.mass, r_i, h)
    s.ω_bf = 12 * SVec(0, 0, 1)
    s.q = qrotation(SVec(1, 0, 0), 0.0)
    s.pivot_to_com_bf = h/2 * s.uw_l 
    s.uh_dot_l = SVec(1, 0, 0) 
    s.α = cos(0)
    s.h_dot = 0
    s.array = zeros(Float32, 10)

    #Payload Plotting
    payload_shape = Cylinder3{Float64}(Point3f(0, 0, 0), Point3f(0, 0, h), r_o)
    S = mesh!(axis3, payload_shape, color=(:green, .2), transparency=true)
    SF = wireframe!(axis3, payload_shape, color=:black)
    
    #Reaction Plotting
  #  S1 = mesh!(axis3, Cylinder3{Float64}(Point3f(0, 0, h_x_from_com), Point3f(width_reaction, 0, h_x_from_com), r_reaction_o), color=(:red, .3), transparency=true)

    autolimits!(axis3)

    apply_state_to_array(s)
    prob = ODEProblem(f, s.array, (0.0, 10.0); saveat=.025)
    sol = solve(prob, DP5())

    record(fig, "simulation.mp4", eachindex(sol.u); framerate=30) do i
        s.array .= sol.u[i]
        expand_state_from_array(s)
       
        rotate!(SF, rb.q)
        rotate!(S, rb.q)

    #    rotate!(S1, rb.q)
    end

end

model()