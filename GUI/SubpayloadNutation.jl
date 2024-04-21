#=
   NAME: SubpayloadNutation.jl
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
=#

module PayloadNutation

using StaticArrays
using JuliaSAILGUI
using Dates
using GLMakie
using GLMakie: rotate!, translate!
using Base.Threads
using FileIO

const SVec = SVector{3, Float32}
const MVec = MVector{3, Float32}
const Quat = Quaternionf

function log_error(e)
    open("error.txt"; create=true, write=true, append=true) do f
        write(f, "=======ERROR====== $(now())\n")
        showerror(f, e, catch_backtrace())
        showerror(stdout, e, catch_backtrace())
        write(f, "========\n")
    end
end

include("RK4.jl")
include("Display.jl")
include("System.jl")
include("Embedded.jl")


function CreateInitialConditions()
    #Define the frame of the payload to be at the com of the cylinder
    r_o = 10E-2
    r_i = 0E-2
    h = 10E-2
    
    s0 = zero(MVector{10, Float32})
    Sy.mass = .25
    Sy.J_bf = J_cylinder_about_vertical(Sy.mass, r_o, h) - J_cylinder_about_vertical(Sy.mass, r_i, h)
    Sy.pivot_to_com_bf = SVec(0, 0, h/2)
    Sy.u_ḣ_bf = SVec(1, 0, 0) 
    Sy.β = 1

    q!(s0, conj(qrotation(SVec(0.0, 1.0, 0.0), Float32(0.1))))
    h_bf!(s0, SVec(0, 0, 0))
    ω_bf!(s0, SVec(0, 0, 8))

    Sy.s = s0

    payload_shape = Cylinder3{Float64}(Point3f(0, 0, 0), Point3f(0, 0, h), r_o)
    
    push!(Shapes, mesh!(Viz3DAxis, load(assetpath("brain.stl")))) #, color=(:green, .2), transparency=true))
   # push!(Shapes, wireframe!(Viz3DAxis, payload_shape, color=:black))
    # push!(Shapes, mesh!(Viz3DAxis, Cylinder3{Float64}(Point3f(0, 0, h_x_from_com), Point3f(width_reaction, 0, h_x_from_com), r_reaction_o), color=(:red, .3), transparency=true))
    autolimits!(Viz3DAxis)   
end

CreateInitialConditions()
initialize_system()
initialize_display()

end

using .PayloadNutation