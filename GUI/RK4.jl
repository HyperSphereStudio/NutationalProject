#=
   NAME: RK4.jl
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
=#

mutable struct RK4{T}
    u::Vector{T}
    u̇::Vector{T}
    k::Vector{Vector{T}}
    U::Vector{T}

    t::T
    i::Int
    ẋ!::Function

    RK4{T}(ẋ, n) where T = new{T}(zeros(Float32, n), zeros(Float32, n), [zeros(Float32, n) for i in 1:4], zeros(Float32, n), 0.0, 0, ẋ)
end

u!(rk4::RK4, u) = rk4.u .= u

function rk4_step!(rk4::RK4, dt)
    u = rk4.u
    ẋ! = rk4.ẋ!
    t = rk4.t
    k1, k2, k3, k4 = rk4.k
    
    U = rk4.U
    U .= u
    
    ẋ!(t, U, k1)

    map!(i -> u[i] + dt * k1[i] / 2, U, eachindex(U))  
    ẋ!(t + dt/2, U, k2)

    map!(i -> u[i] + dt * k2[i] / 2, U, eachindex(U))  
    ẋ!(t + dt/2, U, k3)

    map!(i -> u[i] + dt * k3[i], U, eachindex(U))
    ẋ!(t + dt, U, k4)
 
    rk4.t += dt
    rk4.i += 1
    map!(i -> u[i] + dt/6 * (k1[i] + k2[i] + k3[i] + k4[i]), rk4.u, eachindex(U))
  
    return u
end