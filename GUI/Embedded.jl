#=
   NAME: Embedded.jl
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
=#

const BaudRate = 115200
const Comp = SimpleConnection(MicroControllerPort(:Computer, BaudRate, nothing))

comp_println(x...) = println("[Comp]:", x...)

#Packet Types
const ComputerPrint::UInt8 = 1
const AttitudePacket::UInt8 = 2

function connection_step!()
    if isopen(Comp)
        readport(Comp) do io
            id = read(io, UInt8)
            if id == ComputerPrint
                print(read(io, String))
            end
        end
    end 
end