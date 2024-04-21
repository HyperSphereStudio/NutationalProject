#=
   NAME: Display.jl
   AUTHOR: Johnathan Bizzano
   DATE: 4/20/2023
=#

using Mousetrap
using MousetrapMakie
using Mousetrap: Label
using GeometryBasics

const Shapes = []
const App = Application("com.hypersphere.nutation")
const Viz3DFigure = Figure()
const Viz3DAxis = Axis3(Viz3DFigure[1, 1])

function CreatePortDropDown(onPortInput)
    portList = DropDown()
    registeredPorts = Dict{String, DropDownItemID}()
    focus = FocusEventController()
    gained_focus_initial = Ref(true)

    connect_signal_focus_gained!(focus) do self
        pl = get_port_list()
        gained_focus_initial[] = true

        #Remove ports that are gone
        regis2pldiff = setdiff(keys(registeredPorts), pl)
        foreach(r -> (remove!(portList, registeredPorts[r]); delete!(registeredPorts, r)), regis2pldiff)
        for p in setdiff(pl, keys(registeredPorts))
            registeredPorts[p] = push_back!(portList, p) do _
                println("Set Port:$p")
                gained_focus_initial[] || onPortInput(p)
                gained_focus_initial[] = false
                return nothing
            end
        end  
    end
    add_controller!(portList, focus)

    vbox(Label("COM Port"), portList)
end

function controlSlider()
    nutationAngle = Observable(0.0)
    scale = Scale(0, 5, .01, ORIENTATION_HORIZONTAL)
    scaleLabel = Label("")

    on(nutationAngle) do s
        scaleLabel[] = "Nutation Angle: $(round(s, digits=2))"
        Sy.α = cosd(s)
        println("Adjusted Control Angle To $(round(s, digits=2))")
    end
    Observables.connect!(scale, nutationAngle)
    emit_signal_value_changed(scale)

    vbox(scaleLabel, scale)
end

function build_main_plot()
    canvas = mkfigcanvas(Viz3DFigure)
    hidedecorations!(Viz3DAxis)
    set_size_request!(canvas, Vector2f(400, 400))

    canvas
end

function mkfigcanvas(fig)
    canvas = GLMakieArea()
    
    connect_signal_realize!(canvas) do self
        screen = create_glmakie_screen(canvas)
        display(screen, fig)     
        return nothing
    end

    canvas
end

function initialize_display()
    connect_signal_activate!(App) do _
        try
            win = Window(App) 
            set_title!(win, "Nutational Control")
            grid = Grid()
            set_child!(win, grid)

            insert_at!(grid, CreatePortDropDown(p -> (setport(Comp, p); println("Connected to port: $p"))), 1, 1)
            insert_at!(grid, controlSlider(), 2, 1)
            insert_at!(grid, build_main_plot(), 1, 2, 2, 2)

            lt = Ref(typemax(Int64))
            set_tick_callback!(win) do clock
                try
                   n =  time_ns()
                   if n > lt[]
                        dt = (n - lt[]) * 1E-9
                        model_step!(dt)
                        qV = q(Sy.s)
                        for s in Shapes
                            rotate!(s, qV)
                        end
                    end
                    lt[] = n
                catch e
                    println("Error While Running Run Task!")
                    log_error(e)
                end    
                return TICK_CALLBACK_RESULT_CONTINUE
            end

            present!(win)
        catch ex
            log_error(ex)
            quit!(App)
            isopen(Comp) && close(Comp)
        end
        return nothing
    end
    
    run!(App)
end        

