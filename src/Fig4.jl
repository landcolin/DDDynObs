# Two-obstacle scenario shown in paper Fig. 4.
include("FigureReproduction.jl")

const FIG4_CONFIG = (
    number=4, scenario=2, scenario_name="circular and vertical",
    steps=[1, 20, 30, 40, 50, 60],
    xlims=(-5, 45), ylims=(-12, 22), xticks=0:10:40, yticks=-10:10:20,
    annotation=(-3, -10.5), size=(1800, 840),
    extra_notes=[
        "Two radius-2 m obstacles start at [18, -10] (circle) and [30, 18] (vertical). The vertical obstacle uses velocity [0, -3.5] plus the repository noise implementation, which reinitializes seed 42 each step.",
        "Each obstacle has its own GP pair and RNN. Only obstacles within the 20 m detection range contribute constraints at that step.",
        "Uses squared reference-position error minus 10 times the sum of safety margins, with margins bounded to [2, 10] m. The weight 10 matches Fig. 2/3; the original multi-obstacle script uses weight 1, which failed to maintain solvability in this run. The paper does not specify this weight numerically.",
        "The multi-obstacle MPC yaw update is corrected to include speed, as in the previously corrected single-obstacle MPC. Every solve must return a solved and feasible status before controls are applied.",
        "Multi-obstacle dual variables start from normalized geometric separating directions instead of the degenerate all-zero normals. This changes the numerical initial guess, not the objective or safety constraints.",
        "If the warm-started NLP fails, the same problem is retried with bounded left-turn, right-turn, or braking rollout initial guesses. Only solved, feasible results are applied. solver_attempts records these numerical restarts.",
        "trajectory.csv records minimum clearance to either obstacle, plus individual clearances and the second obstacle position. Checks use sampled robot/obstacle configurations, not continuous-time swept geometry.",
        "Snapshots follow the six published labels: 1, 20, 30, 40, 50, 60.",
    ],
)

function simulate_fig4(config)
    output = figure_output(config)
    mkpath(output)
    Random.seed!(1234)
    BLAS.set_num_threads(1)
    T, H₊, ns = 0.1, 8, 3
    bounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    A = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    robo = robot(T, H, H₊, 20.0, 1.9, bounds, [0.0, 2.0, 0.0, 6.0], A, [2.5, 2.5, 1.0, 1.0])
    obstacles = [obstacle(2.0, [18.0, -10.0], ns), obstacle(2.0, [30.0, 18.0], ns)]
    gps = [GPBase[GPE(mean=MeanConst(1.0), kernel=SEArd([1.0], 1.0), logNoise=-2.0) for _ in 1:2] for _ in obstacles]
    networks = [Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh)) for _ in obstacles]
    φ = fill(rectangle_quantile(0.95), H)
    snapshots, rows, diagnostics = Dict{Int, Any}(), Any[], Any[]
    open(joinpath(output, "trajectory.csv"), "w") do csv
        println(csv, "step,time,x,y,yaw,speed,obstacle_x,obstacle_y,steering_tangent,acceleration,gp_active,clearance_before,clearance_after,solver_status,solver_feasible,obstacle2_x,obstacle2_y,obstacle1_clearance_before,obstacle1_clearance_after,obstacle2_clearance_before,obstacle2_clearance_after,active_obstacles,solver_attempts")
        for k in 1:maximum(config.steps)
            started = time()
            run_obs!(obstacles[1], k - 1, T, 2)
            run_obs_topdown!(obstacles[2], k - 1, T)
            ref = [[6(k + i - 1) * T, 2.0] for i in 1:H₊]
            active = [i for i in eachindex(obstacles) if norm(robo.pose[1:2] - obstacles[i].posn) < robo.R]
            means, variances = Any[nothing for _ in obstacles], Any[nothing for _ in obstacles]
            for i in active
                means[i], variances[i] = predict_obstacle!(robo, obstacles[i], gps[i], networks[i], k)
                @assert all(isfinite, means[i]) && all(isfinite, variances[i]) && minimum(variances[i]) >= 0
            end
            serialize(joinpath(output, "last_problem.jls"), (robot=robo, obstacles=obstacles[active], ref=ref,
                      means=Matrix{Float64}[means[i] for i in active], variances=Matrix{Float64}[variances[i] for i in active], quantiles=φ, step=k))
            solution = if isempty(active)
                nMPC_free(robo, ref; diagnostics)
            else
                nMPC_avoid_multi(robo, obstacles[active], ref, Matrix{Float64}[means[i] for i in active],
                                Matrix{Float64}[variances[i] for i in active], φ; diagnostics, safety_weight=10.0)
            end
            x, y, θ, v, steering, acceleration = solution
            @assert all(values -> all(isfinite, values), solution)
            robo.predPose = [x'; y'; θ'; v']
            before = [rectangle_circle_clearance(robo, obs) for obs in obstacles]
            if k in config.steps
                snapshots[k] = (robot=deepcopy(robo), obstacles=deepcopy(obstacles), x=copy(x), y=copy(y),
                                means=deepcopy(means), variances=deepcopy(variances), quantiles=φ)
            end
            run!(robo, [steering[1], acceleration[1]])
            after = [rectangle_circle_clearance(robo, obs) for obs in obstacles]
            row = (k, k * T, robo.pose..., obstacles[1].posn..., steering[1], acceleration[1],
                   !isempty(active), minimum(before), minimum(after), diagnostics[end].status, diagnostics[end].feasible,
                   obstacles[2].posn..., before[1], after[1], before[2], after[2], join(active, ';'), get(diagnostics[end], :attempts, 1))
            push!(rows, row)
            println(csv, join(row, ','))
            flush(csv)
            serialize(joinpath(output, "run_state.jls"), (snapshots=snapshots, rows=rows, diagnostics=diagnostics))
            println("Step $k/60 | active=$active | solver=$(diagnostics[end].status) | clearance=$(round(minimum(after); digits=3)) m | $(round(time()-started; digits=2)) s")
            flush(stdout)
        end
    end
    return (snapshots=snapshots, rows=rows, diagnostics=diagnostics)
end

result = if "--plot-only" in ARGS
    deserialize(joinpath(figure_output(FIG4_CONFIG), "run_state.jls"))
else
    simulate_fig4(FIG4_CONFIG)
end
@assert length(result.rows) == 60 && all(k -> haskey(result.snapshots, k), FIG4_CONFIG.steps)
@assert all(d -> d.feasible, result.diagnostics)
render_figure(result, FIG4_CONFIG)
