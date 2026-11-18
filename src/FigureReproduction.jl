# Shared figure simulation and export workflow.
include("environment.jl")
using Serialization

H = 8
figure_output(config) = normpath(joinpath(@__DIR__, "..", "output", "fig$(config.number)"))

function rectangle_circle_clearance(robo, obs)
    θ = robo.pose[3]
    rotation = [cos(θ) -sin(θ); sin(θ) cos(θ)]
    local_position = rotation' * (obs.posn - robo.pose[1:2])
    return norm(max.(abs.(local_position) - robo.b[[1, 3]], 0.0)) - obs.r
end

function simulate_figure(config)
    output = figure_output(config)
    Random.seed!(1234)
    BLAS.set_num_threads(1)
    T, H₊, ns = 0.1, 8, 3
    bounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    A = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    robo = robot(T, H, H₊, 20.0, 1.9, bounds, [0.0, 2.0, 0.0, 6.0], A, [2.5, 2.5, 1.0, 1.0])
    obs = obstacle(2.0, copy(config.obstacle_initial), ns)
    gps = GPBase[GPE(mean=MeanConst(1.0), kernel=SEArd([1.0], 1.0), logNoise=-2.0) for _ in 1:2]
    network = Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh))
    φ = fill(rectangle_quantile(0.95), H)
    snapshots = Dict{Int, Any}()
    rows = Any[]
    diagnostics = Any[]
    mkpath(output)
    open(joinpath(output, "trajectory.csv"), "w") do csv
        println(csv, "step,time,x,y,yaw,speed,obstacle_x,obstacle_y,steering_tangent,acceleration,gp_active,clearance_before,clearance_after,solver_status,solver_feasible")
        for k in 1:maximum(config.steps)
            started = time()
            run_obs!(obs, k - 1, T, config.scenario)
            ref = [[6(k + i - 1) * T, 2.0] for i in 1:H₊]
            active = norm(robo.pose[1:2] - obs.posn) < robo.R
            pmean, pvar = nothing, nothing
            if active
                Detection!(robo, obs, gps, network)
                δmean, δvar = zeros(2, ns * H), zeros(2, ns * H)
                times = k * T .+ T .* collect(1:(H * ns)) ./ ns
                for axis in 1:2
                    δmean[axis, :], δvar[axis, :] = predictGP(network, gps[axis], times, axis)
                end
                δmean[:, 1] += obs.posn
                for i in 1:(H * ns - 1)
                    δmean[:, i + 1] += δmean[:, i]
                    δvar[:, i + 1] += δvar[:, i] + [exp(2gp.logNoise.value) for gp in gps]
                end
                pmean = δmean[:, ns:ns:end]
                pvar = δvar[:, ns:ns:end]
                @assert all(isfinite, pmean) && all(isfinite, pvar) && minimum(pvar) >= 0
                solution = nMPC_avoid(robo, obs, ref, pmean, pvar, φ; diagnostics)
            else
                solution = nMPC_free(robo, ref; diagnostics)
            end
            x, y, θ, v, steering, acceleration = solution
            @assert all(values -> all(isfinite, values), solution)
            robo.predPose = [x'; y'; θ'; v']
            clearance_before = rectangle_circle_clearance(robo, obs)
            if k in config.steps
                snapshots[k] = (robot=deepcopy(robo), obstacle=deepcopy(obs), x=copy(x), y=copy(y),
                                mean=pmean, variance=pvar, quantiles=φ)
            end
            run!(robo, [steering[1], acceleration[1]])
            clearance_after = rectangle_circle_clearance(robo, obs)
            row = (k, k * T, robo.pose..., obs.posn..., steering[1], acceleration[1],
                   active, clearance_before, clearance_after, diagnostics[end].status, diagnostics[end].feasible)
            push!(rows, row)
            println(csv, join(row, ','))
            flush(csv)
            # Checkpoint also permits plotting without re-running neural network training.
            serialize(joinpath(output, "run_state.jls"), (snapshots=snapshots, rows=rows, diagnostics=diagnostics))
            println("Step $k/$(maximum(config.steps)) | GP=$active | solver=$(diagnostics[end].status) | clearance=$(round(clearance_after; digits=3)) m | $(round(time()-started; digits=2)) s")
            flush(stdout)
        end
    end
    return (snapshots=snapshots, rows=rows, diagnostics=diagnostics)
end

function render_figure(result, config)
    output = figure_output(config)
    panels = Plots.Plot[]
    for k in config.steps
        snapshot = result.snapshots[k]
        fig = plot_robot(snapshot.robot, snapshot.x, snapshot.y, config.scenario, k; annotate_step=false)
        if hasproperty(snapshot, :obstacles)
            for i in eachindex(snapshot.obstacles)
                if snapshot.means[i] === nothing
                    plot_obs(snapshot.obstacles[i])
                else
                    plot_obs(snapshot.obstacles[i], snapshot.means[i], snapshot.variances[i], snapshot.quantiles)
                end
            end
        elseif snapshot.mean === nothing
            plot_obs(snapshot.obstacle)
        else
            plot_obs(snapshot.obstacle, snapshot.mean, snapshot.variance, snapshot.quantiles)
        end
        plot!(fig; xlims=config.xlims, ylims=config.ylims,
              xticks=(collect(config.xticks), string.(config.xticks)),
              yticks=(collect(config.yticks), string.(config.yticks)), tickfontsize=10,
              fontfamily="Times", grid=false, legend=false, aspect_ratio=:equal,
              framestyle=:axes, background_color=:white, foreground_color_axis=:gray,
              margin=4Plots.mm)
        annotate!(fig, config.annotation..., text("Step $k", 12, :red, :left, "Times"))
        push!(panels, fig)
        savefig(fig, joinpath(output, "step_$(lpad(k, 2, '0')).pdf"))
    end
    combined = plot(panels...; layout=(2, 3), size=get(config, :size, (1800, 660)), dpi=200)
    savefig(combined, joinpath(output, "fig$(config.number)_reproduced.pdf"))
    savefig(combined, joinpath(output, "fig$(config.number)_reproduced.png"))
    min_clearance = minimum(min(row[12], row[13]) for row in result.rows)
    open(joinpath(output, "reproduction_notes.md"), "w") do io
        println(io, "# Fig. $(config.number) computational reproduction\n")
        println(io, "Generated by src/Fig$(config.number).jl using Julia $(VERSION), the committed dependency manifest, seed 1234, and one BLAS thread. All $(length(result.rows)) optimizer calls were checked for a solved, feasible status.\n")
        println(io, "Snapshots: $(join(config.steps, ", ")). Minimum sampled rectangle-circle clearance, before/after each robot update: $(round(min_clearance; digits=6)) m. This is a sampled check, not a continuous collision certificate.\n")
        final_row = last(result.rows)
        println(io, "Final post-update robot position: ($(round(final_row[3]; digits=6)), $(round(final_row[4]; digits=6))) m. Distance from the y=2 reference line: $(round(abs(final_row[4]-2); digits=6)) m. Panels show the state before the robot update at each labeled step.\n")
        println(io, "## Scope and differences from the paper\n")
        println(io, "- This is a fresh simulation of the $(config.scenario_name)-obstacle scenario, not an extraction of the published image or a claim of numerical identity.")
        println(io, "- Uses the paper's H=8 and robot half-length=2.5 m. The original single-obstacle entry point uses H=6 and half-length=2.3 m.")
        println(io, "- Retains repository speed bounds [-10, 20] m/s and initial speed 6 m/s. The paper text lists [-1, 2] m/s, which conflicts with its 6t reference and the repository initial speed.")
        println(io, "- MPC yaw updates include speed, matching robot_ode! and paper equation (20). The velocity warm start uses row 4 rather than the yaw row. The simulator integrates its ODE; the MPC uses Euler constraints.")
        println(io, "- Retains run_obs!'s noise implementation: it recreates seed 1234 each step and scales noise by the sub-sample count. The paper describes i.i.d. velocity noise with standard deviation 1.")
        println(io, "- Uses the locked Flux 0.16 implementation and existing trainNN/predictLSTM functions; historical Flux recurrent-state behavior and the original paper's network weights are unavailable.")
        for note in config.extra_notes
            println(io, "- ", note)
        end
        println(io, "\n## Re-run\n\n```powershell\n.\\scripts\\julia.ps1 fig$(config.number)\n```\n\nRe-render a completed checkpoint without training again:\n\n```bash\njulia --project=src src/Fig$(config.number).jl --plot-only\n```")
    end
    println("Saved Fig. $(config.number) reproduction to $output")
    println("Minimum sampled clearance: $min_clearance m")
end

function run_figure(config)
    result = if "--plot-only" in ARGS
        deserialize(joinpath(figure_output(config), "run_state.jls"))
    else
        simulate_figure(config)
    end
    @assert length(result.rows) == maximum(config.steps)
    @assert all(k -> haskey(result.snapshots, k), config.steps)
    @assert all(d -> d.feasible, result.diagnostics)
    render_figure(result, config)
end
