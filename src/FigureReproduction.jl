using Serialization

function robot_obstacle_clearance(robo::robot, obs::obstacle)
    θ = robo.pose[3]
    p = robo.pose[1:2]
    hx, hy = robo.b[1], robo.b[3]
    Δ = obs.posn - p
    local_x =  cos(θ) * Δ[1] + sin(θ) * Δ[2]
    local_y = -sin(θ) * Δ[1] + cos(θ) * Δ[2]
    dx = max(abs(local_x) - hx, 0.0)
    dy = max(abs(local_y) - hy, 0.0)
    return hypot(dx, dy) - obs.r
end

function predict_single_obstacle!(robo::robot, obs::obstacle, obsGP::Vector{GPBase}, mNN::Chain, k::Int64)
    Detection!(robo, obs, obsGP, mNN)
    dim = length(obs.posn)
    ns  = obs.ns
    H   = robo.H
    T   = robo.T

    δme = zeros(dim, ns * H)
    δva = zeros(dim, ns * H)
    for i in 1:dim
        δme[i, :], δva[i, :] = predictGP(mNN, obsGP[i], k * T .+ T * [x / ns for x in 1:H*ns], i)
    end

    δme[:, 1] += obs.posn
    for i in 1:H*ns-1
        δme[:, i + 1] += δme[:, i]
        δva[:, i + 1] += δva[:, i] + [exp(2obsGP[1].logNoise.value), exp(2obsGP[2].logNoise.value)]
    end

    Pmean = δme[:, [i * ns for i in 1:H]]
    Pvar  = δva[:, [i * ns for i in 1:H]]
    return Pmean, Pvar
end

function write_trajectory_csv(filepath::AbstractString, traj_rows::AbstractVector)
    mkpath(dirname(filepath))
    open(filepath, "w") do io
        println(io, "step,x,y,theta,v,tan_alpha,a,min_clearance_m,solver,solver_attempts,termination_status,primal_status,feasible,solve_time_s,wall_time_s")
        for r in traj_rows
            println(io, join([
                r.step, r.x, r.y, r.theta, r.v, r.tan_alpha, r.a, r.min_clearance_m,
                r.solver, r.solver_attempts, r.termination_status, r.primal_status,
                r.feasible, r.solve_time_s, r.wall_time_s
            ], ","))
        end
    end
    return filepath
end

function write_reproduction_notes(filepath::AbstractString, title::AbstractString, meta::NamedTuple)
    mkpath(dirname(filepath))
    open(filepath, "w") do io
        println(io, "# ", title)
        println(io)
        println(io, "- Steps completed: ", meta.steps_completed, " / ", meta.L)
        println(io, "- Horizon: H=", meta.H, ", H₊=", meta.H₊)
        println(io, "- Robot half-length b[1:2]: ", meta.b1, " m")
        if hasproperty(meta, :safe_weight)
            println(io, "- Safety margin weight: ", meta.safe_weight)
        end
        println(io, "- Network seed: ", meta.seed, ", BLAS threads: ", meta.blas_threads)
        println(io, "- Minimum sampled clearance: ", round(meta.min_clearance_m; digits=6), " m")
        println(io, "- Max solver attempts per step: ", meta.max_attempts)
        println(io, "- Step PDFs directory: `", meta.path_to_fig, "`")
        println(io, "- Combined outputs: `", meta.combined_pdf, "`, `", meta.combined_png, "`")
        println(io, "- Snapshot replot supported via `--plot-only` (redraws figures from `snapshot.jls`; not a simulation checkpoint).")
    end
    return filepath
end

function save_combined_grid(plots_vec::AbstractVector, pdf_path::AbstractString, png_path::AbstractString;
                            layout=(2, 3), size=(3000, 1050))
    mkpath(dirname(pdf_path))
    fig = plot(plots_vec..., layout = layout, size = size)
    savefig(fig, pdf_path)
    savefig(fig, png_path)
    return fig
end

function render_frame(frame, kind::Symbol; tick_fontsize::Union{Int,Nothing}=nothing, show_step::Bool=true, grid_margins::Bool=false)
    robo_snap = frame.robo
    if kind === :single
        fig = plot_robot(robo_snap, frame.pred_x, frame.pred_y, frame.type_of_obs, frame.step; show_step = show_step)
        o = frame.obs[1]
        if o.active
            plot_obs(o.obs, o.Pmean, o.Pvar, o.φ)
        else
            plot_obs(o.obs)
        end
    else
        fig = plot_robot_mobs(robo_snap, frame.pred_x, frame.pred_y, frame.step; show_step = show_step)
        for o in frame.obs
            if o.active
                plot_obs_multi!(o.obs, o.Pmean, o.Pvar, o.φ)
            else
                plot_obs_multi!(o.obs)
            end
        end
    end
    apply_publication_style!(fig; tick_fontsize = tick_fontsize, grid_margins = grid_margins)
    return fig
end

function replot_from_snapshot(path_to_fig::AbstractString, combined_prefix::AbstractString,
                              fig_steps::Vector{Int}, title::AbstractString;
                              tick_fontsize::Union{Int,Nothing}=nothing,
                              show_step::Bool=true,
                              grid_size::Tuple{Int,Int}=(3000, 1050))
    snapshot_path = joinpath(path_to_fig, "snapshot.jls")
    if !isfile(snapshot_path)
        error("Snapshot file not found for --plot-only: $snapshot_path")
    end
    snap = Serialization.deserialize(snapshot_path)
    pdf_out = combined_prefix * ".pdf"
    png_out = combined_prefix * ".png"

    selected_plots = Plots.Plot[]
    for frame in snap.step_frames
        fig = render_frame(frame, snap.kind; show_step = show_step)
        savefig(fig, joinpath(path_to_fig, "$(frame.step).pdf"))
        if frame.step in fig_steps
            fig_grid = render_frame(frame, snap.kind; tick_fontsize = tick_fontsize, show_step = show_step, grid_margins = true)
            push!(selected_plots, fig_grid)
        end
    end

    save_combined_grid(selected_plots, pdf_out, png_out; layout = (2, 3), size = grid_size)
    max_attempts = isempty(snap.solver_logs) ? 0 : maximum(r.attempts for r in snap.solver_logs)
    write_reproduction_notes(
        joinpath(path_to_fig, "README_repro.md"),
        title,
        merge(snap.meta, (steps_completed = length(snap.traj_rows), max_attempts = max_attempts,
                          path_to_fig = path_to_fig, combined_pdf = pdf_out, combined_png = png_out))
    )
    println("Replotted from snapshot: ", snapshot_path)
    println("Saved ", pdf_out, " and ", png_out)
end

function run_single_obstacle_reproduction(;
    TypeOfObs::Int64,
    obs_init::Vector{Float64},
    path_to_fig::AbstractString,
    combined_prefix::AbstractString,
    fig_steps::Vector{Int},
    title::AbstractString,
    L::Int64 = 60,
    H::Int64 = 6,
    H₊::Int64 = 8,
    T::Float64 = 0.1,
    ns::Int64 = 3,
    R::Float64 = 20.0,
    ℓ::Float64 = 1.9,
    str_ang::Float64 = 0.6,
    v_min::Float64 = -10.0,
    v_max::Float64 = 20.0,
    a_min::Float64 = -2.0,
    a_max::Float64 = 2.0,
    init::Vector{Float64} = [0.0, 2.0, 0.0, 6.0],
    A::Matrix{Float64} = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0],
    b::Vector{Float64} = [2.3, 2.3, 1.0, 1.0],
    prob_conf::Float64 = 0.95,
    seed::Int = 1234,
    blas_threads::Int = 1,
    plot_only::Bool = false,
    show_step::Bool = true,
    tick_fontsize::Union{Int,Nothing} = nothing,
    grid_size::Tuple{Int,Int} = (3000, 1000)
)
    LinearAlgebra.BLAS.set_num_threads(blas_threads)
    if plot_only
        return replot_from_snapshot(path_to_fig, combined_prefix, fig_steps, title;
                                    tick_fontsize=tick_fontsize, show_step=show_step, grid_size=grid_size)
    end

    mkpath(path_to_fig)
    pdf_out = combined_prefix * ".pdf"
    png_out = combined_prefix * ".png"
    rm(pdf_out; force = true)
    rm(png_out; force = true)

    Random.seed!(seed)
    pBounds = polyBound(str_ang, v_min, v_max, a_min, a_max)
    robo    = robot(T, H, H₊, R, ℓ, pBounds, copy(init), A, b)
    obs     = obstacle(2.0, copy(obs_init), ns)

    obsGP    = Vector{GPBase}(undef, 2)
    obsGP[1] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
    obsGP[2] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
    mNN      = Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh))

    selected_plots = Plots.Plot[]
    solver_logs    = NamedTuple[]
    traj_rows      = NamedTuple[]
    step_frames    = NamedTuple[]
    min_clearance  = Inf

    println("Now start the simulation (TypeOfObs = $TypeOfObs, seed = $seed, BLAS threads = $blas_threads)")
    try
        for k in 1:L
            println("Time instance $k")
            run_obs!(obs, k - 1, T, TypeOfObs)
            Ref = [[6(k + i - 1) * T, 2.] for i in 1:H₊]

            clearance = robot_obstacle_clearance(robo, obs)
            min_clearance = min(min_clearance, clearance)

            if norm(robo.pose[1:2] - obs.posn) < robo.R
                Pmean, Pvar = predict_single_obstacle!(robo, obs, obsGP, mNN, k)
                Prob = fill(prob_conf, H)
                φ    = [quantile(Normal(0., 1.), (1 + Prob[i]) / 2) for i in 1:H]

                x, y, θ, v, tα, a = nMPC_avoid(robo, obs, Ref, Pmean, Pvar, φ; step = k, solver_log = solver_logs)
                robo.predPose = [x'; y'; θ'; v']
                obs_entry = (active = true, obs = deepcopy(obs), Pmean = copy(Pmean), Pvar = copy(Pvar), φ = copy(φ))
            else
                x, y, θ, v, tα, a = nMPC_free(robo, Ref; step = k, solver_log = solver_logs)
                robo.predPose = [x'; y'; θ'; v']
                obs_entry = (active = false, obs = deepcopy(obs), Pmean = zeros(0, 0), Pvar = zeros(0, 0), φ = Float64[])
            end

            frame = (step = k, type_of_obs = TypeOfObs, robo = deepcopy(robo),
                     pred_x = collect(Float64, x), pred_y = collect(Float64, y), obs = [obs_entry])
            push!(step_frames, frame)

            fig = render_frame(frame, :single; show_step = show_step)
            savefig(fig, joinpath(path_to_fig, "$k.pdf"))
            if k in fig_steps
                fig_grid = render_frame(frame, :single; tick_fontsize = tick_fontsize, show_step = show_step, grid_margins = true)
                push!(selected_plots, fig_grid)
            end

            rec = solver_logs[end]
            push!(traj_rows, (
                step = k, x = robo.pose[1], y = robo.pose[2], theta = robo.pose[3], v = robo.pose[4],
                tan_alpha = tα[1], a = a[1], min_clearance_m = clearance,
                solver = rec.solver, solver_attempts = rec.attempts,
                termination_status = rec.termination_status, primal_status = rec.primal_status,
                feasible = rec.feasible, solve_time_s = rec.solve_time_s, wall_time_s = rec.wall_time_s
            ))
            run!(robo, [tα[1], a[1]])
        end
    finally
        write_solver_log_csv(joinpath(path_to_fig, "solver_log.csv"), solver_logs)
        !isempty(traj_rows) && write_trajectory_csv(joinpath(path_to_fig, "trajectory.csv"), traj_rows)
    end

    save_combined_grid(selected_plots, pdf_out, png_out; layout = (2, 3), size = grid_size)
    snapshot_path = joinpath(path_to_fig, "snapshot.jls")
    meta = (L = L, H = H, H₊ = H₊, b1 = b[1], seed = seed, blas_threads = blas_threads, min_clearance_m = min_clearance)
    Serialization.serialize(snapshot_path, (kind = :single, meta = meta, step_frames = step_frames, traj_rows = traj_rows, solver_logs = solver_logs))

    max_attempts = isempty(solver_logs) ? 0 : maximum(r.attempts for r in solver_logs)
    write_reproduction_notes(
        joinpath(path_to_fig, "README_repro.md"),
        title,
        (steps_completed = length(traj_rows), L = L, H = H, H₊ = H₊, b1 = b[1],
         seed = seed, blas_threads = blas_threads,
         min_clearance_m = min_clearance, max_attempts = max_attempts,
         path_to_fig = path_to_fig, combined_pdf = pdf_out, combined_png = png_out)
    )
    println("Saved ", pdf_out, " and ", png_out)
end

function run_multi_obstacle_reproduction(;
    path_to_fig::AbstractString,
    combined_prefix::AbstractString,
    fig_steps::Vector{Int},
    title::AbstractString,
    L::Int64 = 60,
    H::Int64 = 8,
    H₊::Int64 = 8,
    T::Float64 = 0.1,
    ns::Int64 = 3,
    R::Float64 = 20.0,
    ℓ::Float64 = 1.9,
    str_ang::Float64 = 0.6,
    v_min::Float64 = -10.0,
    v_max::Float64 = 20.0,
    a_min::Float64 = -2.0,
    a_max::Float64 = 2.0,
    init::Vector{Float64} = [0.0, 2.0, 0.0, 6.0],
    A::Matrix{Float64} = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0],
    b::Vector{Float64} = [2.3, 2.3, 1.0, 1.0],
    prob_conf::Float64 = 0.95,
    safe_weight::Float64 = 1.0,
    seed::Int = 1234,
    blas_threads::Int = 1,
    plot_only::Bool = false,
    show_step::Bool = true,
    tick_fontsize::Union{Int,Nothing} = 16,
    grid_size::Tuple{Int,Int} = (2400, 1200)
)
    LinearAlgebra.BLAS.set_num_threads(blas_threads)
    if plot_only
        return replot_from_snapshot(path_to_fig, combined_prefix, fig_steps, title;
                                    tick_fontsize=tick_fontsize, show_step=show_step, grid_size=grid_size)
    end

    mkpath(path_to_fig)
    pdf_out = combined_prefix * ".pdf"
    png_out = combined_prefix * ".png"
    rm(pdf_out; force = true)
    rm(png_out; force = true)

    Random.seed!(seed)
    pBounds      = polyBound(str_ang, v_min, v_max, a_min, a_max)
    obs_circle   = obstacle(2., [18., -10.], ns)
    obs_straight = obstacle(2., [30., 18.], ns)
    obs_list     = [obs_circle, obs_straight]
    robo         = robot(T, H, H₊, R, ℓ, pBounds, copy(init), A, b)

    obsGPs = [Vector{GPBase}(undef, 2) for _ in 1:length(obs_list)]
    for i in 1:length(obs_list)
        obsGPs[i][1] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
        obsGPs[i][2] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
    end
    mNNs = [Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh)) for _ in 1:length(obs_list)]

    selected_plots = Plots.Plot[]
    solver_logs    = NamedTuple[]
    traj_rows      = NamedTuple[]
    step_frames    = NamedTuple[]
    min_clearance  = Inf

    println("Now start the simulation (Scenario 3 / Multi-obstacle, seed = $seed, BLAS threads = $blas_threads)")
    try
        for k in 1:L
            println("Time instance $k")
            run_obs!(obs_circle, k - 1, T, 2)
            run_obs_topdown!(obs_straight, k - 1, T)

            clearance = minimum(robot_obstacle_clearance(robo, o) for o in obs_list)
            min_clearance = min(min_clearance, clearance)

            Ref  = [[6(k + i - 1) * T, 2.] for i in 1:H₊]
            Prob = fill(prob_conf, H)
            φ    = [quantile(Normal(0., 1.), (1 + Prob[i]) / 2) for i in 1:H]

            active_ids = [i for i in eachindex(obs_list) if norm(robo.pose[1:2] - obs_list[i].posn) < robo.R]

            obs_entries = NamedTuple[]
            if !isempty(active_ids)
                Pmeans     = Matrix{Float64}[]
                Pvars      = Matrix{Float64}[]
                active_obs = obstacle[]
                for i in active_ids
                    Pmean, Pvar = predict_obstacle!(robo, obs_list[i], obsGPs[i], mNNs[i], k)
                    push!(Pmeans, Pmean)
                    push!(Pvars, Pvar)
                    push!(active_obs, obs_list[i])
                end

                x, y, θ, v, tα, a = nMPC_avoid_multi(
                    robo, active_obs, Ref, Pmeans, Pvars, φ;
                    safe_weight = safe_weight, step = k, solver_log = solver_logs
                )
                robo.predPose = [x'; y'; θ'; v']

                for i in eachindex(obs_list)
                    idx = findfirst(isequal(i), active_ids)
                    if idx === nothing
                        push!(obs_entries, (active = false, obs = deepcopy(obs_list[i]), Pmean = zeros(0, 0), Pvar = zeros(0, 0), φ = Float64[]))
                    else
                        push!(obs_entries, (active = true, obs = deepcopy(obs_list[i]), Pmean = copy(Pmeans[idx]), Pvar = copy(Pvars[idx]), φ = copy(φ)))
                    end
                end
            else
                x, y, θ, v, tα, a = nMPC_free(robo, Ref; step = k, solver_log = solver_logs)
                robo.predPose = [x'; y'; θ'; v']
                for o in obs_list
                    push!(obs_entries, (active = false, obs = deepcopy(o), Pmean = zeros(0, 0), Pvar = zeros(0, 0), φ = Float64[]))
                end
            end

            frame = (step = k, type_of_obs = 0, robo = deepcopy(robo),
                     pred_x = collect(Float64, x), pred_y = collect(Float64, y), obs = obs_entries)
            push!(step_frames, frame)

            fig = render_frame(frame, :multi; show_step = show_step)
            savefig(fig, joinpath(path_to_fig, "$k.pdf"))
            if k in fig_steps
                fig_grid = render_frame(frame, :multi; tick_fontsize = tick_fontsize, show_step = show_step, grid_margins = true)
                push!(selected_plots, fig_grid)
            end

            rec = solver_logs[end]
            push!(traj_rows, (
                step = k, x = robo.pose[1], y = robo.pose[2], theta = robo.pose[3], v = robo.pose[4],
                tan_alpha = tα[1], a = a[1], min_clearance_m = clearance,
                solver = rec.solver, solver_attempts = rec.attempts,
                termination_status = rec.termination_status, primal_status = rec.primal_status,
                feasible = rec.feasible, solve_time_s = rec.solve_time_s, wall_time_s = rec.wall_time_s
            ))
            run!(robo, [tα[1], a[1]])
        end
    finally
        write_solver_log_csv(joinpath(path_to_fig, "solver_log.csv"), solver_logs)
        !isempty(traj_rows) && write_trajectory_csv(joinpath(path_to_fig, "trajectory.csv"), traj_rows)
    end

    save_combined_grid(selected_plots, pdf_out, png_out; layout = (2, 3), size = grid_size)
    snapshot_path = joinpath(path_to_fig, "snapshot.jls")
    meta = (L = L, H = H, H₊ = H₊, b1 = b[1], safe_weight = safe_weight, seed = seed, blas_threads = blas_threads, min_clearance_m = min_clearance)
    Serialization.serialize(snapshot_path, (kind = :multi, meta = meta, step_frames = step_frames, traj_rows = traj_rows, solver_logs = solver_logs))

    max_attempts = isempty(solver_logs) ? 0 : maximum(r.attempts for r in solver_logs)
    write_reproduction_notes(
        joinpath(path_to_fig, "README_repro.md"),
        title,
        (steps_completed = length(traj_rows), L = L, H = H, H₊ = H₊, b1 = b[1],
         safe_weight = safe_weight, seed = seed, blas_threads = blas_threads,
         min_clearance_m = min_clearance, max_attempts = max_attempts,
         path_to_fig = path_to_fig, combined_pdf = pdf_out, combined_png = png_out)
    )
    println("Saved ", pdf_out, " and ", png_out)
end
