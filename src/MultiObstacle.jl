function run_obs_topdown!(obs::obstacle, k::Int64, τ::Float64)
    ns = obs.ns
    Δτ = τ / ns

    rng       = Random.MersenneTwister(42)
    obs.time  = [k * τ + i * Δτ for i in 1:ns]
    obs.tseri = [obs.tseri; obs.time]

    for i in 1:ns
        obs.δpos[:, i] = Δτ * [0., -3.5] + Δτ * randn(rng, Float64, 2) / ns
        obs.traj       = [obs.traj obs.posn + sum(obs.δpos[:, j] for j in 1:i)]
    end

    obs.posn = obs.traj[:, end]
end

function predict_obstacle!(robo::robot, obs::obstacle, obsGP::Vector{GPBase}, mNN::Chain, k::Int64)
    Detection!(robo, obs, obsGP, mNN)

    dim = length(obs.posn)
    ns  = obs.ns
    H   = robo.H
    T   = robo.T

    δme = zeros(dim, ns * H)
    δva = zeros(dim, ns * H)
    for i in 1:dim
        δme[i, :], δva[i, :] = predictGP(mNN, obsGP[i], k * T .+ T * [x / ns for x in 1:H * ns], i)
    end

    δme[:, 1] += obs.posn
    for i in 1:H * ns - 1
        δme[:, i + 1] += δme[:, i]
        δva[:, i + 1] += δva[:, i] + [exp(2obsGP[1].logNoise.value), exp(2obsGP[2].logNoise.value)]
    end

    Pmean = δme[:, [i * ns for i in 1:H]]
    Pvar  = δva[:, [i * ns for i in 1:H]]
    return Pmean, Pvar
end

function nMPC_avoid_multi(robo::robot, obs_list::Vector{obstacle}, ref::Vector{Vector{Float64}},
                          Pmeans::Vector{Matrix{Float64}}, Pvars::Vector{Matrix{Float64}}, φ::Vector{Float64};
                          safe_weight::Float64=1.0,
                          step::Union{Int,Nothing}=nothing,
                          solver_log::Union{Nothing,AbstractVector}=nothing)
    H     = robo.H
    H₊    = robo.H₊
    τ     = robo.T
    A     = robo.A
    b     = robo.b
    E     = A
    ℓ     = robo.ℓ
    bds   = robo.pBnd
    state = robo.pose

    dimR = length(robo.b)
    dimO = 4
    nobs = length(obs_list)

    d = [[φ[i] * [sqrt(Pvars[j][1, i]), sqrt(Pvars[j][1, i]), sqrt(Pvars[j][2, i]), sqrt(Pvars[j][2, i])]
          for i in 1:H] for j in 1:nobs]

    function build_model(guess)
        opti = JuMP.Model(Ipopt.Optimizer)
        set_silent(opti)

        x₀, y₀, θ₀, v₀, tα₀, a₀ = guess.x, guess.y, guess.θ, guess.v, guess.tα, guess.a
        @variable(opti, x[i = 1:H₊], start = x₀[i])
        @variable(opti, y[i = 1:H₊], start = y₀[i])
        @variable(opti, θ[i = 1:H₊], start = θ₀[i])
        @variable(opti, v[i = 1:H₊], start = v₀[i])

        @variable(opti, tα[i = 1:H₊], start = tα₀[i])
        @variable(opti, a[i = 1:H₊], start = a₀[i])
        @variable(opti, λ[1:dimR, 1:H₊, 1:nobs])
        @variable(opti, μ[1:dimO, 1:H₊, 1:nobs])
        @variable(opti, safe[1:H₊, 1:nobs])

        for j in 1:nobs
            r_j = obs_list[j].r
            for i in 1:H₊
                obs_idx = i <= H ? i : 1
                λ₀, μ₀, safe₀ = init_separating_duals([x₀[i], y₀[i]], θ₀[i], Pmeans[j][:, obs_idx], d[j][obs_idx], r_j, A, b, E)
                set_start_value.(λ[:, i, j], λ₀)
                set_start_value.(μ[:, i, j], μ₀)
                set_start_value(safe[i, j], safe₀)
            end
        end

        J = sum(dot([x[i], y[i]] - ref[i], [x[i], y[i]] - ref[i]) - safe_weight * sum(safe[i, j] for j in 1:nobs) for i in 1:H₊)
        @objective(opti, Min, J)

        @constraint(opti, safe .>= 2.)
        @constraint(opti, safe .<= 10.)

        @constraints(opti, begin
            bds.v_max .>= v .>= bds.v_min
            bds.a_max .>= a .>= bds.a_min
            tan(bds.str_ang) .>= tα .>= -tan(bds.str_ang)
        end)

        @constraint(opti, λ .>= 0)
        @constraint(opti, μ .>= 0)

        for i in 1:H₊
            if i == 1
                @constraint(opti, x[i] == state[1] + τ * state[4] * cos(state[3]))
                @constraint(opti, y[i] == state[2] + τ * state[4] * sin(state[3]))
                @constraint(opti, θ[i] == state[3] + τ * state[4] * tα[i] / ℓ)
                @constraint(opti, v[i] == state[4] + τ * a[i])
            else
                @constraint(opti, x[i] == x[i - 1] + τ * v[i - 1] * cos(θ[i - 1]))
                @constraint(opti, y[i] == y[i - 1] + τ * v[i - 1] * sin(θ[i - 1]))
                @constraint(opti, θ[i] == θ[i - 1] + τ * v[i - 1] * tα[i] / ℓ)
                @constraint(opti, v[i] == v[i - 1] + τ * a[i])
            end
        end

        for j in 1:nobs
            r = obs_list[j].r
            for i in 1:H
                @constraint(opti, dot(E' * μ[:, i, j], E' * μ[:, i, j]) <= 1.)
                @constraint(opti, μ[:, i, j]' * (E * ([x[i], y[i]] - Pmeans[j][:, i]) - d[j][i]) - λ[:, i, j]' * b >= r + safe[i, j])
                @constraint(opti, λ[:, i, j]' * A + μ[:, i, j]' * E * [cos(θ[i]) -sin(θ[i]); sin(θ[i]) cos(θ[i])] .== 0)
            end

            for i in H + 1:H₊
                @constraint(opti, dot(E' * μ[:, i, j], E' * μ[:, i, j]) <= 1.)
                @constraint(opti, μ[:, i, j]' * (E * ([x[i], y[i]] - Pmeans[j][:, 1]) - d[j][1]) - λ[:, i, j]' * b >= r + safe[i, j])
                @constraint(opti, λ[:, i, j]' * A + μ[:, i, j]' * E * [cos(θ[i]) -sin(θ[i]); sin(θ[i]) cos(θ[i])] .== 0)
            end
        end

        return opti, (x = x, y = y, θ = θ, v = v, tα = tα, a = a)
    end

    return solve_with_retries(build_model, candidate_initial_guesses(robo), "nMPC_avoid_multi"; step=step, solver_log=solver_log)
end

function plot_obs_multi!(obs::obstacle)
    plot!(obs.traj[1, :], obs.traj[2, :], label = "", linewidth = 2, linecolor = :blue)

    pts = Plots.partialcircle(0, 2π, 100, obs.r)
    X, Y = Plots.unzip(pts)
    X = obs.posn[1] .+ X
    Y = obs.posn[2] .+ Y
    pts = collect(zip(X, Y))
    plot!(Shape(pts), fillcolor = plot_color(:blue, 0.5), label = "")
end

function plot_obs_multi!(obs::obstacle, Pmean::Matrix{Float64}, Pvar::Matrix{Float64}, φ::Vector{Float64})
    plot_obs_multi!(obs)

    H_pred = size(Pmean, 2)
    for i in 1:H_pred
        δ₁ = φ[i] * sqrt(Pvar[1, i])
        δ₂ = φ[i] * sqrt(Pvar[2, i])
        rec = Shape([Pmean[1, i] + δ₁, Pmean[1, i] + δ₁, Pmean[1, i] - δ₁, Pmean[1, i] - δ₁],
                    [Pmean[2, i] + δ₂, Pmean[2, i] - δ₂, Pmean[2, i] - δ₂, Pmean[2, i] + δ₂])
        plot!(rec, fillcolor = plot_color(:orange, 0.4), label = "")
    end
    plot!([obs.posn[1]; Pmean[1, :]], [obs.posn[2]; Pmean[2, :]], label = "", linewidth = 2, linecolor = :orange)
end

function plot_robot_mobs(robo::robot, x::Vector{Float64}, y::Vector{Float64}, k::Int64; show_step::Bool=true)
    θ = robo.pose[3]
    p = robo.pose[1:2]
    b = robo.b
    Rot = [cos(θ) -sin(θ); sin(θ) cos(θ)]
    RU = Rot * [b[1], b[3]]
    RD = Rot * [b[1], -b[3]]
    LU = -RD
    LD = -RU
    rec = Shape([p[1] + RU[1], p[1] + RD[1], p[1] + LD[1], p[1] + LU[1]],
                [p[2] + RU[2], p[2] + RD[2], p[2] + LD[2], p[2] + LU[2]])

    fig = plot(Array(0:45), 2ones(46), size = (600, 500), aspect_ratio = :equal, color = :brown, linestyle = :dash, linewidth = 2, label = "")
    plot!(rec, label = "", fillcolor = plot_color(:red, 0.5), fontsize = 20)

    pts = Plots.partialcircle(0, 2π, 100, robo.R)
    X, Y = Plots.unzip(pts)
    X = robo.pose[1] .+ X
    Y = robo.pose[2] .+ Y
    pts = collect(zip(X, Y))
    plot!(Shape(pts), fillcolor = plot_color(:yellow, 0.1), linestyle = :dash, label = "")

    xlims!((0., 45.))
    ylims!((-12., 20.))
    show_step && annotate!(15, 20, text("Step $k", "Times", :red, :right, 20))

    plot!([p[1]; x], [p[2]; y], label = "", linewidth = 2, linecolor = :green, xtickfontsize = 20, ytickfontsize = 20)
    plot!(robo.traj[1, :], robo.traj[2, :], label = "", linewidth = 2, linecolor = :red)
    apply_publication_style!(fig)
    return fig
end
