# Shared motion, prediction, and MPC for multiple obstacles.
function run_obs_topdown!(obs::obstacle, k::Int64, τ::Float64; rng=Random.default_rng())
    advance_obstacle!(_ -> [0., -3.5], obs, k, τ, rng)
end


function predict_obstacle!(robo::robot, obs::obstacle, obsGP::Vector{GPBase}, mNN::DisplacementRNN, elapsed_steps::Int64)
    decision_time = elapsed_steps * robo.T
    isapprox(last(obs.tseri), decision_time) || throw(ArgumentError("Obstacle position and decision time must agree"))
    Detection!(robo, obs, obsGP, mNN)
    ns, H, T = obs.ns, robo.H, robo.T
    times = [decision_time + j * (T / ns) for j in 1:(ns * H)]

    δme, δva = predict_displacements(mNN, obsGP, times, T / ns)

    # Eq. (12): each latent displacement contributes its own observation noise.
    δva .+= GaussianProcesses.noise_variance.(obsGP)
    δme[:, 1] += obs.posn
    for i in 1:H * ns - 1
        δme[:, i + 1] += δme[:, i]
        δva[:, i + 1] += δva[:, i]
    end

    Pmean = δme[:, [i * ns for i in 1:H]]
    Pvar  = δva[:, [i * ns for i in 1:H]]
    return Pmean, Pvar
end


function nMPC_avoid_multi(robo::robot, obs_list::Vector{obstacle}, ref::Vector{Vector{Float64}},
                          Pmeans::Vector{Matrix{Float64}}, Pvars::Vector{Matrix{Float64}}, φ::Vector{Float64};
                          diagnostics=nothing, initial_controls=nothing, retry=true, safety_weight=1.0)
    opti = JuMP.Model(Ipopt.Optimizer)
    set_silent(opti)

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

    x₀ = robo.predPose[1, :]
    y₀ = robo.predPose[2, :]
    θ₀ = robo.predPose[3, :]
    v₀ = robo.predPose[4, :]
    if initial_controls !== nothing
        steering, acceleration = initial_controls
        guess = copy(state)
        for i in 1:H₊
            x₀[i] = guess[1] + τ * guess[4] * cos(guess[3])
            y₀[i] = guess[2] + τ * guess[4] * sin(guess[3])
            θ₀[i] = guess[3] + τ * guess[4] * steering / ℓ
            v₀[i] = clamp(guess[4] + τ * acceleration, bds.v_min, bds.v_max)
            guess = [x₀[i], y₀[i], θ₀[i], v₀[i]]
        end
    end
    @variable(opti, x[i = 1:H₊], start = x₀[i])
    @variable(opti, y[i = 1:H₊], start = y₀[i])
    @variable(opti, θ[i = 1:H₊], start = θ₀[i])
    @variable(opti, v[i = 1:H₊], start = v₀[i])

    @variable(opti, tα[1:H₊])
    @variable(opti, a[1:H₊])
    @variable(opti, λ[1:dimR, 1:H₊, 1:nobs])
    @variable(opti, μ[1:dimO, 1:H₊, 1:nobs])
    @variable(opti, safe[1:H₊, 1:nobs])

    # A zero separating normal is a degenerate start for the bilinear dual
    # constraints. Initialize normals from obstacle to predicted robot position,
    # and initialize the matching robot-frame support coefficients.
    for i in 1:H₊
        set_start_value(tα[i], initial_controls === nothing ? robo.input[1] : initial_controls[1])
        set_start_value(a[i], initial_controls === nothing ? robo.input[2] : initial_controls[2])
        rotation = [cos(θ₀[i]) -sin(θ₀[i]); sin(θ₀[i]) cos(θ₀[i])]
        for j in 1:nobs
            column = i <= H ? i : 1
            delta = [x₀[i], y₀[i]] - Pmeans[j][:, column]
            normal = norm(delta) > 1e-8 ? delta / norm(delta) : [1.0, 0.0]
            robot_normal = -rotation' * normal
            obstacle_dual = [max(normal[1], 0), max(-normal[1], 0), max(normal[2], 0), max(-normal[2], 0)]
            robot_dual = [max(robot_normal[1], 0), max(-robot_normal[1], 0), max(robot_normal[2], 0), max(-robot_normal[2], 0)]
            for face in 1:4
                set_start_value(μ[face, i, j], obstacle_dual[face])
                set_start_value(λ[face, i, j], robot_dual[face])
            end
            set_start_value(safe[i, j], 2.0)
        end
    end

    # J = sum(dot([x[i], y[i]] - ref[i], [x[i], y[i]] - ref[i]) - 10 * sum(safe[i, j] for j in 1:nobs) for i in 1:H₊)
    J = sum(dot([x[i], y[i]] - ref[i], [x[i], y[i]] - ref[i]) - safety_weight * sum(safe[i, j] for j in 1:nobs) for i in 1:H₊)
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

    JuMP.optimize!(opti)
    if !is_solved_and_feasible(opti) && retry
        # Try distinct turn/brake trajectories to escape a failed local basin.
        # Each attempt solves exactly the same objective and hard constraints.
        guesses = [(-tan(bds.str_ang), bds.a_min), (tan(bds.str_ang), bds.a_min),
                   (0.0, bds.a_min), (-tan(bds.str_ang) / 2, 0.0), (tan(bds.str_ang) / 2, 0.0)]
        for (attempt, controls) in enumerate(guesses)
            attempt_diagnostics = Any[]
            try
                solution = nMPC_avoid_multi(robo, obs_list, ref, Pmeans, Pvars, φ;
                                           diagnostics=attempt_diagnostics, initial_controls=controls, retry=false, safety_weight)
                if diagnostics !== nothing
                    push!(diagnostics, merge(only(attempt_diagnostics), (attempts=attempt + 1,)))
                end
                println("MPC recovered with $(attempt + 1) initial guesses")
                return solution
            catch err
                if !(err isa ErrorException && startswith(err.msg, "MPC solve failed:"))
                    rethrow()
                end
            end
        end
    end
    check_mpc_solution!(opti; diagnostics)

    x  = JuMP.value.(x)
    y  = JuMP.value.(y)
    θ  = JuMP.value.(θ)
    v  = JuMP.value.(v)
    tα = JuMP.value.(tα)
    a  = JuMP.value.(a)

    return x, y, θ, v, tα, a
end
