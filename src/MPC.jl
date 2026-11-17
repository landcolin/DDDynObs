function ensure_solved(opti::JuMP.Model, name::AbstractString;
                       step::Union{Int,Nothing}=nothing,
                       solver_log::Union{Nothing,AbstractVector}=nothing,
                       attempts::Int=1,
                       wall_time_s::Float64=NaN)
    term_status = JuMP.termination_status(opti)
    prim_status = JuMP.primal_status(opti)
    feasible    = JuMP.is_solved_and_feasible(opti)
    has_vals    = JuMP.has_values(opti)
    solve_t     = try
        JuMP.solve_time(opti)
    catch
        NaN
    end

    record = (
        step = step === nothing ? -1 : step,
        solver = String(name),
        attempts = attempts,
        termination_status = string(term_status),
        primal_status = string(prim_status),
        feasible = feasible,
        has_values = has_vals,
        solve_time_s = solve_t,
        wall_time_s = wall_time_s,
    )
    if solver_log !== nothing
        push!(solver_log, record)
    end

    if !(feasible && has_vals)
        step_info = step === nothing ? "" : " at step $step"
        error("$name failed$step_info: termination_status=$term_status, primal_status=$prim_status, feasible=$feasible, has_values=$has_vals, solve_time=$(round(solve_t, digits=4))s")
    end
    return record
end

function write_solver_log_csv(filepath::AbstractString, solver_log::AbstractVector)
    mkpath(dirname(filepath))
    open(filepath, "w") do io
        println(io, "step,solver,attempts,termination_status,primal_status,feasible,has_values,solve_time_s,wall_time_s")
        for r in solver_log
            println(io, join([
                r.step,
                r.solver,
                r.attempts,
                r.termination_status,
                r.primal_status,
                r.feasible,
                r.has_values,
                r.solve_time_s,
                r.wall_time_s,
            ], ","))
        end
    end
    return filepath
end

function init_separating_duals(p_robot::AbstractVector{<:Real},
                               θ_robot::Real,
                               p_obs::AbstractVector{<:Real},
                               d_vec::AbstractVector{<:Real},
                               r::Real,
                               A::AbstractMatrix{<:Real},
                               b::AbstractVector{<:Real},
                               E::AbstractMatrix{<:Real};
                               scale::Float64=0.95)
    Δp  = p_robot - p_obs
    nrm = norm(Δp)
    s   = nrm > 1e-8 ? (scale * Δp / nrm) : [0.0, scale]

    μ₀  = [max(s[1], 0.0), max(-s[1], 0.0), max(s[2], 0.0), max(-s[2], 0.0)]
    Rot = [cos(θ_robot) -sin(θ_robot); sin(θ_robot) cos(θ_robot)]
    w   = Rot' * (E' * μ₀)
    λ₀  = [max(-w[1], 0.0), max(w[1], 0.0), max(-w[2], 0.0), max(w[2], 0.0)]

    margin = dot(μ₀, E * Δp - d_vec) - dot(λ₀, b) - r
    safe₀  = clamp(margin, 2.0, 10.0)
    return λ₀, μ₀, safe₀
end

function rollout_initial_guess(robo::robot, tα_cmd::Real, a_cmd::Real)
    H₊    = robo.H₊
    τ     = robo.T
    ℓ     = robo.ℓ
    bds   = robo.pBnd
    state = robo.pose

    tα_val = clamp(Float64(tα_cmd), -tan(bds.str_ang), tan(bds.str_ang))
    a_val  = clamp(Float64(a_cmd), bds.a_min, bds.a_max)

    x₀  = Vector{Float64}(undef, H₊)
    y₀  = Vector{Float64}(undef, H₊)
    θ₀  = Vector{Float64}(undef, H₊)
    v₀  = Vector{Float64}(undef, H₊)
    tα₀ = fill(tα_val, H₊)
    a₀  = Vector{Float64}(undef, H₊)

    for i in 1:H₊
        px, py, pθ, pv = i == 1 ? (state[1], state[2], state[3], state[4]) : (x₀[i - 1], y₀[i - 1], θ₀[i - 1], v₀[i - 1])
        x₀[i] = px + τ * pv * cos(pθ)
        y₀[i] = py + τ * pv * sin(pθ)
        θ₀[i] = pθ + τ * pv * tα₀[i] / ℓ
        v_next = clamp(pv + τ * a_val, bds.v_min, bds.v_max)
        a₀[i]  = (v_next - pv) / τ
        v₀[i]  = v_next
    end
    return (x = x₀, y = y₀, θ = θ₀, v = v₀, tα = tα₀, a = a₀)
end

function candidate_initial_guesses(robo::robot)
    H₊ = robo.H₊
    bds = robo.pBnd
    tα_lim = tan(bds.str_ang)
    primary = (
        x = Vector{Float64}(robo.predPose[1, :]),
        y = Vector{Float64}(robo.predPose[2, :]),
        θ = Vector{Float64}(robo.predPose[3, :]),
        v = Vector{Float64}(robo.predPose[4, :]),
        tα = zeros(H₊),
        a = zeros(H₊),
    )
    return [
        primary,
        rollout_initial_guess(robo,  0.5 * tα_lim, 0.0),
        rollout_initial_guess(robo, -0.5 * tα_lim, 0.0),
        rollout_initial_guess(robo,  0.0,          bds.a_min),
        rollout_initial_guess(robo,  0.5 * tα_lim, 0.5 * bds.a_min),
        rollout_initial_guess(robo, -0.5 * tα_lim, 0.5 * bds.a_min),
    ]
end

function solve_with_retries(build_model_fn::Function,
                            candidates::AbstractVector,
                            name::AbstractString;
                            step::Union{Int,Nothing}=nothing,
                            solver_log::Union{Nothing,AbstractVector}=nothing)
    t0 = time()
    n_candidates = length(candidates)
    for (attempt, guess) in enumerate(candidates)
        opti, vars = build_model_fn(guess)
        JuMP.optimize!(opti)
        feasible = JuMP.is_solved_and_feasible(opti) && JuMP.has_values(opti)
        if feasible || attempt == n_candidates
            ensure_solved(opti, name; step=step, solver_log=solver_log, attempts=attempt, wall_time_s=time() - t0)
            return JuMP.value.(vars.x), JuMP.value.(vars.y), JuMP.value.(vars.θ), JuMP.value.(vars.v), JuMP.value.(vars.tα), JuMP.value.(vars.a)
        end
    end
end

function nMPC_avoid(robo::robot, obs::obstacle, ref::Vector{Vector{Float64}}, Pmean::Matrix{Float64}, Pvar::Matrix{Float64}, φ::Vector{Float64};
                    step::Union{Int,Nothing}=nothing,
                    solver_log::Union{Nothing,AbstractVector}=nothing)
    H     = robo.H
    H₊    = robo.H₊
    τ     = robo.T
    A     = robo.A
    b     = robo.b
    E     = A
    ℓ     = robo.ℓ
    r     = obs.r

    d = [φ[i]*[sqrt(Pvar[1,i]), sqrt(Pvar[1,i]), sqrt(Pvar[2,i]), sqrt(Pvar[2,i])] for i in 1:H]

    bds   = robo.pBnd
    state = robo.pose

    dimR = length(robo.b)
    dimO = length(d[1])

    function build_model(guess)
        opti = JuMP.Model(Ipopt.Optimizer)
        set_silent(opti)

        x₀, y₀, θ₀, v₀, tα₀, a₀ = guess.x, guess.y, guess.θ, guess.v, guess.tα, guess.a
        @variable(opti, x[i=1:H₊], start = x₀[i])
        @variable(opti, y[i=1:H₊], start = y₀[i])
        @variable(opti, θ[i=1:H₊], start = θ₀[i])
        @variable(opti, v[i=1:H₊], start = v₀[i])

        @variable(opti, tα[i=1:H₊], start = tα₀[i])
        @variable(opti, a[i=1:H₊], start = a₀[i])
        @variable(opti, λ[1:dimR,1:H₊])
        @variable(opti, μ[1:dimO,1:H₊])
        @variable(opti, safe[1:H₊])

        for i in 1:H₊
            obs_idx = i <= H ? i : 1
            λ₀, μ₀, safe₀ = init_separating_duals([x₀[i], y₀[i]], θ₀[i], Pmean[:, obs_idx], d[obs_idx], r, A, b, E)
            set_start_value.(λ[:, i], λ₀)
            set_start_value.(μ[:, i], μ₀)
            set_start_value(safe[i], safe₀)
        end

        J = sum(dot([x[i], y[i]] - ref[i], [x[i], y[i]] - ref[i]) - 10*safe[i] for i in 1:H₊)
        @objective(opti, Min, J)

        @constraint(opti, safe .>= 2.)
        @constraint(opti, safe .<= 10.)

        @constraints(opti, begin    bds.v_max  .>= v .>= bds.v_min
                                    bds.a_max  .>= a .>= bds.a_min
                                    tan(bds.str_ang) .>= tα .>= -tan(bds.str_ang) end)

        @constraint(opti, λ .>= 0)
        @constraint(opti, μ .>= 0)

        for i in 1:H₊
            if i == 1
                @constraint(opti, x[i] == state[1] + τ*state[4]*cos(state[3]))
                @constraint(opti, y[i] == state[2] + τ*state[4]*sin(state[3]))
                @constraint(opti, θ[i] == state[3] + τ*state[4]*tα[i]/ℓ)
                @constraint(opti, v[i] == state[4] + τ*a[i])
            else
                @constraint(opti, x[i] == x[i-1] + τ*v[i-1]*cos(θ[i-1]))
                @constraint(opti, y[i] == y[i-1] + τ*v[i-1]*sin(θ[i-1]))
                @constraint(opti, θ[i] == θ[i-1] + τ*v[i-1]*tα[i]/ℓ)
                @constraint(opti, v[i] == v[i-1] + τ*a[i])
            end
        end

        for i in 1:H
            @constraint(opti, dot(E'*μ[:,i], E'*μ[:,i]) <= 1.)
            @constraint(opti, μ[:,i]'*(E*([x[i], y[i]] - Pmean[:,i]) - d[i])- λ[:,i]'*b >= r+safe[i])
            @constraint(opti, λ[:,i]'*A + μ[:,i]'*E*[cos(θ[i]) -sin(θ[i]); sin(θ[i]) cos(θ[i])] .== 0)
        end

        for i in H+1:H₊
            @constraint(opti, dot(E'*μ[:,i], E'*μ[:,i]) <= 1.)
            @constraint(opti, μ[:,i]'*(E*([x[i], y[i]] - Pmean[:,1]) - d[1])- λ[:,i]'*b >= r+safe[i])
            @constraint(opti, λ[:,i]'*A + μ[:,i]'*E*[cos(θ[i]) -sin(θ[i]); sin(θ[i]) cos(θ[i])] .== 0)
        end

        return opti, (x = x, y = y, θ = θ, v = v, tα = tα, a = a)
    end

    return solve_with_retries(build_model, candidate_initial_guesses(robo), "nMPC_avoid"; step=step, solver_log=solver_log)
end

function nMPC_free(robo::robot, ref::Vector{Vector{Float64}};
                   step::Union{Int,Nothing}=nothing,
                   solver_log::Union{Nothing,AbstractVector}=nothing)
    H₊    = robo.H₊
    τ     = robo.T
    ℓ     = robo.ℓ
    bds   = robo.pBnd
    state = robo.pose

    function build_model(guess)
        opti = JuMP.Model(Ipopt.Optimizer)
        set_silent(opti)

        x₀, y₀, θ₀, v₀, tα₀, a₀ = guess.x, guess.y, guess.θ, guess.v, guess.tα, guess.a
        @variable(opti, x[i=1:H₊], start = x₀[i])
        @variable(opti, y[i=1:H₊], start = y₀[i])
        @variable(opti, θ[i=1:H₊], start = θ₀[i])
        @variable(opti, v[i=1:H₊], start = v₀[i])
        @variable(opti, tα[i=1:H₊], start = tα₀[i])
        @variable(opti, a[i=1:H₊], start = a₀[i])

        J = sum((x[i] - ref[i][1])^2 + (y[i] - ref[i][2])^2 + 20(tα[i+1] - tα[i])^2 for i in 1:H₊-1)
        @objective(opti, Min, J)

        @constraints(opti, begin    bds.v_max  .>= v .>= bds.v_min
                                    bds.a_max  .>= a .>= bds.a_min
                                    tan(bds.str_ang) .>= tα .>= -tan(bds.str_ang) end)
        for i in 1:H₊
            if i == 1
                @constraint(opti, x[i] == state[1] + τ*state[4]*cos(state[3]))
                @constraint(opti, y[i] == state[2] + τ*state[4]*sin(state[3]))
                @constraint(opti, θ[i] == state[3] + τ*state[4]*tα[i]/ℓ)
                @constraint(opti, v[i] == state[4] + τ*a[i])
            else
                @constraint(opti, x[i] == x[i-1] + τ*v[i-1]*cos(θ[i-1]))
                @constraint(opti, y[i] == y[i-1] + τ*v[i-1]*sin(θ[i-1]))
                @constraint(opti, θ[i] == θ[i-1] + τ*v[i-1]*tα[i]/ℓ)
                @constraint(opti, v[i] == v[i-1] + τ*a[i])
            end
        end

        return opti, (x = x, y = y, θ = θ, v = v, tα = tα, a = a)
    end

    return solve_with_retries(build_model, candidate_initial_guesses(robo), "nMPC_free"; step=step, solver_log=solver_log)
end