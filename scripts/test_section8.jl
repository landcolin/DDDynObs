const REPO_ROOT = dirname(@__DIR__)
const SRC_DIR   = joinpath(REPO_ROOT, "src")

include(joinpath(SRC_DIR, "environment.jl"))
include(joinpath(SRC_DIR, "objects.jl"))
include(joinpath(SRC_DIR, "computing.jl"))
include(joinpath(SRC_DIR, "MPC.jl"))
include(joinpath(SRC_DIR, "mPlots.jl"))
include(joinpath(SRC_DIR, "MultiObstacle.jl"))

using Test

function make_test_robot(; init = [0.0, 2.0, 0.0, 6.0])
    pBounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    A = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    b = [2.5, 2.5, 1.0, 1.0]
    return robot(0.1, 8, 8, 20.0, 1.9, pBounds, copy(init), A, b)
end

@testset "Section 8 Consistency & Recovery Verification" begin
    @testset "1. Robot ODE vs Forward Euler Yaw Consistency (Constant Speed)" begin
        robo = make_test_robot()
        u = [0.4, 0.0] # non-zero steering, zero acceleration
        θ_euler = robo.pose[3] + robo.T * robo.pose[4] * u[1] / robo.ℓ
        run!(robo, u)
        @test isapprox(robo.pose[3], θ_euler; atol = 1e-12)
    end

    @testset "2. Single-Obstacle MPC Steering Prediction vs ODE Integration" begin
        robo = make_test_robot()
        obs  = obstacle(2.0, [11.5, 1.8], 3)
        Ref  = [[6 * i * robo.T, 2.0] for i in 1:robo.H₊]
        Pmean = hcat([[11.5 - 0.5 * i * robo.T, 1.8 + 0.3 * i * robo.T] for i in 1:robo.H]...)
        Pvar  = fill(0.05, 2, robo.H)
        φ     = fill(quantile(Normal(0.0, 1.0), 0.975), robo.H)

        logs = NamedTuple[]
        x, y, θ, v, tα, a = nMPC_avoid(robo, obs, Ref, Pmean, Pvar, φ; step = 1, solver_log = logs)
        @test logs[end].feasible
        @test abs(tα[1]) > 1e-2 # ensure active steering so velocity factor is exercised

        θ_mpc = θ[1]
        run!(robo, [tα[1], a[1]])
        θ_ode = robo.pose[3]

        # Exact difference between Tsit5 and 1-step forward Euler for dθ/dt = v(t)*tα/ℓ with linear v(t)
        euler_quadrature_bound = 0.5 * abs(a[1] * tα[1]) * robo.T^2 / robo.ℓ + 1e-6
        @test abs(θ_mpc - θ_ode) <= euler_quadrature_bound
        # Confirm it is far closer than the buggy v-less formula (θ₀ + τ*tα/ℓ)
        θ_buggy = 0.0 + robo.T * tα[1] / robo.ℓ
        @test abs(θ_mpc - θ_ode) < 0.05 * abs(θ_buggy - θ_ode)
        println("Single-obs yaw check: θ_mpc=$(round(θ_mpc; digits=6)), θ_ode=$(round(θ_ode; digits=6)), |diff|=$(abs(θ_mpc - θ_ode))")
    end

    @testset "3. Multi-Obstacle MPC Steering Prediction vs ODE Integration" begin
        robo = make_test_robot()
        obs1 = obstacle(2.0, [11.5, 1.8], 3)
        obs2 = obstacle(2.0, [16.0, 6.0], 3)
        Ref  = [[6 * i * robo.T, 2.0] for i in 1:robo.H₊]
        Pmean1 = hcat([[11.5 - 0.4 * i * robo.T, 1.8 + 0.3 * i * robo.T] for i in 1:robo.H]...)
        Pmean2 = hcat([[16.0, 6.0 - 3.5 * i * robo.T] for i in 1:robo.H]...)
        Pvar1  = fill(0.05, 2, robo.H)
        Pvar2  = fill(0.05, 2, robo.H)
        φ      = fill(quantile(Normal(0.0, 1.0), 0.975), robo.H)

        logs = NamedTuple[]
        x, y, θ, v, tα, a = nMPC_avoid_multi(robo, [obs1, obs2], Ref, [Pmean1, Pmean2], [Pvar1, Pvar2], φ;
                                             safe_weight = 10.0, step = 1, solver_log = logs)
        @test logs[end].feasible
        @test abs(tα[1]) > 1e-2

        θ_mpc = θ[1]
        run!(robo, [tα[1], a[1]])
        θ_ode = robo.pose[3]

        euler_quadrature_bound = 0.5 * abs(a[1] * tα[1]) * robo.T^2 / robo.ℓ + 1e-6
        @test abs(θ_mpc - θ_ode) <= euler_quadrature_bound
        println("Multi-obs yaw check:  θ_mpc=$(round(θ_mpc; digits=6)), θ_ode=$(round(θ_ode; digits=6)), |diff|=$(abs(θ_mpc - θ_ode))")
    end

    @testset "4. Multi-Start Retry Recovery & Strict Infeasible Stop" begin
        robo = make_test_robot(init = [0.0, 2.0, 0.0, 6.0])
        Ref  = [[6 * i * robo.T, 2.0] for i in 1:robo.H₊]

        # 4a. Physically infeasible scenario must strictly stop (no infeasible iterate returned)
        obs_blocked = obstacle(2.0, [4.6, 2.0], 3)
        Pmean_blk   = hcat([[4.6, 2.0] for _ in 1:robo.H]...)
        Pvar_blk    = fill(0.02, 2, robo.H)
        φ           = fill(quantile(Normal(0.0, 1.0), 0.975), robo.H)
        logs_blk    = NamedTuple[]
        @test_throws ErrorException nMPC_avoid(robo, obs_blocked, Ref, Pmean_blk, Pvar_blk, φ; step = 99, solver_log = logs_blk)
        @test !logs_blk[end].feasible
        @test logs_blk[end].attempts == 6

        # 4b. Verify solve_with_retries triggers attempt >= 2 and recovers when primary guess stalls/fails
        forced_bad_primary = (
            x = fill(10.0, robo.H₊),
            y = fill(2.0, robo.H₊),
            θ = fill(0.0, robo.H₊),
            v = fill(0.0, robo.H₊),
            tα = zeros(robo.H₊),
            a = zeros(robo.H₊),
        )
        all_cands = candidate_initial_guesses(robo)
        cands_with_bad = vcat([forced_bad_primary], all_cands[2:end])

        # A non-convex avoidance problem where starting inside the obstacle center (x=10, y=2, v=0) fails on attempt 1
        function build_recovery_nlp(g)
            opti = JuMP.Model(Ipopt.Optimizer)
            set_silent(opti)
            @variable(opti, x[i=1:robo.H₊], start = g.x[i])
            @variable(opti, y[i=1:robo.H₊], start = g.y[i])
            @variable(opti, θ[i=1:robo.H₊], start = g.θ[i])
            @variable(opti, v[i=1:robo.H₊], start = g.v[i])
            @variable(opti, tα[i=1:robo.H₊], start = g.tα[i])
            @variable(opti, a[i=1:robo.H₊], start = g.a[i])
            @constraint(opti, x[1] == robo.pose[1] + robo.T * robo.pose[4] * cos(robo.pose[3]))
            @constraint(opti, y[1] == robo.pose[2] + robo.T * robo.pose[4] * sin(robo.pose[3]))
            @constraint(opti, θ[1] == robo.pose[3] + robo.T * robo.pose[4] * tα[1] / robo.ℓ)
            @constraint(opti, v[1] == robo.pose[4] + robo.T * a[1])
            for i in 2:robo.H₊
                @constraint(opti, x[i] == x[i-1] + robo.T * v[i-1] * cos(θ[i-1]))
                @constraint(opti, y[i] == y[i-1] + robo.T * v[i-1] * sin(θ[i-1]))
                @constraint(opti, θ[i] == θ[i-1] + robo.T * v[i-1] * tα[i] / robo.ℓ)
                @constraint(opti, v[i] == v[i-1] + robo.T * a[i])
            end
            @constraint(opti, -tan(0.6) .<= tα .<= tan(0.6))
            @constraint(opti, -2.0 .<= a .<= 2.0)
            @constraint(opti, -10.0 .<= v .<= 20.0)
            @objective(opti, Min, sum((x[i] - Ref[i][1])^2 + (y[i] - Ref[i][2])^2 for i in 1:robo.H₊))
            for i in 1:robo.H
                @constraint(opti, (x[i] - 10.0)^2 + (y[i] - 2.0)^2 >= 6.0^2)
            end
            return opti, (x = x, y = y, θ = θ, v = v, tα = tα, a = a)
        end

        logs2 = NamedTuple[]
        solve_with_retries(build_recovery_nlp, cands_with_bad, "retry_recovery_test"; step = 1, solver_log = logs2)
        @test logs2[end].feasible
        @test logs2[end].attempts >= 2
        println("Retry recovery confirmed: attempts=$(logs2[end].attempts), status=$(logs2[end].termination_status)")
    end
end

println("All Section 8 tests PASSED.")
