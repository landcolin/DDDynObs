println("Loading project dependencies with Julia ", VERSION)
flush(stdout)
include(joinpath(@__DIR__, "..", "src", "environment.jl"))
using Test: @test_throws

function check_obstacle_noise()
    T = 0.1
    for ns in (1, 3), scenario in 1:4
        obs = obstacle(2., [0., 0.], ns)
        rng, reference = MersenneTwister(17), MersenneTwister(17)
        for k in 0:2
            if scenario == 4
                run_obs_topdown!(obs, k, T; rng)
            else
                run_obs!(obs, k, T, scenario; rng)
            end
            for (i, t) in enumerate(obs.time)
                nominal = if scenario == 1
                    [-5., 5sin(π*t)]
                elseif scenario == 2
                    [5cos(π*t/4), 5sin(π*t/4)]
                elseif scenario == 3
                    [0., 6.]
                else
                    [0., -3.5]
                end
                velocity_noise = obs.δpos[:, i] / (T / ns) - nominal
                @assert velocity_noise ≈ randn(reference, 2)
            end
        end
    end
    println("PASS: all motion models use a continuous stream of unit-normal velocity samples")
end

function check_prediction_timing(robo, network, ns)
    obs = obstacle(2.0, [10.0, 0.0], ns)
    dt = robo.T / obs.ns
    # x has constant speed 5 m/s; y increments equal their end timestamps.
    noise_std = [0.25, 0.5]
    gps = GPBase[GPE(mean=m, kernel=SEArd([1.0], 0.0), logNoise=log(σ))
                 for (m, σ) in zip((MeanConst(5dt), MeanLin([1.0])), noise_std)]
    positions, variances = predict_obstacle!(robo, obs, gps, network, 0)
    @assert all(gp -> isempty(gp.y), gps)
    @assert all(isfinite, variances) && all(>=(0), variances)
    @assert size(positions) == size(variances) == (2, max(robo.H, robo.H₊))
    for h in axes(positions, 2)
        n = h * obs.ns
        @assert positions[1, h] ≈ 10 + 5h * robo.T
        @assert positions[2, h] ≈ dt * n * (n + 1) / 2
        @assert variances[:, h] ≈ n .* (1 .+ noise_std.^2)
    end
    println("PASS: first-decision times, positions and Eq. (12) variances at every horizon (ns=$ns)")
end

function check_recurrent_forecast(robo, obs, gps, model)
    dt = robo.T / obs.ns
    # The first stored input uses no measurements, the second uses only the first.
    @assert model.posterior_inputs[:, 1] == [1, 1]
    for axis in 1:2
        first_gp = GPE(obs.time[1:1], obs.δpos[axis, 1:1], MeanConst(1.0), SEArd([1.0], 1.0), -2.0)
        m, _ = predict_f(first_gp, obs.time[2:2])
        @assert model.posterior_inputs[axis, 2] ≈ Float32(only(m))
    end
    run_obs!(obs, 1, robo.T, 1)
    next_mean, _ = predict_displacements(model, gps, obs.time[1:1], dt; minData=3)
    weights = copy(model.network[1].cell.Wh)
    Detection!(robo, obs, gps, model; minData=3)
    @assert model.posterior_inputs[:, 4] ≈ Float32.(vec(next_mean))
    @assert norm(model.network[1].cell.Wh - weights) > 0
    saved_inputs = copy(model.posterior_inputs)
    Detection!(robo, obs, gps, model; minData=3)
    @assert model.posterior_inputs == saved_inputs && length(gps[1].y) == 6

    times = last(obs.time) .+ dt .* collect(1:4)
    predicted, variance = predict_displacements(model, gps, times, dt; minData=3)
    # Independent one-step rollout. Historical inputs are observed posterior predictions,
    # then each future input is the previous output, with the hidden state carried across.
    cell, output = model.network[1].cell, model.network[2]
    state = Flux.initialstates(cell)
    n = size(model.posterior_inputs, 2)
    generated = zeros(2, n + length(times) - 1)
    for j in axes(generated, 2)
        x = j <= n ? model.posterior_inputs[:, j] : Float32.(generated[:, j-1])
        hidden, state = cell(x, state)
        generated[:, j] = output(hidden)
    end
    for (axis, gp) in enumerate(gps)
        prior = GaussianProcesses.mean(gp.mean, gp.x)
        prior[4:end] = generated[axis, 3:n-1]
        K = cov(gp.kernel, gp.x, gp.x) + exp(2gp.logNoise.value) * I
        cross = cov(gp.kernel, gp.x, reshape(times, 1, :))
        expected = generated[axis, n:end] + cross' * (K \ (gp.y - prior))
        @assert isapprox(predicted[axis, :], expected; atol=1e-6)
        @assert variance[axis, :] ≈ last(predict_f(gp, times))
    end
    println("PASS: causal GP inputs, recurrent-weight update and four-step RNN + GP posterior rollout")
end

function check_tail_constraints(robo, ref)
    obs = obstacle(2.0, [50.0, 2.0], 1)
    means = repeat(obs.posn, 1, robo.H₊)
    variances = zeros(2, robo.H₊)
    φ = ones(robo.H₊)
    single(m, v) = nMPC_avoid(robo, obs, ref, m, v, φ)
    multi(m, v) = nMPC_avoid_multi(robo, [obs], ref, [m], [v], φ; retry=false)

    for solve in (single, multi)
        @test_throws DimensionMismatch solve(means[:, 1:robo.H], variances)
        for h in robo.H+1:robo.H₊
            # Each change alone leaves room ahead. Together the rectangle at h
            # covers the robot's entire reachable region at that step.
            shifted = copy(means)
            shifted[1, h] = 15.0
            expanded = copy(variances)
            expanded[:, h] .= 20.0^2
            solve(shifted, variances)
            solve(means, expanded)
            @test_throws "MPC solve failed:" solve(shifted, expanded)
        end
    end
    println("PASS: both MPC solvers use the matching center and width at steps 7 and 8")
end

function check_environment()
    check_obstacle_noise()
    Random.seed!(42)
    LinearAlgebra.BLAS.set_num_threads(1)

    bounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    shape = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    robo = robot(0.1, 6, 8, 20.0, 1.9, bounds, [0.0, 2.0, 0.0, 6.0], shape, [2.3, 2.3, 1.0, 1.0])
    network = DisplacementRNN()
    for ns in (1, 3)
        check_prediction_timing(robo, network, ns)
    end
    ref = [[6i * robo.T, 2.0] for i in 1:robo.H₊]
    check_tail_constraints(robo, ref)
    x, y, θ, v, steering, acceleration = nMPC_free(robo, ref)
    @assert all(isfinite, vcat(x, y, θ, v, steering, acceleration))
    @assert isapprox(x[1], 0.6; atol=1e-5)
    run!(robo, [steering[1], acceleration[1]])
    @assert isapprox(robo.pose[1], 0.6; atol=1e-3)
    println("\nPASS: JuMP/Ipopt optimization and OrdinaryDiffEq integration")

    turning_robot = deepcopy(robo)
    turning_ref = [[robo.pose[1] + 6i * robo.T, 2.0 + 0.15i] for i in 1:robo.H₊]
    _, _, predicted_yaw, _, turn, accel = nMPC_free(turning_robot, turning_ref)
    @assert abs(turn[1]) > 1e-3
    run!(turning_robot, [turn[1], accel[1]])
    @assert isapprox(turning_robot.pose[3], predicted_yaw[1]; atol=0.02)
    println("PASS: MPC turning prediction agrees with simulated yaw")

    obs = obstacle(2.0, [10.0, 0.0], 3)
    gps = GPBase[GPE(mean=MeanConst(1.0), kernel=SEArd([1.0], 1.0), logNoise=-2.0) for _ in 1:2]
    run_obs!(obs, 0, robo.T, 1)
    horizon_positions, horizon_variances = predict_obstacle!(robo, obs, gps, network, 1)
    # At the second decision, check the forecast against the fitted GP at
    # absolute substep times, starting from the measured end of the first step.
    observed_at = last(obs.time)
    @assert observed_at ≈ robo.T
    substep_ends = observed_at .+ (1:(obs.ns * robo.H₊)) .* (robo.T / obs.ns)
    for axis in eachindex(gps)
        @assert vec(gps[axis].x) == obs.time
        increments, _ = predict_f(gps[axis], substep_ends)
        _, noisy_variances = predict_y(gps[axis], substep_ends)
        for h in 1:robo.H₊
            @assert horizon_positions[axis, h] ≈ obs.posn[axis] + sum(increments[1:h*obs.ns])
            @assert horizon_variances[axis, h] ≈ sum(noisy_variances[1:h*obs.ns])
        end
    end
    times = observed_at .+ (1:robo.H₊) .* robo.T
    means, variances = predict_f(gps[1], times)
    check_recurrent_forecast(robo, deepcopy(obs), gps, network)

    pmean = repeat(obs.posn, 1, robo.H₊) + [means'; means']
    pvar = [variances'; variances']
    for coverage in (0.80, 0.95, 0.99)
        q = rectangle_quantile(coverage)
        axis_probability = cdf(Normal(), q) - cdf(Normal(), -q)
        @assert isapprox(axis_probability^2, coverage; atol=1e-12, rtol=0)
    end
    φ = fill(rectangle_quantile(0.95), robo.H₊)
    solution = nMPC_avoid(robo, obs, ref, pmean, pvar, φ)
    @assert all(values -> all(isfinite, values), solution)
    println("PASS: obstacle-avoidance optimization")

    multi_robot = deepcopy(robo)
    multi_obstacles = [obstacle(2.0, [20.0, 6.0], 3), obstacle(2.0, [20.0, -6.0], 3)]
    multi_means = [repeat(obs.posn, 1, robo.H₊) for obs in multi_obstacles]
    multi_variances = [fill(1e-3, 2, robo.H₊) for _ in multi_obstacles]
    multi_diagnostics = Any[]
    multi_solution = nMPC_avoid_multi(multi_robot, multi_obstacles, turning_ref, multi_means, multi_variances, φ;
                                    diagnostics=multi_diagnostics)
    @assert all(values -> all(isfinite, values), multi_solution)
    @assert only(multi_diagnostics).feasible
    @assert abs(multi_solution[5][1]) > 1e-3
    run!(multi_robot, [multi_solution[5][1], multi_solution[6][1]])
    @assert isapprox(multi_robot.pose[3], multi_solution[3][1]; atol=0.02)
    println("PASS: two-obstacle MPC and simulated turning agree")

    mktempdir() do output
        fig = plot_robot(robo, x, y, 1, 1)
        plot_obs(obs, pmean, pvar, φ)
        target = joinpath(output, "environment_check.pdf")
        savefig(fig, target)
        @assert filesize(target) > 0
    end
    println("PASS: headless Plots/GR PDF export")
    println("Environment check passed.")
end

check_environment()
