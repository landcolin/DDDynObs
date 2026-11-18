println("Loading project dependencies with Julia ", VERSION)
flush(stdout)
include(joinpath(@__DIR__, "..", "src", "environment.jl"))
H = 6  # Plotting functions use the same horizon as the simulation entry points.

function check_prediction_timing(robo, network)
    obs = obstacle(2.0, [10.0, 0.0], 3)
    dt = robo.T / obs.ns
    # x has constant speed 5 m/s; y increments equal their end timestamps.
    gps = GPBase[GPE(mean=m, kernel=SEArd([1.0], 1.0), logNoise=-2.0)
                 for m in (MeanConst(5dt), MeanLin([1.0]))]
    positions, variances = predict_obstacle!(robo, obs, gps, network, 0)
    @assert all(gp -> isempty(gp.y), gps)
    @assert all(isfinite, variances) && all(>=(0), variances)
    for h in 1:robo.H
        n = h * obs.ns
        @assert positions[1, h] ≈ 10 + 5h * robo.T
        @assert positions[2, h] ≈ dt * n * (n + 1) / 2
    end
    println("PASS: first decision uses no future observations; all prediction columns end at the robot horizon times")
end

function check_environment()
    Random.seed!(42)
    LinearAlgebra.BLAS.set_num_threads(1)

    bounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    shape = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    robo = robot(0.1, 6, 8, 20.0, 1.9, bounds, [0.0, 2.0, 0.0, 6.0], shape, [2.3, 2.3, 1.0, 1.0])
    network = Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh))
    check_prediction_timing(robo, network)
    ref = [[6i * robo.T, 2.0] for i in 1:robo.H₊]
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
    horizon_positions, _ = predict_obstacle!(robo, obs, gps, network, 1)
    # At the second decision, check the forecast against the fitted GP at
    # absolute substep times, starting from the measured end of the first step.
    observed_at = last(obs.time)
    @assert observed_at ≈ robo.T
    substep_ends = observed_at .+ (1:(obs.ns * robo.H)) .* (robo.T / obs.ns)
    for axis in eachindex(gps)
        @assert vec(gps[axis].x) == obs.time
        increments, _ = predict_f(gps[axis], substep_ends)
        for h in 1:robo.H
            @assert horizon_positions[axis, h] ≈ obs.posn[axis] + sum(increments[1:h*obs.ns])
        end
    end
    times = collect(0.2:0.1:0.7)
    means, variances = predictGP(network, gps[1], times, 1)
    @assert all(isfinite, means) && all(isfinite, variances)
    @assert minimum(variances) >= -1e-10
    initial_weights = copy(network[2].weight)
    trainNN(network, obs.time, obs.δpos; epochs=2)
    @assert norm(network[2].weight - initial_weights) > 0
    predictions = predictLSTM(network, times)
    @assert size(predictions) == (2, length(times)) && all(isfinite, predictions)
    # Exercise the trained-neural-mean prediction branch as well.
    means, variances = predictGP(network, gps[1], times, 1; minData=3)
    @assert all(isfinite, means) && all(isfinite, variances)
    println("PASS: GaussianProcesses prediction and Flux gradient/update")

    pmean = repeat(obs.posn, 1, robo.H) + [means'; means']
    pvar = [variances'; variances']
    for coverage in (0.80, 0.95, 0.99)
        q = rectangle_quantile(coverage)
        axis_probability = cdf(Normal(), q) - cdf(Normal(), -q)
        @assert isapprox(axis_probability^2, coverage; atol=1e-12, rtol=0)
    end
    φ = fill(rectangle_quantile(0.95), robo.H)
    solution = nMPC_avoid(robo, obs, ref, pmean, pvar, φ)
    @assert all(values -> all(isfinite, values), solution)
    println("PASS: obstacle-avoidance optimization")

    multi_robot = deepcopy(robo)
    multi_obstacles = [obstacle(2.0, [20.0, 6.0], 3), obstacle(2.0, [20.0, -6.0], 3)]
    multi_means = [repeat(obs.posn, 1, robo.H) for obs in multi_obstacles]
    multi_variances = [fill(1e-3, 2, robo.H) for _ in multi_obstacles]
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
