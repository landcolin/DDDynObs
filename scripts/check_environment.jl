println("Loading project dependencies with Julia ", VERSION)
flush(stdout)
include(joinpath(@__DIR__, "..", "src", "environment.jl"))
H = 6  # Plotting functions use the same horizon as the simulation entry points.

function check_environment()
    Random.seed!(42)
    LinearAlgebra.BLAS.set_num_threads(1)

    bounds = polyBound(0.6, -10.0, 20.0, -2.0, 2.0)
    shape = [1.0 0.0; -1.0 0.0; 0.0 1.0; 0.0 -1.0]
    robo = robot(0.1, 6, 8, 20.0, 1.9, bounds, [0.0, 2.0, 0.0, 6.0], shape, [2.3, 2.3, 1.0, 1.0])
    ref = [[6i * robo.T, 2.0] for i in 1:robo.H₊]
    x, y, θ, v, steering, acceleration = nMPC_free(robo, ref)
    @assert all(isfinite, vcat(x, y, θ, v, steering, acceleration))
    @assert isapprox(x[1], 0.6; atol=1e-5)
    run!(robo, [steering[1], acceleration[1]])
    @assert isapprox(robo.pose[1], 0.6; atol=1e-3)
    println("\nPASS: JuMP/Ipopt optimization and OrdinaryDiffEq integration")

    obs = obstacle(2.0, [10.0, 0.0], 3)
    gps = GPBase[GPE(mean=MeanConst(1.0), kernel=SEArd([1.0], 1.0), logNoise=-2.0) for _ in 1:2]
    network = Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh))
    run_obs!(obs, 0, robo.T, 1)
    Detection!(robo, obs, gps, network)
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
    φ = fill(quantile(Normal(), 0.975), robo.H)
    solution = nMPC_avoid(robo, obs, ref, pmean, pvar, φ)
    @assert all(values -> all(isfinite, values), solution)
    println("PASS: obstacle-avoidance optimization")

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
