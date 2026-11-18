# Proposition 5.1: independent x and y intervals must each cover sqrt(joint_coverage).
rectangle_quantile(joint_coverage) = quantile(Normal(), (1 + sqrt(joint_coverage)) / 2)

# Each column is a two-axis displacement forecast made before that observation arrived.
mutable struct DisplacementRNN
    network::Chain
    posterior_inputs::Matrix{Float32}
end

DisplacementRNN() = DisplacementRNN(Chain(RNN(2 => 16, tanh), Dense(16 => 2, tanh)), zeros(Float32, 2, 0))

# Only successive substeps are next-step training pairs (a detection gap can span many steps).
function recent_sequence(times, dt)
    for i in length(times):-1:2
        isapprox(times[i] - times[i-1], dt) || return i:length(times)
    end
    return 1:length(times)
end

function gp_displacements(gps, times)
    means = zeros(length(gps), length(times))
    variances = similar(means)
    for (axis, gp) in enumerate(gps)
        if isempty(gp.y)
            x = reshape(times, 1, :)
            means[axis, :] = GaussianProcesses.mean(gp.mean, x)
            variances[axis, :] = diag(cov(gp.kernel, x, x))
        else
            means[axis, :], variances[axis, :] = predict_f(gp, times)
        end
    end
    return means, variances
end

# Replay observed posterior inputs to reconstruct the hidden state with the current weights.
# For unobserved future steps the previous RNN output itself becomes the next input.
function recurrent_priors(network, inputs, horizon)
    states = network[1](inputs)
    outputs = network[2](states)
    state = states[:, end]
    prior = outputs[:, end]
    future = zeros(Float64, 2, horizon)
    future[:, 1] = prior
    for h in 2:horizon
        hidden, state = network[1].cell(prior, state)
        prior = network[2](hidden)
        future[:, h] = prior
    end
    return outputs, future
end

function predict_displacements(model::DisplacementRNN, gps, times, dt; minData=9)
    means, variances = gp_displacements(gps, times)
    observed = vec(gps[1].x)
    recent = recent_sequence(observed, dt)
    if isempty(times) || length(recent) < minData || !isapprox(times[1] - last(observed), dt)
        return means, variances
    end
    all(h -> isapprox(times[h] - times[h-1], dt), 2:length(times)) ||
        throw(ArgumentError("RNN forecasts require consecutive obstacle substeps"))

    history, future = recurrent_priors(model.network, model.posterior_inputs[:, recent], length(times))
    for (axis, gp) in enumerate(gps)
        # Eq. (8) uses the mean at BOTH the training inputs and the query inputs.
        # Use the prescribed mean during warm-up and outside this contiguous sequence.
        prior = GaussianProcesses.mean(gp.mean, gp.x)
        prior[first(recent)+minData:last(recent)] .= history[axis, minData:end-1]
        cross_cov = cov(gp.kernel, gp.x, reshape(times, 1, :))
        means[axis, :] = future[axis, :] + cross_cov' * (gp.cK \ (gp.y - prior))
    end
    return means, variances
end

function train_displacement_rnn!(network, inputs, targets; epochs=400)
    optimizer = Flux.setup(ADAM(1e-2), network)
    best = deepcopy(network)
    bestloss = Flux.mse(network(inputs), targets)
    reference_loss = bestloss
    stale = 0
    for _ in 1:epochs
        grads = Flux.gradient(net -> Flux.mse(net(inputs), targets), network)
        Flux.update!(optimizer, network, grads[1])
        loss = Flux.mse(network(inputs), targets)
        if loss < bestloss
            bestloss = loss
            Flux.loadmodel!(best, network)
        end
        if loss < 0.95f0 * reference_loss
            reference_loss = loss
            stale = 0
        else
            stale += 1
            stale >= 50 && break
        end
    end
    Flux.loadmodel!(network, best)
end

function Detection!(robo::robot, obs::obstacle, gps::Vector{GPBase}, model::DisplacementRNN; minData=9, maxData=60)
    isempty(obs.time) && return
    norm(robo.pose[1:2] - obs.posn) < robo.R || return
    dt = robo.T / obs.ns
    first_new = isempty(gps[1].y) ? 1 : searchsortedlast(obs.time, last(gps[1].x)) + 1
    first_new > length(obs.time) && return
    new_samples = first_new:length(obs.time)
    # Keep the existing GP window policy; the RNN history uses exactly the same indices.
    drop = length(gps[1].y) > maxData ? length(new_samples) : 0
    for j in new_samples
        t = obs.time[j]
        posterior, _ = predict_displacements(model, gps, [t], dt; minData)
        keep = (j == first_new ? drop + 1 : 1):length(gps[1].y)
        for (axis, gp) in enumerate(gps)
            times = [vec(gp.x)[keep]; t]
            values = [gp.y[keep]; obs.δpos[axis, j]]
            gps[axis] = GPE(reshape(times, 1, :), values, gp.mean, gp.kernel, gp.logNoise)
        end
        model.posterior_inputs = hcat(model.posterior_inputs[:, keep], Float32.(posterior))
    end

    for gp in gps
        if length(gp.y) >= minData
            gp.kernel.iℓ2[1] = 1.
            gp.logNoise.value = -2.
            gp.kernel.σ2 = 1.
            GaussianProcesses.optimize!(gp, domean=false, kern=true, noise=true,
                noisebounds=[-3., -1.], kernbounds=[[-5., -5.], [5., 5.]], method=Optim.NelderMead())
        else
            GaussianProcesses.optimize!(gp, domean=true, kern=true, noise=true,
                noisebounds=[-3., -1.], kernbounds=[[-5., -5.], [5., 5.]])
        end
    end
    recent = recent_sequence(vec(gps[1].x), dt)
    if length(recent) >= max(minData, 2)
        inputs = model.posterior_inputs[:, first(recent):last(recent)-1]
        targets = Float32.(permutedims(hcat((gp.y[first(recent)+1:last(recent)] for gp in gps)...)))
        train_displacement_rnn!(model.network, inputs, targets)
    end
end
