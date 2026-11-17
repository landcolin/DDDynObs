dim = 2
T   = 0.1
H   = 8
H₊  = 8
L   = 70
ns  = 3

include("environment.jl")
include("objects.jl")
include("computing.jl")
include("MPC.jl")
include("mPlots.jl")
include("MultiObstacle.jl")

R = 20.
ℓ = 1.9
str_ang = 0.6
v_min   = -10.
v_max   = 20.
a_min   = -2.
a_max   =  2.
pBounds = polyBound(str_ang, v_min, v_max, a_min, a_max)

init = [0., 2., 0., 6.]
A    = [1. 0; -1. 0.; 0. 1.; 0. -1.]
b    = [2.3, 2.3, 1., 1.]

obs_circle   = obstacle(2., [18., -10.], ns)
obs_straight = obstacle(2., [30., 18.], ns)
obs_list     = [obs_circle, obs_straight]

robo = robot(T, H, H₊, R, ℓ, pBounds, init, A, b)

obsGPs = [Vector{GPBase}(undef, 2) for _ in 1:length(obs_list)]
for i in 1:length(obs_list)
    obsGPs[i][1] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
    obsGPs[i][2] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
end

mNNs = [Chain(RNN(1 => 16, tanh), Dense(16 => 2, tanh)) for _ in 1:length(obs_list)]

path_to_fig = joinpath(PROJECT_ROOT, "figs", "mobs")
mkpath(path_to_fig)

println("Now start the simulation")
solver_logs = NamedTuple[]
try
for k in 1:L
    println("Time instance $k")

    run_obs!(obs_circle, k - 1, T, 2)
    run_obs_topdown!(obs_straight, k - 1, T)

    Ref = [[6(k + i - 1) * T, 2.] for i in 1:H₊]
    Prob = [0.95 for i in 1:H]
    φ = [quantile(Normal(0., 1.), (1 + Prob[i]) / 2) for i in 1:H]

    active_ids = [i for i in eachindex(obs_list) if norm(robo.pose[1:2] - obs_list[i].posn) < robo.R]

    if !isempty(active_ids)
        Pmeans = Matrix{Float64}[]
        Pvars  = Matrix{Float64}[]
        active_obs = obstacle[]

        for i in active_ids
            Pmean, Pvar = predict_obstacle!(robo, obs_list[i], obsGPs[i], mNNs[i], k)
            push!(Pmeans, Pmean)
            push!(Pvars, Pvar)
            push!(active_obs, obs_list[i])
        end

        x, y, θ, v, tα, a = nMPC_avoid_multi(robo, active_obs, Ref, Pmeans, Pvars, φ; step=k, solver_log=solver_logs)
        robo.predPose = [x'; y'; θ'; v']

        fig = plot_robot_mobs(robo, x, y, k)
        for i in eachindex(obs_list)
            idx = findfirst(isequal(i), active_ids)
            if idx === nothing
                plot_obs_multi!(obs_list[i])
            else
                plot_obs_multi!(obs_list[i], Pmeans[idx], Pvars[idx], φ)
            end
        end
        display(fig)
        savefig(fig, joinpath(path_to_fig, "$k.pdf"))
        run!(robo, [tα[1], a[1]])
    else
        x, y, θ, v, tα, a = nMPC_free(robo, Ref; step=k, solver_log=solver_logs)
        robo.predPose = [x'; y'; θ'; v']

        fig = plot_robot_mobs(robo, x, y, k)
        for obs in obs_list
            plot_obs_multi!(obs)
        end
        display(fig)
        savefig(fig, joinpath(path_to_fig, "$k.pdf"))
        run!(robo, [tα[1], a[1]])
    end
end
finally
    write_solver_log_csv(joinpath(path_to_fig, "solver_log.csv"), solver_logs)
end
