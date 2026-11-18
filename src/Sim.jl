dim = 2
T   = 0.1
H   = 6
H₊  = 8
L   = 60
ns  = 3


include("environment.jl")

color =  cgrad(:turbo, 2, categorical = true, scale = :lin)

R = 20.
ℓ = 1.9
str_ang =  0.6
v_min   = -10.
v_max   =  20.
a_min   = -2.
a_max   =  2.
pBounds = polyBound(str_ang, v_min, v_max, a_min, a_max)


init  = [0., 2., 0., 6.]
A     = [1. 0; -1. 0.; 0. 1.; 0. -1.]
b     = [2.3, 2.3, 1., 1.]
TypeOfObs = 1

test_points = [10, 20, 30, 40, 50]
obs_traj = zeros(2,L)
pre_traj = fill(NaN, 2, H, L)
mse_pre  = zeros(length(test_points))


path_to_fig = ""
if TypeOfObs == 1 #Sinusoid move
    obs   = obstacle(2., [22., 0.], ns)
    path_to_fig = "figs/sinusoid/"
elseif TypeOfObs == 2 #Circle move
    obs   = obstacle(2., [18., -10.], ns)
    path_to_fig = "figs/circle/"
else #Straight move
    obs   = obstacle(2., [15., -15.], ns)
    path_to_fig = "figs/straight/"
end
path_to_fig = normpath(joinpath(@__DIR__, "..", path_to_fig))
mkpath(path_to_fig)

robo  = robot(T, H, H₊, R, ℓ, pBounds, init, A, b)

obsGP    = Vector{GPBase}(undef, 2) #MeanPoly(ones(1,10))
obsGP[1] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
obsGP[2] = GPE(mean = MeanConst(1.), kernel = SEArd([1.], 1.), logNoise = -2.)
mNN      = DisplacementRNN()
motion_rng = MersenneTwister(1234)


# Run simulation
println("Now start the simulation")
for k in 1:L
    println("Time instance $k")

    Ref  = [[6(k+i-1)*T, 2.] for i in 1:H₊]

    if norm(robo.pose[1:2] - obs.posn) < robo.R
        Pmean, Pvar = predict_obstacle!(robo, obs, obsGP, mNN, k - 1)

        pre_traj[:,:,k] = Pmean[:, 1:H]

        φ = fill(rectangle_quantile(0.95), forecast_horizon(robo))

        x, y, θ, v, tα, a = nMPC_avoid(robo, obs, Ref, Pmean, Pvar, φ)
        robo.predPose = [x';y';θ';v']
        fig = plot_robot(robo, x, y, TypeOfObs,k)
        plot_obs(obs, Pmean, Pvar, φ)
        display(fig)
        savefig(fig, joinpath(path_to_fig, "$k.pdf"))
    else
        x, y, θ, v, tα, a = nMPC_free(robo, Ref)
        robo.predPose = [x';y';θ';v']
        fig = plot_robot(robo, x, y, TypeOfObs,k)
        plot_obs(obs)
        display(fig)
        savefig(fig, joinpath(path_to_fig, "$k.pdf"))
    end
    run!(robo, [tα[1], a[1]])
    run_obs!(obs, k - 1, T, TypeOfObs; rng=motion_rng)
    obs_traj[:, k] = obs.posn
end


for (i, v) in enumerate(test_points)
    print(v)
    mse = sqrt(1/H*sum(norm(obs_traj[:,v+h-1] - pre_traj[:,h,v])^2 for h in 1:H))
    mse_pre[i] = mse
end
