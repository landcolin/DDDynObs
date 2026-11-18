dim = 2
T   = 0.1
H   = 8
H₊  = 8
L   = 70
ns  = 3


include("environment.jl")

path_to_fig = normpath(joinpath(@__DIR__, "..", "figs", "mobs"))
mkpath(path_to_fig)


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

    for i in 1:H
        δ₁ = φ[i] * sqrt(Pvar[1, i])
        δ₂ = φ[i] * sqrt(Pvar[2, i])
        rec = Shape([Pmean[1, i] + δ₁, Pmean[1, i] + δ₁, Pmean[1, i] - δ₁, Pmean[1, i] - δ₁],
                    [Pmean[2, i] + δ₂, Pmean[2, i] - δ₂, Pmean[2, i] - δ₂, Pmean[2, i] + δ₂])
        plot!(rec, fillcolor = plot_color(:orange, 0.4), label = "")
    end
    plot!([obs.posn[1]; Pmean[1, :]], [obs.posn[2]; Pmean[2, :]], label = "", linewidth = 2, linecolor = :orange)
end


function plot_robot_mobs(robo::robot, x::Vector{Float64}, y::Vector{Float64}, k::Int64)
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
    annotate!(15, 20, text("Step $k", :red, :right, 20))

    plot!([p[1]; x], [p[2]; y], label = "", linewidth = 2, linecolor = :green, xtickfontsize = 20, ytickfontsize = 20)
    plot!(robo.traj[1, :], robo.traj[2, :], label = "", linewidth = 2, linecolor = :red)
    return fig
end


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

mNNs = [DisplacementRNN() for _ in obs_list]
circle_rng, vertical_rng = MersenneTwister(1234), MersenneTwister(42)


println("Now start the simulation")
for k in 1:L
    println("Time instance $k")

    Ref = [[6(k + i - 1) * T, 2.] for i in 1:H₊]
    Prob = [0.95 for _ in 1:H]
    φ = rectangle_quantile.(Prob)

    active_ids = [i for i in eachindex(obs_list) if norm(robo.pose[1:2] - obs_list[i].posn) < robo.R]

    if !isempty(active_ids)
        Pmeans = Matrix{Float64}[]
        Pvars  = Matrix{Float64}[]
        active_obs = obstacle[]

        for i in active_ids
            Pmean, Pvar = predict_obstacle!(robo, obs_list[i], obsGPs[i], mNNs[i], k - 1)
            push!(Pmeans, Pmean)
            push!(Pvars, Pvar)
            push!(active_obs, obs_list[i])
        end

        x, y, θ, v, tα, a = nMPC_avoid_multi(robo, active_obs, Ref, Pmeans, Pvars, φ)
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
    else
        x, y, θ, v, tα, a = nMPC_free(robo, Ref)
        robo.predPose = [x'; y'; θ'; v']

        fig = plot_robot_mobs(robo, x, y, k)
        for obs in obs_list
            plot_obs_multi!(obs)
        end
        display(fig)
        savefig(fig, joinpath(path_to_fig, "$k.pdf"))
    end
    run!(robo, [tα[1], a[1]])
    run_obs!(obs_circle, k - 1, T, 2; rng=circle_rng)
    run_obs_topdown!(obs_straight, k - 1, T; rng=vertical_rng)
end
