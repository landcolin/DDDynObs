include("environment.jl")
include("objects.jl")
include("computing.jl")
include("MPC.jl")
include("mPlots.jl")
include("FigureReproduction.jl")

run_single_obstacle_reproduction(
    TypeOfObs       = 2,
    obs_init        = [18., -10.],
    path_to_fig     = joinpath(PROJECT_ROOT, "output", "fig2"),
    combined_prefix = joinpath(PROJECT_ROOT, "output", "fig2", "fig2"),
    fig_steps       = [1, 10, 20, 30, 40, 60],
    title           = "Fig. 2 Reproduction (Circle Obstacle)",
    L               = 60,
    H               = 8,
    H₊              = 8,
    b               = [2.5, 2.5, 1.0, 1.0],
    seed            = 1234,
    blas_threads    = 1,
    plot_only       = ("--plot-only" in ARGS),
    grid_size       = (3000, 1000)
)
