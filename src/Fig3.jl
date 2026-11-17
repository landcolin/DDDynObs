include("environment.jl")
include("objects.jl")
include("computing.jl")
include("MPC.jl")
include("mPlots.jl")
include("FigureReproduction.jl")

run_single_obstacle_reproduction(
    TypeOfObs       = 1,
    obs_init        = [22., 0.],
    path_to_fig     = joinpath(PROJECT_ROOT, "output", "fig3"),
    combined_prefix = joinpath(PROJECT_ROOT, "output", "fig3", "fig3"),
    fig_steps       = [1, 3, 10, 20, 30, 50],
    title           = "Fig. 3 Reproduction (Sinusoid Obstacle)",
    L               = 50,
    H               = 8,
    H₊              = 8,
    b               = [2.5, 2.5, 1.0, 1.0],
    seed            = 1234,
    blas_threads    = 1,
    plot_only       = ("--plot-only" in ARGS),
    tick_fontsize   = 18,
    grid_size       = (3000, 1050)
)
