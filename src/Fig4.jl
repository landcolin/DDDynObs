include("environment.jl")
include("objects.jl")
include("computing.jl")
include("MPC.jl")
include("mPlots.jl")
include("MultiObstacle.jl")
include("FigureReproduction.jl")

run_multi_obstacle_reproduction(
    path_to_fig     = joinpath(PROJECT_ROOT, "output", "fig4"),
    combined_prefix = joinpath(PROJECT_ROOT, "output", "fig4", "fig4"),
    fig_steps       = [1, 20, 30, 40, 50, 60],
    title           = "Fig. 4 Reproduction (Two Obstacles: Circle + Straight)",
    L               = 60,
    H               = 8,
    H₊              = 8,
    b               = [2.5, 2.5, 1.0, 1.0],
    safe_weight     = 10.0,
    seed            = 1234,
    blas_threads    = 1,
    plot_only       = ("--plot-only" in ARGS),
    tick_fontsize   = 16,
    grid_size       = (2400, 1200)
)
