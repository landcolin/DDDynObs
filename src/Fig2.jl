# Circular-obstacle scenario shown in paper Fig. 2.
include("FigureReproduction.jl")

run_figure((
    number=2, scenario=2, scenario_name="circular",
    steps=[1, 10, 20, 30, 40, 60], obstacle_initial=[18.0, -10.0],
    xlims=(-5, 47), ylims=(-13, 13), xticks=0:10:40, yticks=-10:5:10,
    annotation=(-2, -10.5),
    extra_notes=["Panel labels follow the published figure's final Step 60; the discussion text instead mentions step 65."],
))
