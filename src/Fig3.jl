# Sinusoidal-obstacle scenario shown in paper Fig. 3.
include("FigureReproduction.jl")

run_figure((
    number=3, scenario=1, scenario_name="sinusoidal",
    steps=[1, 3, 10, 20, 30, 50], obstacle_initial=[22.0, 0.0],
    xlims=(-10, 35), ylims=(-12, 10), xticks=-10:10:30, yticks=-10:5:10,
    annotation=(-8, -10.5),
    extra_notes=[
        "Uses the repository's sinusoidal velocity: xdot=-5+noise and ydot=5*sin(pi*t)+noise. The paper text instead specifies sin(pi*t/4); its published Fig. 3 shows multiple oscillations. The repository frequency is retained, not silently replaced by the conflicting text formula.",
        "Snapshots follow the six labels in the published Fig. 3: 1, 3, 10, 20, 30, 50.",
    ],
))
