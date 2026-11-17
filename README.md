# DDDynObs

Dynamic obstacle avoidance simulations in Julia.

## Reference

[![LEAP-O diagram](figs/LEAP-O_dia.png)](figs/LEAP-O_dia.pdf)

## Setup

This project uses the Julia environment defined in [`src/Project.toml`](src/Project.toml) and locked by [`src/Manifest.toml`](src/Manifest.toml).

Use **Julia 1.12.1**, the version recorded in the manifest. Keep the manifest to reproduce the existing package versions.

### Windows / PowerShell

From the repository root:

```powershell
.\scripts\julia.ps1 setup
.\scripts\julia.ps1 check
```

The launcher locates the matching Julia installation managed by juliaup, including when the WindowsApps execution alias cannot run. For a different installation location, pass `-JuliaExecutable 'C:\path\to\julia.exe'` or set `JULIA_EXE`. The launcher checks the Julia version and activates `src` for every task.

`setup` installs the locked dependencies and precompiles them. `check` verifies dependency loading, JuMP/Ipopt optimization, ODE integration, Gaussian process prediction, Flux training, and headless PDF export. It does not run the full simulations.

### Direct Julia commands

From the repository root:

```bash
julia --startup-file=no --project=src scripts/setup.jl
julia --startup-file=no --project=src scripts/check_environment.jl
```

The first setup/load may take several minutes. See the [Julia Pkg environment documentation](https://pkgdocs.julialang.org/v1/environments/) for how `instantiate` uses the manifest.

## Run

On Windows / PowerShell:

```powershell
.\scripts\julia.ps1 single
.\scripts\julia.ps1 multi
```

Run the single-obstacle simulation:

```bash
julia --project=src src/Sim.jl
```

Run the multiple-obstacle simulation:

```bash
julia --project=src src/Sim_mobs.jl
```

## Output

To generate a six-panel computational reproduction of paper Fig. 2 (circular obstacle):

```powershell
.\scripts\julia.ps1 fig2
```

The run uses the paper's prediction horizon (8) and robot half-length (2.5 m), and saves PDF/PNG figures, individual panels, a trajectory CSV, a Julia checkpoint, and reproduction notes under `output/fig2/`. It checks every MPC solve. The shared free/single-obstacle MPC now includes the speed factor in its yaw dynamics, consistent with the simulator and paper equation (20), and initializes velocity from the velocity row. This is a new simulation; the notes describe remaining discrepancies, so it is not an exact numerical reproduction. Use `julia --project=src src/Fig2.jl --plot-only` to redraw a completed checkpoint.

Generated figures are written to `figs/`.

For Fig. 3 (sinusoidal obstacle), run:

```powershell
.\scripts\julia.ps1 fig3
```

This runs 50 steps and exports snapshots at steps 1, 3, 10, 20, 30, and 50 to `output/fig3/`, in the same formats as Fig. 2. It retains the repository's sinusoidal velocity frequency `sin(pi*t)`; the paper text instead lists `sin(pi*t/4)`. This discrepancy and other reproduction limits are recorded in `reproduction_notes.md`. Re-render with `julia --project=src src/Fig3.jl --plot-only`. Both figure entry points use `src/FigureReproduction.jl`.

Single-obstacle runs use `figs/sinusoid/`, `figs/circle/`, or `figs/straight/`; multiple-obstacle runs use `figs/mobs/`. These directories are created automatically, relative to the repository location. Plotting uses headless GR by default. Simulations run on the CPU and do not require CUDA.

For Fig. 4 (a circular obstacle and a downward-moving obstacle), run:

```powershell
.\scripts\julia.ps1 fig4
```

This runs 60 steps and exports snapshots at steps 1, 20, 30, 40, 50, and 60 to `output/fig4/`. Each obstacle has an independent GP/RNN predictor; the MPC constrains every currently detected obstacle. `src/MultiObstacle.jl` is shared with `Sim_mobs.jl`, including the corrected yaw dynamics and checked solver status. The trajectory CSV includes individual and minimum sampled clearances. Re-render with `julia --project=src src/Fig4.jl --plot-only`.

The multi-obstacle solver initializes separating normals geometrically. If a local solve fails, it retries the same optimization problem with bounded turn/brake rollout initial guesses; `solver_attempts` records the number of guesses. The objective and hard safety constraints are unchanged by these retries.

The Fig. 4 entry point uses safety-margin weight 10, matching Fig. 2/3; the original multi-obstacle script's default remains 1. This parameter adjustment and remaining differences from the paper are documented alongside the output.
