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

Generated figures are written to `figs/`.

Single-obstacle runs use `figs/sinusoid/`, `figs/circle/`, or `figs/straight/`; multiple-obstacle runs use `figs/mobs/`. These directories are created automatically, relative to the repository location. Plotting uses headless GR by default. Simulations run on the CPU and do not require CUDA.
