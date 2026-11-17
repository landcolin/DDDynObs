# DDDynObs

Dynamic obstacle avoidance simulations in Julia (LEAP-O).

## Reference

[![LEAP-O diagram](figs/LEAP-O_dia.png)](figs/LEAP-O_dia.pdf)

## Setup & Environment Check

This project uses **Julia 1.12.1** and the locked environment defined in [`src/Project.toml`](src/Project.toml) and [`src/Manifest.toml`](src/Manifest.toml).

From the repository root (PowerShell):

```powershell
.\scripts\julia.ps1 setup
.\scripts\julia.ps1 check
```

If PowerShell script execution is restricted by system policy, run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 check
```

Alternatively, using `julia` directly:

```bash
julia --project=src -e 'using Pkg; Pkg.instantiate()'
```

## Run Simulations

Run the default single-obstacle simulation (`src/Sim.jl`):

```powershell
.\scripts\julia.ps1 single
```

Run the default multiple-obstacle simulation (`src/Sim_mobs.jl`):

```powershell
.\scripts\julia.ps1 multi
```

## Reproduce Paper Figures (Fig. 2 – Fig. 4)

Run the dedicated figure reproduction entry points:

```powershell
.\scripts\julia.ps1 fig2
.\scripts\julia.ps1 fig3
.\scripts\julia.ps1 fig4
```

To re-plot figures directly from saved snapshots (`snapshot.jls`) without re-running MPC or GP/RNN training:

```powershell
.\scripts\julia.ps1 fig2 --plot-only
.\scripts\julia.ps1 fig3 --plot-only
.\scripts\julia.ps1 fig4 --plot-only
```

## Output

- Default simulations write step PDFs and `solver_log.csv` to `figs/sinusoid/`, `figs/circle/`, or `figs/mobs/`.
- Paper figure entries write step PDFs, combined 2×3 grids (`fig2.{pdf,png}`, `fig3.{pdf,png}`, `fig4.{pdf,png}`), `trajectory.csv`, `solver_log.csv`, `snapshot.jls`, and `README_repro.md` to `output/fig2/`, `output/fig3/`, and `output/fig4/`.
