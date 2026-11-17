const REPO_ROOT = dirname(@__DIR__)
const SRC_DIR   = joinpath(REPO_ROOT, "src")

println("Julia version: ", VERSION)
if VERSION != v"1.12.1"
    error("Expected Julia 1.12.1, found $(VERSION)")
end

include(joinpath(SRC_DIR, "environment.jl"))

println("PROJECT_ROOT: ", PROJECT_ROOT)
println("JULIA_DEPOT_PATH: ", get(ENV, "JULIA_DEPOT_PATH", "(default)"))
println("GKSwstype: ", get(ENV, "GKSwstype", "(unset)"))

# Quick sanity checks for core packages
opti = JuMP.Model(Ipopt.Optimizer)
set_silent(opti)
@variable(opti, x >= 0.0, start = 0.5)
@objective(opti, Min, (x - 1.0)^2)
JuMP.optimize!(opti)
if !JuMP.is_solved_and_feasible(opti)
    error("Ipopt sanity check failed: $(JuMP.termination_status(opti))")
end

prob = OrdinaryDiffEq.ODEProblem((du, u, p, t) -> (du[1] = -u[1]), [1.0], (0.0, 0.1))
sol  = OrdinaryDiffEq.solve(prob, OrdinaryDiffEq.Tsit5())
if isempty(sol.u)
    error("OrdinaryDiffEq sanity check failed")
end

gp = GaussianProcesses.GPE([0.0 1.0], [0.0, 1.0], GaussianProcesses.MeanConst(0.0), GaussianProcesses.SEArd([0.0], 0.0), -2.0)
GaussianProcesses.predict_f(gp, [0.5;;])

nn = Flux.Chain(Flux.RNN(1 => 4, tanh), Flux.Dense(4 => 2, tanh))
_ = nn(Float32[0.1;;])

p = Plots.plot([0.0, 1.0], [0.0, 1.0], label = "")
tmp_pdf = tempname() * ".pdf"
Plots.savefig(p, tmp_pdf)
rm(tmp_pdf; force = true)

println("Environment check PASSED.")
