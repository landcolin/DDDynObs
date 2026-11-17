import Pkg
Pkg.activate(@__DIR__)

# Configure headless plotting before loading the GR backend.
get!(ENV, "GKSwstype", "nul")

using Optim, Plots, JuMP, GaussianProcesses, LinearAlgebra, OrdinaryDiffEq, Flux, Random, Ipopt, Distributions

# Resolve ambiguity between GaussianProcesses and the locked PDMats versions.
LinearAlgebra.ldiv!(A::GaussianProcesses.PDMats.PDMat, B::LinearAlgebra.AbstractVecOrMat) =
    LinearAlgebra.ldiv!(A.chol, B)
LinearAlgebra.ldiv!(A::GaussianProcesses.ElasticPDMats.ElasticPDMat, B::LinearAlgebra.AbstractVecOrMat) =
    LinearAlgebra.ldiv!(A.chol, B)

include("objects.jl")
include("computing.jl")
include("MPC.jl")
include("mPlots.jl")
