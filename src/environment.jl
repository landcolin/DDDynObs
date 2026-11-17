const SRC_DIR      = @__DIR__
const PROJECT_ROOT = dirname(@__DIR__)

if !(SRC_DIR in LOAD_PATH)
    push!(LOAD_PATH, SRC_DIR)
end

# Configure headless GR before loading Plots.
ENV["GKSwstype"] = "nul"

import Pkg
using Pkg
Pkg.activate(SRC_DIR)

using Optim, Plots, JuMP, GaussianProcesses, LinearAlgebra, OrdinaryDiffEq, Flux, Random, Ipopt, Distributions

# Existing compatibility patch: resolve method ambiguity between GaussianProcesses and PDMats.
LinearAlgebra.ldiv!(A::GaussianProcesses.PDMats.PDMat, B::LinearAlgebra.AbstractVecOrMat) =
    LinearAlgebra.ldiv!(A.chol, B)
LinearAlgebra.ldiv!(A::GaussianProcesses.ElasticPDMats.ElasticPDMat, B::LinearAlgebra.AbstractVecOrMat) =
    LinearAlgebra.ldiv!(A.chol, B)
