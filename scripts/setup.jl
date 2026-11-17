import Pkg
using Pkg

const REPO_ROOT = dirname(@__DIR__)
const SRC_DIR   = joinpath(REPO_ROOT, "src")

println("Activating locked environment at: ", SRC_DIR)
Pkg.activate(SRC_DIR)

println("Instantiating dependencies from src/Project.toml and src/Manifest.toml...")
Pkg.instantiate()

println("Running strict precompilation...")
Pkg.precompile(strict = true)

println("Setup completed successfully with Julia ", VERSION)
