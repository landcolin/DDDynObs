using Pkg

println("Instantiating the locked environment with Julia ", VERSION)
flush(stdout)
Pkg.instantiate(; julia_version_strict=true)
Pkg.precompile(; strict=true)
Pkg.status()
println("Setup complete. Run scripts/check_environment.jl to verify the runtime.")
