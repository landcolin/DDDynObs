# DDDynObs 仿真运行指南

在 Google Antigravity 中打开 DDDynObs 仓库根目录，使用 Local 模式和 Windows PowerShell。当前工作区的 `.julia-local` 包含 Julia 1.12.1 便携版和已安装的依赖。该目录被 Git 忽略，新的克隆或 worktree 需要使用现有本机环境的路径，或按 [README](README.md) 中的说明准备自己的环境。

## 运行仿真

从仓库根目录执行以下代码。将 `$entry` 设为本次测试所需的入口。

```powershell
$entry = 'src/Sim.jl'       # 单障碍仿真
# $entry = 'src/Sim_mobs.jl' # 多障碍仿真

$julia = Join-Path $PWD.Path '.julia-local/julia-1.12.1/bin/julia.exe'
$env:JULIA_DEPOT_PATH = Join-Path $PWD.Path '.julia-local/depot'
$env:JULIA_PKG_PRECOMPILE_AUTO = '0'
$env:GKSwstype = 'nul'

if (!(Test-Path -LiteralPath $julia -PathType Leaf)) {
    throw "Julia 1.12.1 was not found at $julia. Check the workspace root and local installation."
}

& $julia --startup-file=no --pkgimages=no --compiled-modules=existing --project=src $entry
if ($LASTEXITCODE -ne 0) {
    throw "Simulation failed with Julia exit code $LASTEXITCODE. Check the terminal output."
}
```

`--project=src` 使用 `src/Project.toml` 和 `src/Manifest.toml` 定义的环境。`GKSwstype` 让 GR 在无图形窗口的情况下输出文件。首次启动可能需要几分钟加载依赖和编译代码。

## 检查结果

| 默认入口 | 完成步数 | 应生成的 PDF |
| --- | ---: | --- |
| `src/Sim.jl` | 60 | `figs/sinusoid/1.pdf` 至 `figs/sinusoid/60.pdf` |
| `src/Sim_mobs.jl` | 70 | `figs/mobs/1.pdf` 至 `figs/mobs/70.pdf` |

等待 Julia 进程退出，检查退出码、终端中的最后一个 `Time instance` 和本次生成的 PDF。脚本会覆盖同名 PDF，验证时用文件更新时间区分已有输出。若测试调整了步数 `L` 或单障碍类型 `TypeOfObs`，按此次配置核对输出。
