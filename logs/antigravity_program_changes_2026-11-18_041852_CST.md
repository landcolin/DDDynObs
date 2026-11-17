# DDDynObs 程序修改汇总

| 记录信息 | 内容 |
| --- | --- |
| 记录生成时间 | 2026-11-18 04:18:52 +08:00 |
| 时区 | Asia/Shanghai（北京时间） |
| 编码 | UTF-8 |
| 项目目录 | `C:\Users\lu_ke\Documents\GitHub\DDDynObs` |
| 基准 HEAD | `2aaa67450e3a60d004018ab0659e07bc800c4ad4`（`2aaa674 Update README.md`） |
| 比较范围 | 原始仓库基准提交 `2aaa674` 与当前工作区全部改动（含前期环境准备与本轮代码重构/验证） |
| 记录性质 | 对当前工作区最终状态及已完成验证的回顾性汇总 |

---

## 1. 环境配置与启动方式

- 保留原有 **Julia 1.12.1**（工作区便携环境 `.julia-local/julia-1.12.1/` 与 `.julia-local/depot/`）以及锁定的 `src/Project.toml`、`src/Manifest.toml`，没有升级依赖。
- 新增 `AGENTS.md`，记录本地 Julia 1.12.1 便携环境的启动方式与默认回归核对标准。
- 新增 `scripts/setup.jl`，用于激活 `src/`、执行 `Pkg.instantiate()` 及 `Pkg.precompile(strict = true)`。
- 新增 `scripts/check_environment.jl`，校验 Julia 版本（`1.12.1`）、无头绘图模式及 `Ipopt` / `OrdinaryDiffEq` / `GaussianProcesses` / `Flux` / `Plots` 基础功能。
- 新增 `scripts/julia.ps1`，自动定位 Julia `1.12.1`（优先 `.julia-local`，并显式绕过不可直接调用的 `WindowsApps` 别名），支持 `setup`、`check`、`test`、`single`、`multi`、`fig2`、`fig3`、`fig4` 及自定义 `.jl` 路径。
- 新增 `src/environment.jl`，集中完成项目激活、`LOAD_PATH` 设置、在加载 `Plots` 前强制设置无头 GR（`ENV["GKSwstype"] = "nul"`）、公共依赖导入，以及保留原有的 `GaussianProcesses` / `PDMats` `LinearAlgebra.ldiv!` 方法消歧补丁。

---

## 2. MPC 实现修正

对照 `src/objects.jl` 中的实际机器人 ODE 模型（`robot_ode!`：`dx[3] = x[4]*u[1]/ℓ`）及论文第 5 页式 (20)，修正了原有 MPC 动力学模型与初值索引错误：

- 自由运动（`nMPC_free`）、单障碍物（`nMPC_avoid`）和多障碍物（`nMPC_avoid_multi`）的航向角更新全部补上速度因子：

  修改前：
  ```text
  θ[1] == state[3] + τ * tα[1] / ℓ
  θ[i] == θ[i-1] + τ * tα[i] / ℓ
  ```

  修改后：
  ```text
  θ[1] == state[3] + τ * state[4] * tα[1] / ℓ
  θ[i] == θ[i-1] + τ * v[i-1] * tα[i] / ℓ
  ```

- 单障碍物 MPC（`src/MPC.jl` 中 `nMPC_avoid`）的速度变量初值由 `robo.predPose[3, :]`（航向角行）修正为 `robo.predPose[4, :]`（速度行）。
- 主要涉及文件：`src/MPC.jl`、`src/MultiObstacle.jl`。

---

## 3. 求解检查、对偶初始化与多初值重试

- **严格可行性拦截（`ensure_solved`）**：在读取 `JuMP.value.` 与应用控制量之前，先检查 `JuMP.is_solved_and_feasible(opti) && JuMP.has_values(opti)`；不可行时立即抛错停止，避免继续使用不可行解生成伪合法轨迹。
- **求解诊断记录**：每步记录 `step`、`solver`、`attempts`（以及轨迹 CSV 中的 `solver_attempts` 列）、`termination_status`、`primal_status`、`feasible`、`has_values`、`solve_time_s`、`wall_time_s`，并通过 `try ... finally` 确保即使中途报错停止也能将已产生记录写入 `solver_log.csv`（及已完成步的 `trajectory.csv`）。
- **几何分离方向对偶初始化（`init_separating_duals`）**：基于预测机器人位姿与障碍物预测中心间的单位分离方向，解析构造满足 $\|E^\top \mu\|_2 < 1$ 与 $\lambda^\top A + \mu^\top E R(\theta) = 0$ 的非负 `λ₀`、`μ₀` 及 `safe₀` 初值，在单障碍与多障碍避障 MPC 中替代原先隐式的全零初值。
- **多初值重试（`rollout_initial_guess` / `candidate_initial_guesses` / `solve_with_retries`）**：首次默认热启动求解失败后，按与 MPC 一致的离散前向欧拉运动学生成最多 **5 组**备用转向与制动轨迹初值（左转保速、右转保速、直行制动、左转+制动、右转+制动），并同步重算对应的几何分离对偶初值；重试过程严格保留同一目标函数与全部硬约束。
- 多初值重试仍属局部求解增强策略，不保证一定找到可行解，也不保证全局最优；全部 6 次尝试均不可行时依然报错停止。

---

## 4. 公共功能整理

- 新增 `src/MultiObstacle.jl`，从 `src/Sim_mobs.jl` 提取 `run_obs_topdown!`、`predict_obstacle!`、`nMPC_avoid_multi`、`plot_obs_multi!`、`plot_robot_mobs`，由 `src/Sim_mobs.jl` 与 `src/Fig4.jl` 共用，消除重复代码。
- 消除多处隐式全局变量依赖：`predict_obstacle!` 改为从 `robo` 与 `obs` 读取 `dim`、`ns`、`H`、`T`；`plot_obs`（`src/mPlots.jl`）与 `plot_obs_multi!`（`src/MultiObstacle.jl`）改为按 `size(Pmean, 2)` 循环。
- 新增 `src/FigureReproduction.jl`，统一单/多障碍论文复现仿真循环、最小采样间距计算（`robot_obstacle_clearance`）、`trajectory.csv` / `solver_log.csv` 导出、每步帧快照（`snapshot.jls`）、复现说明（`README_repro.md`）及六图拼合输出。
- `src/computing.jl` 保留原有 `Flux.setup(Flux.Adam(1e-2), mNN)` 兼容性修改，其余训练与预测逻辑保持不变。

---

## 5. 新增论文图形运行入口

| 入口 | 场景 | 运行步数 | 取图时刻 | 输出目录 |
| --- | --- | ---: | --- | --- |
| `src/Fig2.jl` | 圆周单障碍物 | 60 | 1、10、20、30、40、60 | `output/fig2/` |
| `src/Fig3.jl` | 正弦单障碍物 | 50 | 1、3、10、20、30、50 | `output/fig3/` |
| `src/Fig4.jl` | 圆周 + 向下直线双障碍物 | 60 | 1、20、30、40、50、60 | `output/fig4/` |

- 各入口均固定网络初始化随机种子为 **`1234`**，并将 BLAS 线程数设为单线程（`BLAS.set_num_threads(1)`）。
- 输出包括单步 PDF、六幅组合图（`fig2.{pdf,png}`、`fig3.{pdf,png}`、`fig4.{pdf,png}`）、含 `solver_attempts` 的 `trajectory.csv`、`solver_log.csv`、`snapshot.jls` 快照以及 `README_repro.md`。
- 支持 `--plot-only` 参数，直接从 `snapshot.jls` 重绘单步 PDF、组合图与说明文件，无需重新运行仿真或训练（快照仅支持重绘，不是完整仿真的断点续跑）。

---

## 6. 新图入口参数调整与原有入口保留

| 参数 | 原有仿真入口（`Sim.jl` / `Sim_mobs.jl`） | 新图入口（`Fig2.jl` / `Fig3.jl` / `Fig4.jl`） |
| --- | --- | --- |
| 预测时域 `H` | 单障碍物 6；多障碍物 8 | 均为 **8** |
| 车身半长 `b[1:2]` | `2.3 m` | **`2.5 m`** |
| 多障碍物安全余量权重 | 默认 **`1.0`** | `Fig4.jl` 显式使用 **`10.0`** |
| 网络初始化随机种子 | 未统一固定 | **`1234`** |

- 原有 `src/Sim.jl`、`src/Sim_mobs.jl` 及其默认参数（`b = 2.3 m`、多障碍默认安全权重 `1.0`）保持不变。
- `Fig4.jl` 将安全余量权重从 `1.0` 调整为 `10.0` 属于控制目标参数变化（并非单纯初值修正），使其与 `Fig2.jl` / `Fig3.jl` 保持一致。
- 采样时间 `0.1 s`、初始速度 `6.0 m/s` 及原有速度/加速度边界保持不变。

---

## 7. 输出路径、图形排版与仓库规则

- 修复 `src/Sim.jl` 原代码保存图片时使用未定义变量 `path` 的问题，统一改为 `joinpath(path_to_fig, "$k.pdf")`，并通过 `PROJECT_ROOT` 相对项目根目录定位、使用 `mkpath` 自动创建目录；`src/Sim_mobs.jl` 同步改用 `joinpath(PROJECT_ROOT, "figs", "mobs")` 与 `mkpath`。
- 在 `src/mPlots.jl`、`src/MultiObstacle.jl` 与 `src/FigureReproduction.jl` 中新增 `Times` 字体设置、ASCII 负号刻度格式化（`format_ascii_tick`，避免部分 PDF 阅读器把 Unicode `−` 显示异常）、六图边距微调、可关闭的步数标注开关 `show_step`（仅保留图内单一标注，无重复标题）。
- 更新 `README.md`，补充 `scripts/julia.ps1` 的环境检查、默认仿真及 Fig. 2～4（含 `--plot-only`）命令说明。
- 更新 `.gitignore`，加入 `.julia-local/`、`figs/*/solver_log.csv`、`figs/*/trajectory.csv`、`output/fig2/`、`output/fig3/`、`output/fig4/` 忽略规则。

---

## 8. 已完成的验证与实测结果

- **环境检查（`.\scripts\julia.ps1 check`）**：退出码 `0`，输出 `Environment check PASSED.`，验证了 Julia `1.12.1`、全部依赖加载以及 Ipopt / OrdinaryDiffEq / GaussianProcesses / Flux / Plots 基础连通性。（注：`scripts/setup.jl` 已编写但因 `.julia-local` 已预先完成离线安装未单独重跑。）
- **专项一致性与恢复测试（`.\scripts\julia.ps1 test`，`scripts/test_section8.jl`）**：**13/13 断言全部通过**（耗时 `35.0s`）：
  - 常速 ODE 与前向欧拉航向一致性误差 `< 1e-12`；
  - 单障碍 `nMPC_avoid` 与双障碍 `nMPC_avoid_multi` 转向预测与 `Tsit5()` ODE 积分实测 `|θ_mpc - θ_ode| ≈ 0.003601 rad`（严格符合线性加减速二阶积分截断界）；
  - 物理不可行阻塞用例尝试 6 次后严格报错停止；人为制造首组失败初值时，多初值重试在第 2 次恢复 `LOCALLY_SOLVED`。
- **原默认单障碍入口回归（`.\scripts\julia.ps1 single`，`src/Sim.jl`）**：在完成第 1～4 节修改后实测 60 步全部可行通过（退出码 `0`），生成 `figs/sinusoid/1.pdf`～`60.pdf`。
- **论文图形全流程生成（`.\scripts\julia.ps1 fig2 / fig3 / fig4`）**：三组场景全部可行跑通（退出码 `0`），产物位于 `output/fig2/`、`output/fig3/`、`output/fig4/`：

| 图形 | 完整运行步数 | 最大尝试次数 | 最小采样间距 | 生成 PDF 数量 |
| --- | ---: | ---: | ---: | ---: |
| Fig. 2 | 60 / 60 | 1 | `2.74483 m` | 61（60 单步 + `fig2.pdf`） |
| Fig. 3 | 50 / 50 | 1 | `3.567993 m` | 51（50 单步 + `fig3.pdf`） |
| Fig. 4 | 60 / 60 | 1 | `2.757795 m` | 61（60 单步 + `fig4.pdf`） |

- **补充观测与复现限制（与第 9 节一致）**：
  - 保留 `src/computing.jl`（GP/RNN 训练与预测主体）及 `src/objects.jl`（机器人及障碍物运动主体），保留每步重设固定噪声种子（`1234` / `42`）与 `sin(π*t)` 频率；
  - `Fig4.jl` 第 60 步最终位置 `y ≈ 8.50693 m`（距参考线 `y = 2` 约 `6.51 m`），与论文原图穿行回到参考线的轨迹存在明显差异，属于场景复现而非完全数值复现；
  - 采样点处的正最小间距不构成连续时间无碰撞证明；
  - 保留原安全余量权重 `1.0` 的 `src/Sim_mobs.jl`（`.\scripts\julia.ps1 multi`）在实测中第 23、25 步通过多初值重试恢复可行解，第 26 步 6 次尝试均不可行后严格停止。

---

## 9. 当前工作区文件变更清单（相对基准 `2aaa674`）

- **已修改的追踪文件**：`.gitignore`、`README.md`、`src/MPC.jl`、`src/Sim.jl`、`src/Sim_mobs.jl`、`src/computing.jl`、`src/mPlots.jl`
- **新增文件/目录**：`AGENTS.md`、`scripts/setup.jl`、`scripts/check_environment.jl`、`scripts/test_section8.jl`、`scripts/julia.ps1`、`src/environment.jl`、`src/MultiObstacle.jl`、`src/FigureReproduction.jl`、`src/Fig2.jl`、`src/Fig3.jl`、`src/Fig4.jl`、`logs/`（含本日志）

---

## 10. 常用命令

在仓库根目录执行（若系统 PowerShell 策略限制未签名脚本，按 `README.md` 加 `-ExecutionPolicy Bypass`）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 setup
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 check
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 test
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 single
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 multi
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 fig2
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 fig3
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 fig4
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\julia.ps1 fig4 --plot-only
```
