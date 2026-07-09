# robot_control — 7-DOF 空间机械臂控制程序

基于 CoppeliaSim 仿真环境，实现七自由度空间机械臂的三步行走运动。

## 文件结构说明

### 核心算法（不依赖仿真器）

| 文件 | 作用 |
|------|------|
| kinematics_chain.py | **正运动学（FK）**。k(q) 计算给定7个关节角下 R_Base 在 L_Base 坐标系中的位姿（4x4矩阵），匹配 SpaceRobot.ttt 场景的关节链参数。 |
| ik_solver_chain.py | **逆运动学求解器（DLS）**。实现 Damped Least Squares 算法 + 舱体碰撞检测/避免 + 多初值随机尝试。 |
| kinematics_dh.py | 旧版 DH 法正运动学（MATLAB 迁移，未使用）。 |
| ik_solver.py | 旧版逆运动学求解器（MATLAB 迁移，未使用）。 |

### 轨迹规划与路径点

| 文件 | 作用 |
|------|------|
| scene_poses.py | **路径点定义**。三步行走各阶段起点/终点/中间点坐标，含五次样条插值函数 path_quintic_arclength()。 |
| 	rajectory.py | 旧版轨迹规划（MATLAB 迁移，未使用）。 |

### 仿真控制

| 文件 | 作用 |
|------|------|
| coppeliasim_control.py | **CoppeliaSim 连接封装**。CoppeliaSimController 提供 get/set 位姿、关节角、仿真启停等接口，底层通过 ZeroMQ Remote API 通信。 |

### 主控逻辑

| 文件 | 作用 |
|------|------|
| walk_main.py | **核心算法库**（约17KB）。DLS IK、碰撞避免、路径求解、三步走完整轨迹计算 compute_full_trajectory()、轨迹保存/加载、离线验证。 |
| walk_solver.py | **离线求解器入口**。调用 save_trajectory() 求解轨迹并保存为 .npz，**只计算不连仿真器**。 |
| walk_runner.py | **轨迹播放器入口**。加载 .npz 中的 q1, q2, traj_home，连接 CoppeliaSim 逐帧播放，**只播放不计算**。 |
| main.py | **统一入口**。整合四种模式：--verify（IK验证）、--solve（离线求解）、--run <file>（播放）、--walk（计算+仿真同时进行）。 |

### 辅助工具

| 文件 | 作用 |
|------|------|
| plot_trajectory.py | 轨迹可视化，绘制关节角-时间曲线。 |
| __init__.py | 包初始化。 |

---

## 三步走流程

`
第一步：B端固定（R_Base），A端（L_Base）从起点 (0.65, 0, 0.235) 
        沿弧线移动到顶部垫片 (-0.25, 0, 0.235)

第二步：A端固定（L_Base），B端（R_Base）从起点 (0.35, 0, 0.235)
        沿高弧线移动到侧面垫片 (-0.65, 0.133, 0.101)

折叠：  B端固定在侧面垫片上，机械臂从行走结束姿态
        平滑过渡到全零关节角（joint 7 = 360deg）
`

---

## 如何链接仿真软件进行实际仿真

### 环境要求

1. **CoppeliaSim** 已安装并打开
2. 加载场景文件 scenes/SpaceRobot.ttt
3. 确认 **ZeroMQ Remote API** 服务已启用（默认端口 23000）

### 推荐工作流（计算与控制分离）

`ash
# 步骤1：离线求解轨迹（只需一次）
python walk_solver.py --speed 0.9
# → 生成 walk_traj.npz（约5-15秒）

# 步骤2：打开 CoppeliaSim，加载 SpaceRobot.ttt

# 步骤3：播放轨迹
python walk_runner.py --traj walk_traj.npz [--speed 1.0] [--hold 12.0]
`

### 常用命令

| 目的 | 命令 |
|------|------|
| 仅IK验证（不连仿真器） | python main.py --verify |
| 离线求解+保存 | python main.py --solve 或 python walk_solver.py |
| 从文件播放 | python main.py --run walk_traj.npz 或 python walk_runner.py --traj walk_traj.npz |
| 一步到位（计算+仿真） | python main.py --walk |

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| --speed | 0.9 | 速度倍率（0.9 ≈ 原始3倍速） |
| --h | 0.30 | 抬升高度（m） |
| --tilt | 0.15 | 拔出时倾斜角（rad） |
| --k1 / --k2 | 0 | Z轴旋转系数 |
| --hold | 12.0 | 结束后停留秒数（仅 walk_runner） |
| --traj | walk_traj.npz | 轨迹文件路径（仅 walk_runner） |
| --host / --port | localhost:23000 | 仿真器地址 |

---

## 计算-控制分离架构

`
┌─────────────────┐        ┌──────────────────┐
│  walk_solver.py │ ─────→ │  walk_traj.npz   │ ─────→ ┌──────────────────┐
│  离线 IK 求解    │  保存  │  (q1, q2, home)  │  读取  │  walk_runner.py  │ ─────→ CoppeliaSim
│  无需仿真器      │        │                  │        │  逐帧播放轨迹     │
└─────────────────┘        └──────────────────┘        └──────────────────┘
`

- **求解一次，可反复播放**，无需重复计算 IK
- **求解无需 CoppeliaSim**，可在无仿真器环境下调试
- **播放时无 IK 计算**，仅设置关节角和步进仿真，速度快
- **求解时输出详细诊断**（间隙、连续性、位置精度）

---

## 关键参数速查

| 参数 | 值 |
|------|-----|
| 舱体圆柱半径 | 0.134 m |
| 舱体圆柱中心 Z | 0.101 m |
| 碰撞激活阈值 | 0.194 m |
| 安全裕度 | 0.005 m |
| DLS 阻尼 lambda | 0.3（自适应） |
| 零空间偏置 k_null | 0.03 |
| Fold 阶段迭代数 | 50 |
| Fold 阶段帧数 | 600 |
| 默认每段帧数 | ~278（speed=0.9） |
