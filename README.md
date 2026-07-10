# 机器人与智能系统综合实践 — 实验一：机械臂运动控制仿真

以中国空间站太空机械臂（缩小版，7 自由度对称构型）为对象，在 CoppeliaSim 中
实现机械臂在空间站上行走 2 步的运动控制仿真。

## 目录结构

- `scenes/SpaceRobot.ttt` — CoppeliaSim 仿真场景
- `src/kinematics.py` — 链式(chain)正运动学、几何雅可比、逆运动学（矢状面对称拱形解析解 + 阻尼最小二乘数值解）
- `src/trajectory.py` — 分段五次多项式轨迹规划（速度、加速度连续）
- `src/walk.py` — 行走主程序（步 1：左足 → 位置 1 顶部落足盘；步 2：右足 → 位置 2 侧面落足盘）
- `src/chain_constants.py` — 从场景零位测得的链路常值变换
- `report/` — 实验报告（docx，OpenXML/python-docx 生成）、生成脚本与图表
- `docs/` — 实验要求截图
- `example.py` — 课程提供的 ZMQ Remote API 示例

## 运行方法

```bash
pip install coppeliasim-zmqremoteapi-client numpy matplotlib
# 启动 CoppeliaSim 并打开 scenes/SpaceRobot.ttt，然后：
cd src && python3 walk.py
```

运行结束后关节曲线数据保存在 `report/figures/walk_log.npz`。

## 重新生成实验报告

```bash
pip install python-docx
cd report && python3 fill_report.py
```
