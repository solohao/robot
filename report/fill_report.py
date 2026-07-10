# -*- coding: utf-8 -*-
"""Fill the experiment-1 report template (OpenXML .docx) with python-docx.

Input : template.docx  (converted from the provided .doc template)
Output: 实验1-机械臂运动控制仿真-实验报告.docx
"""

import copy
import os

from docx import Document
from docx.shared import Pt, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, 'figures')


def add_par_after(par, text='', bold=False, size=10.5, align=None, indent=False):
    new = copy.deepcopy(par._p)
    par._p.addnext(new)
    from docx.text.paragraph import Paragraph
    np_ = Paragraph(new, par._parent)
    for r in list(np_.runs):
        r._r.getparent().remove(r._r)
    run = np_.add_run(text)
    run.bold = bold
    run.font.size = Pt(size)
    if align is not None:
        np_.alignment = align
    if indent:
        np_.paragraph_format.first_line_indent = Cm(0.74)
    return np_


def add_pic_after(par, path, width_cm=13.5, caption=None):
    p = add_par_after(par, '', align=WD_ALIGN_PARAGRAPH.CENTER)
    p.add_run().add_picture(path, width=Cm(width_cm))
    last = p
    if caption:
        last = add_par_after(p, caption, size=9, align=WD_ALIGN_PARAGRAPH.CENTER)
    return last


def find_par(doc, key):
    for p in doc.paragraphs:
        if key in p.text:
            return p
    raise KeyError(key)


def main():
    doc = Document(os.path.join(HERE, 'template.docx'))

    sec31 = find_par(doc, '3.1 D-H参数表')
    sec32 = find_par(doc, '3.2 正运动学求解与推导过程')
    sec33 = find_par(doc, '3.3 逆运动学解析解推导过程')
    sec34 = find_par(doc, '3.4 轨迹规划方法')
    sec4 = find_par(doc, '四、实验结果说明与分析')

    # ---------------- 3.1 DH 参数表 ----------------
    p = add_par_after(sec31, indent=True,
        text='机械臂为 7 自由度对称构型（关节轴依次为 Z-Y-X-X-X-Y-Z），两端均为可与'
        '基座电磁吸合的末端。以左足（L_Base）落足面为基坐标系{0}（z0 垂直于落足面'
        '向外），沿链路到右足（R_Base）建立各关节坐标系，按标准 D-H 方法（zi 沿'
        '关节 i+1 轴线）测得参数如下（长度单位 mm，角度单位 °）：')
    tbl = doc.add_table(rows=9, cols=5)
    tbl_pr = tbl._tbl.tblPr
    from docx.oxml.ns import qn
    borders = tbl_pr.makeelement(qn('w:tblBorders'), {})
    for edge in ('top', 'left', 'bottom', 'right', 'insideH', 'insideV'):
        el = tbl_pr.makeelement(qn('w:' + edge), {qn('w:val'): 'single',
                                                  qn('w:sz'): '4'})
        borders.append(el)
    tbl_pr.append(borders)
    head = ['i', 'a(i-1)/mm', 'α(i-1)/°', 'd(i)/mm', 'θ(i)']
    rows = [
        ['1', '0', '0', '65', 'θ1'],
        ['2', '0', '90', '50', 'θ2'],
        ['3', '0', '90', '50', 'θ3'],
        ['4', '400', '0', '100', 'θ4'],
        ['5', '400', '180', '100', 'θ5'],
        ['6', '0', '90', '-50', 'θ6'],
        ['7', '0', '90', '-50', 'θ7'],
        ['末端', '0', '0', '65', '—'],
    ]
    for j, t in enumerate(head):
        tbl.rows[0].cells[j].text = t
    for i, r in enumerate(rows):
        for j, t in enumerate(r):
            tbl.rows[i + 1].cells[j].text = t
    p._p.addnext(tbl._tbl)

    # ---------------- 3.2 正运动学 ----------------
    p = add_par_after(sec32, indent=True,
        text='每个关节绕自身 z 轴旋转，相邻坐标系之间的变换为常值矩阵 Ai（由零位测量'
        '得到）。正运动学为齐次变换连乘：')
    p = add_par_after(p, 'T(0→7) = A0·Rz(θ1)·A1·Rz(θ2)·A2·Rz(θ3)·A3·Rz(θ4)·A4·Rz(θ5)·A5·Rz(θ6)·A6·Rz(θ7)·A7',
                      align=WD_ALIGN_PARAGRAPH.CENTER)
    p = add_par_after(p, indent=True,
        text='其中 Rz(θ) 为绕 z 轴的旋转矩阵，Ai 由 3.1 的 D-H 参数（a、α、d）确定：'
        'Ai = Trans(z, d)·Rot(x, α)·Trans(x, a)。程序实现见 src/kinematics.py 的 fk()。'
        '经验证：零位 θ=0 时正运动学给出的右足位姿 (0.35, 0, 0.235) 与仿真场景中'
        'R_Base 的实际位姿完全一致（误差 < 1e-6 m）。')

    # ---------------- 3.3 逆运动学解析解 ----------------
    texts33 = [
        '行走时支撑足固定于落足盘、摆动足需到达给定位姿，7 自由度存在 1 个冗余'
        '自由度。取矢状面（x–z 平面）行走位形族并用“对称拱形”条件消解冗余：',
        '(1) 令 θ1 = θ7 = 0，机械臂保持在 y = 0 平面内；',
        '(2) 中间三个平行俯仰关节取 θ3 = t，θ4 = π − 2t，θ5 = π − t。此时三关节'
        '簇的净转角为 0（末端姿态不受影响），且其在 y 方向的位移相互抵消，'
        '两肩关节之间形成一根“虚拟连杆” v = (−L, h)，其中 L = 300 mm 为固定轴向'
        '距离，h = 2·l·cos t（l = 400 mm 为长臂杆长度）；',
        '(3) 设目标摆动足相对支撑足的偏移为 (D, Δz)（足面法线保持竖直），令'
        ' r² = D² + Δz²，则解析解为：',
        'h² = r² − L² (h ≥ 0)，  t = arccos( h / 2l )',
        'θ2 = atan2(Δz, D) − atan2(h, −L)，  θ6 = −θ2',
        '该平面问题存在两个解析解：t = arccos(h/2l) ≥ 0 对应“肘上”支，取 t → −t'
        '（相应 θ4 = π − 2t、θ5 = π − t 随之改变）得到“肘下”支，两支到达完全相同'
        '的足端位姿。程序取其中一支，并在相邻路径点之间选择与上一点连续的 2π 分支'
        '以避免关节跳变。',
        '可解性条件为 L² ≤ r² ≤ L² + 4l²，即 0.3 m ≤ r ≤ 0.906 m。第 1 步'
        '（D = 0.6 m）满足该条件。位置 2 位于侧面落足盘（法线沿 +y，需三维姿态'
        '变换），此时在解析解给出的初值附近用基于解析雅可比矩阵的阻尼最小二乘法'
        '迭代求精（src/kinematics.py 的 ik_numeric()），数步内收敛到 1e-6。',
        '此外，θ3 = θ5 = s、θ4 = 0（θ1 = θ2 = θ6 = θ7 = 0）为保持两足位姿不变的'
        '自运动族，用于起步前从零位平滑过渡到拱形位形族（s: 0 → π/2）。',
    ]
    p = sec33
    for t in texts33:
        center = t.startswith(('h² =', 'θ2 ='))
        p = add_par_after(p, t, align=WD_ALIGN_PARAGRAPH.CENTER if center else None,
                          indent=not center and not t.startswith('('))

    # ---------------- 3.4 轨迹规划 ----------------
    texts34 = [
        '采用关节空间分段五次多项式（quintic）轨迹。相邻路径点之间使用归一化时间'
        '标度 s(τ) = 10τ³ − 15τ⁴ + 6τ⁵，其一、二阶导数在两端均为零，因此整条轨迹'
        '的关节位置、速度、加速度全程连续，且吸合/脱开瞬间速度与加速度为零，满足'
        '电磁铁吸合的平稳性要求。',
        '路径点由逆运动学求得：第 1 步（左足 0.65 m → 位置 1（−0.25 m 顶部落足'
        '盘），以右足为支撑）共 7 个路径点（含自运动过渡、抬升、越过支撑足、下落'
        '吸合），历时 14 s；第 2 步（右足 0.35 m → 位置 2（−0.65 m 侧面落足盘，'
        '法线沿 +y），以左足为支撑）共 7 个路径点（抬升、沿舱体上方平移、在舱体'
        '上方转入侧面姿态、沿侧面落足盘 +y 法线逐段逼近并吸合），历时 13 s。'
        '实现见 src/trajectory.py 与 src/walk.py。',
        '行走通过“基座交换”实现：第 1 步中右足固定，运动学树根（L_Base）按'
        ' T_L = T_R(fixed)·T(0→7)(q)⁻¹ 浮动更新；第 2 步左足（树根）固定，直接'
        '驱动关节。',
    ]
    p = sec34
    for t in texts34:
        p = add_par_after(p, t, indent=True)

    # ---------------- 四、实验结果 ----------------
    res = [
        '仿真在 CoppeliaSim 4.7（ZeroMQ Remote API + Python）中完成。机械臂从初始'
        '位置（两足位于 x = 0.65 m 与 0.35 m 顶部落足盘）出发，第 1 步左足吸合到'
        '位置 1（x = −0.25 m 顶部落足盘），第 2 步右足吸合到位置 2（x = −0.65 m '
        '侧面落足盘），完成 2 步行走。终点位姿误差为 0（左足 (−0.25, 0, 0.235)，'
        '右足 (−0.65, 0.1325, 0.101)，均与落足盘理论位姿一致）。',
        '全程关节速度峰值约 4.4 rad/s、加速度峰值约 5.5 rad/s²，速度与加速度曲线'
        '连续无跳变（图 2），足端在吸合与脱开时刻速度、加速度为零；第 1 步中两足'
        '始终保持在 y = 0 平面内，第 2 步足端法线由 +z 平滑翻转到 +y。',
        '第 2 步路径考虑了与舱体（半径约 0.134 m 的圆柱）的避碰：每个路径点在'
        '碰撞检测下选取无碰撞的逆解分支（src/plan_step2.py），足端在舱体上方转入'
        '侧面落足盘姿态后沿其 +y 法线方向逐步逼近，臂杆自然绕过舱体侧面；对整条'
        '五次多项式轨迹按 400 个采样点密集校验，各连杆与舱体表面的最小间隙全程'
        '非负，无穿透。',
    ]
    p = sec4
    for t in res:
        p = add_par_after(p, t, indent=True)
    p = add_pic_after(p, os.path.join(FIG, 'initial_pose.png'), 12,
                      '图 1  初始位形（左：场景总览，两足位于顶部落足盘）')
    p = add_pic_after(p, os.path.join(FIG, 'step1_swing.png'), 12,
                      '图 2  第 1 步摆动过程（左足越过支撑足摆向位置 1）')
    p = add_pic_after(p, os.path.join(FIG, 'step2_swing.png'), 12,
                      '图 3  第 2 步摆动过程（右足在舱体上方转入侧面姿态）')
    p = add_pic_after(p, os.path.join(FIG, 'step2_dock.png'), 12,
                      '图 4  第 2 步沿侧面落足盘 +y 法线逼近（臂杆绕过舱体，无穿透）')
    p = add_pic_after(p, os.path.join(FIG, 'final_docked.png'), 12,
                      '图 5  行走完成（左足位于位置 1，右足吸合于侧面位置 2）')
    p = add_pic_after(p, os.path.join(FIG, 'joint_curves.png'), 12,
                      '图 6  关节位置/速度/加速度曲线')
    p = add_pic_after(p, os.path.join(FIG, 'foot_positions.png'), 12,
                      '图 7  两足端位置随时间变化曲线')

    # ---------------- 运动学与轨迹的仿真验证 ----------------
    p = add_par_after(p, '运动学与轨迹的仿真验证', bold=True, size=12)
    p = add_par_after(p, indent=True,
        text='为验证正/逆运动学模型的正确性，将若干组关节角逐一通过 ZeroMQ Remote '
        'API 设入 CoppeliaSim，读取仿真界面中 R_Base 的世界位姿，与链式正运动学'
        ' T = T(L_Base)·T(0→7)(q) 的预测值逐组比对，结果如下表。三组任意位形（含零'
        '位与两组随机角）以及逆运动学的“肘上/肘下”两解，末端位置误差均为 0（数值'
        '层面约 1e-13 m），且两个逆解到达完全相同的目标位姿 (1.25, 0, 0.385)，印证'
        '了解析解的双分支性质。')
    pcont = add_par_after(p, indent=True,
        text='对轨迹的连续性亦作数值核验：在每个路径点采样分段五次多项式，关节速度与'
        '加速度均为 0，相邻采样点间无跳变，解析导数与位置的数值微分一致，确认整条'
        '轨迹的位置、速度、加速度全程连续。')
    vt = doc.add_table(rows=6, cols=4)
    vt_pr = vt._tbl.tblPr
    vborders = vt_pr.makeelement(qn('w:tblBorders'), {})
    for edge in ('top', 'left', 'bottom', 'right', 'insideH', 'insideV'):
        el = vt_pr.makeelement(qn('w:' + edge), {qn('w:val'): 'single',
                                                 qn('w:sz'): '4'})
        vborders.append(el)
    vt_pr.append(vborders)
    vhead = ['配置', '仿真界面(x,y,z)/m', '链式FK(x,y,z)/m', '位置误差']
    vrows = [
        ['零位 q=0', '(0.350, 0, 0.235)', '(0.350, 0, 0.235)', '0'],
        ['任意位形 A', '(0.590, -0.441, 0.566)', '(0.590, -0.441, 0.566)', '0'],
        ['任意位形 B', '(0.529, -0.585, 0.061)', '(0.529, -0.585, 0.061)', '0'],
        ['IK 解(肘上)', '(1.250, 0, 0.385)', '(1.250, 0, 0.385)', '0'],
        ['IK 解(肘下)', '(1.250, 0, 0.385)', '(1.250, 0, 0.385)', '0'],
    ]
    for j, t in enumerate(vhead):
        vt.rows[0].cells[j].text = t
    for i, r in enumerate(vrows):
        for j, t in enumerate(r):
            vt.rows[i + 1].cells[j].text = t
    p._p.addnext(vt._tbl)   # place table between intro (p) and continuity (pcont)

    out = os.path.join(HERE, '实验1-机械臂运动控制仿真-实验报告.docx')
    doc.save(out)
    print('saved', out)


if __name__ == '__main__':
    main()
