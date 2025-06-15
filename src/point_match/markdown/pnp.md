# Bundle Adjustment with Gauss-Newton: Full Derivation

## 1. 问题描述

我们有一组世界坐标系下的 3D 点 $\mathbf{P}_i = [X_i, Y_i, Z_i]^T$，以及相机观测到的 2D 像素坐标 $\mathbf{p}_i = [u_i^{\text{obs}}, v_i^{\text{obs}}]^T$。

相机内参为：

$$
K = 
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

相机位姿 $T \in SE(3)$，用于将世界坐标点变换到相机坐标系：

$$
\mathbf{P}_i^c = T \cdot \mathbf{P}_i = R \mathbf{P}_i + \mathbf{t}
$$

像素坐标通过针孔模型投影为：

$$
\begin{aligned}
u_i &= f_x \frac{X_i^c}{Z_i^c} + c_x \\
v_i &= f_y \frac{Y_i^c}{Z_i^c} + c_y
\end{aligned}
$$

---

## 2. 目标函数

目标是最小化重投影误差：

$$
\mathbf{e}_i = 
\begin{bmatrix}
u_i^{\text{obs}} - f_x \frac{X_i^c}{Z_i^c} - c_x \\
v_i^{\text{obs}} - f_y \frac{Y_i^c}{Z_i^c} - c_y
\end{bmatrix}
$$

总目标函数为：

$$
\min_{\boldsymbol{\xi}} \sum_{i=1}^N \left\| \mathbf{e}_i(\boldsymbol{\xi}) \right\|^2
$$

其中 $\boldsymbol{\xi} \in \mathbb{R}^6$ 是 $SE(3)$ 的李代数，表示扰动。

---

## 3. Gauss-Newton 更新

在每次迭代中，更新量 $\Delta \boldsymbol{\xi}$ 解如下线性系统：

$$
\Delta \boldsymbol{\xi} = (J^\top J)^{-1} J^\top \mathbf{e}
$$

然后更新位姿：

$$
T_{\text{new}} = \exp(\Delta \boldsymbol{\xi}) \cdot T
$$

---

## 4. 雅可比推导

设相机坐标下的点为 $\mathbf{P}^c = [X, Y, Z]^\top$，重投影误差为：

$$
\mathbf{e} =
\begin{bmatrix}
u^{\text{obs}} - f_x \frac{X}{Z} - c_x \\
v^{\text{obs}} - f_y \frac{Y}{Z} - c_y
\end{bmatrix}
$$

### 对 $\boldsymbol{\xi}$ 的导数：

$$
\frac{\partial \mathbf{P}^c}{\partial \boldsymbol{\xi}} = 
\begin{bmatrix}
1 & 0 & 0 & 0 & Z & -Y \\
0 & 1 & 0 & -Z & 0 & X \\
0 & 0 & 1 & Y & -X & 0
\end{bmatrix}
$$

### 对 $\mathbf{P}^c$ 的导数：

$$
\frac{\partial \mathbf{e}}{\partial \mathbf{P}^c} = 
\begin{bmatrix}
-\frac{f_x}{Z} & 0 & \frac{f_x X}{Z^2} \\
0 & -\frac{f_y}{Z} & \frac{f_y Y}{Z^2}
\end{bmatrix}
$$

### 链式法则得到总雅可比：

$$
\frac{\partial \mathbf{e}}{\partial \boldsymbol{\xi}} = 
\frac{\partial \mathbf{e}}{\partial \mathbf{P}^c} \cdot \frac{\partial \mathbf{P}^c}{\partial \boldsymbol{\xi}}
$$

即：

$$
J =
\begin{bmatrix}
-\frac{f_x}{Z} & 0 & \frac{f_x X}{Z^2} & \frac{f_x XY}{Z^2} & -f_x - \frac{f_x X^2}{Z^2} & \frac{f_x Y}{Z} \\
0 & -\frac{f_y}{Z} & \frac{f_y Y}{Z^2} & f_y + \frac{f_y Y^2}{Z^2} & -\frac{f_y XY}{Z^2} & -\frac{f_y X}{Z}
\end{bmatrix}
$$

---

## 5. 迭代终止条件

设误差平方和为：

$$
\text{cost} = \sum_i \| \mathbf{e}_i \|^2
$$

当误差不再下降或更新量 $\| \Delta \boldsymbol{\xi} \| < \epsilon$ 时，终止迭代。

---

## 6. 总结

这是一个典型的基于 Gauss-Newton 的 Bundle Adjustment 问题，其中：

- 变量是相机位姿 $T$
- 误差是 2D 观测和重投影之间的差异
- 优化目标是最小化重投影误差平方和
- 雅可比利用 $SE(3)$ 李代数链式法则推导

---
