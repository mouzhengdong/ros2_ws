## pixel2cam 函数数学推导

将图像像素坐标 $\mathbf{p} = [u, v]^T$ 转换到归一化相机坐标系 $\mathbf{x} = [x, y]^T$（Z=1平面）。

---

### 1. 相机内参矩阵

相机内参矩阵 $K$ 定义为：

$$
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

其中：
- $f_x, f_y$：x和y方向的焦距（像素单位）
- $c_x, c_y$：主点坐标（像素单位）

---

### 2. 坐标变换关系

像素坐标到归一化坐标的投影关系：

$$
\begin{bmatrix}
u \\
v \\
1
\end{bmatrix}
=
K
\begin{bmatrix}
x \\
y \\
1
\end{bmatrix}
$$

---

### 3. 反解归一化坐标

从像素坐标反解归一化坐标：

$$
\begin{cases}
x = \dfrac{u - c_x}{f_x} \\
y = \dfrac{v - c_y}{f_y}
\end{cases}
$$

---

### 4. 代码实现

```cpp
Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d(
    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),  // x = (u - cx)/fx
    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)   // y = (v - cy)/fy
  );
}