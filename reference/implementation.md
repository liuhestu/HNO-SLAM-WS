# Observer.cpp Implementation Checklist

本文档只保留在 `Observer.cpp` 中真正会直接调用或显式实现的公式，按工程执行顺序整理。  
只考虑**双目视觉**版本，并标注论文原文序号。

---

## 1. 初始化时需要准备的量

### 1.1 状态结构（原文公式 (4)）

主状态按下面顺序组织：

$$
\hat{X} = M(\hat{R},\hat{p},\hat{v},\hat{g},\hat{p}_L),
\qquad
\hat{p}_L = [\hat{p}_1,\dots,\hat{p}_n].
$$

在 `Observer.cpp` 里通常不会真的存整个矩阵 $\hat{X}$，而是分别存：

- $\hat{R}$
- $\hat{p}$
- $\hat{v}$
- $\hat{g}$
- $\hat{p}_i$

### 1.2 投影算子（原文公式 (3)）

$$
\pi(x) = I - \frac{x x^\top}{\|x\|^2},
\qquad x \neq 0.
$$

用途：双目 bearing 创新里要用它去掉视线方向。

### 1.3 选择矩阵（原文公式 (28) 前定义）

$$
Q := \begin{bmatrix} I_3 & 0_{3\times(3+n)} \end{bmatrix}.
$$

用途：从每个地标创新项里提取前 3 维，拼出总视觉创新 $\sigma_p$。

### 1.4 平移协方差 / Riccati 矩阵初值（原文公式 (9)）

$$
P(0) = P(0)^\top > 0.
$$

用途：后续传播和视觉更新都围绕 $P$ 展开。

---

## 2. IMU 传播时真正会调用的公式

### 2.1 旋转修正项

论文在 Section V 中选取

$$
\sigma_R := \hat{g} \times g.
$$

用途：这是每个 IMU 时刻都要计算的姿态反馈。

### 2.2 显式观测器连续方程（原文公式 (46) - (50)）

完整形式是：

$$
\dot{\hat{R}}
=
\hat{R}[\omega^B + \hat{R}^\top \sigma_R]_\times
$$

$$
\dot{\hat{p}}
=
[\sigma_R]_\times \hat{p}
+ \hat{v}
+ \hat{R}\sum_{j=1}^{n} K_j^p \sigma_j^p
$$

$$
\dot{\hat{v}}
=
[\sigma_R]_\times \hat{v}
+ \hat{g}
+ \hat{R}a^B
+ \hat{R}\sum_{j=1}^{n} K_j^v \sigma_j^p
$$

$$
\dot{\hat{g}}
=
[\sigma_R]_\times \hat{g}
+ \hat{R}\sum_{j=1}^{n} K_j^g \sigma_j^p
$$

$$
\dot{\hat{p}}_i
=
[\sigma_R]_\times \hat{p}_i
+ \hat{R}\sum_{j=1}^{n} \Gamma_{ij}\sigma_j^p.
$$

但在 `Observer.cpp` 的 **IMU 传播阶段**，实际用的是它们的无视觉项版本（对应 Algorithm 1 第 5 - 9 行）：

$$
\dot{\hat{R}} = \hat{R}[\omega^B + \hat{R}^\top \sigma_R]_\times
$$

$$
\dot{\hat{p}} = [\sigma_R]_\times \hat{p} + \hat{v}
$$

$$
\dot{\hat{v}} = [\sigma_R]_\times \hat{v} + \hat{g} + \hat{R}a^B
$$

$$
\dot{\hat{g}} = [\sigma_R]_\times \hat{g}
$$

$$
\dot{\hat{p}}_i = [\sigma_R]_\times \hat{p}_i.
$$

用途：这是 `propagate()` 里最核心的一组状态更新公式。

### 2.3 平移误差传播矩阵（原文公式 (35) 与 (37)）

$$
A(t)
=
\begin{bmatrix}
D(t) & 0_{6\times 3n} \\
B_n \otimes I_3 & I_n \otimes (-[\omega^B]_\times)
\end{bmatrix},
$$

其中

$$
D(t)
=
\begin{bmatrix}
-[\omega^B]_\times & I_3 \\
0_{3\times 3} & -[\omega^B]_\times
\end{bmatrix},
\qquad
B_n = \begin{bmatrix} 1^\top & 0_{n\times 1} \end{bmatrix}.
$$

用途：这是 `P` 传播时要构造的系统矩阵。

### 2.4 连续 Riccati 传播（原文公式 (9) 的传播部分）

理论完整式是

$$
\dot{P}(t)
=
A(t)P(t) + P(t)A^\top(t)
- P(t)C^\top(t)Q^{-1}(t)C(t)P(t)
+ V(t).
$$

但在混合实现里，IMU 段只传播先验（Algorithm 1 第 10 行）：

$$
\dot{P} = A(t)P + PA^\top(t) + V(t).
$$

用途：这是 `propagate()` 里对 `P` 的更新公式。

---

## 3. 双目视觉更新时真正会调用的公式

### 3.1 像素到 bearing（原文公式 (24)）

对左右相机分别做：

$$
\bar{y}_i
=
\frac{K^{-1} z_i}{\|K^{-1} z_i\|},
\qquad
z_i = [u_i,v_i,1]^\top .
$$

用途：把前端给的像素坐标转成单位方向向量。

### 3.2 双目修正矩阵（原文公式 (27)）

$$
\Xi_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q).
$$

用途：给每个地标建立双目方向约束。

### 3.3 每个地标的创新项（原文公式 (26)）

双目情形下，单地标创新项可写成

$$
\tilde{y}_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q)\,
(\hat{X}^{-1}r_i - p_{c_q}),
$$

统一写法是

$$
\tilde{y}_i
=
\Xi_i\left(\hat{X}^{-1}r_i - X^{-1}r_i\right).
$$

用途：这是每个观测地标的原始残差。

### 3.4 总视觉创新（原文公式 (28)）

$$
\sigma_p
=
\left(
Q[\tilde{y}_1,\tilde{y}_2,\dots,\tilde{y}_n]
\right)^\vee .
$$

用途：把全部地标残差拼成一个总修正向量，供后面的增益更新使用。

### 3.5 输出矩阵（原文公式 (33)）

$$
C(t)
=
\begin{bmatrix}
0_3 & 0_3 & \Pi_1 & 0_3 & \cdots & 0_3 \\
0_3 & 0_3 & 0_3 & \Pi_2 & \cdots & 0_3 \\
\vdots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0_3 & 0_3 & 0_3 & 0_3 & \cdots & \Pi_n
\end{bmatrix},
$$

其中

$$
\Pi_i := Q \Xi_i Q^\top.
$$

用途：这是视觉更新时要构造的测量矩阵。

### 3.6 离散增益（Algorithm 1 第 13 行）

混合实现使用

$$
L
=
P C^\top(t_k)\left(C(t_k) P C^\top(t_k) + Q(t_k)\right)^{-1}.
$$

用途：这是视觉更新时真正求解的总增益。

---

## 4. 从总增益分解到各更新块

### 4.1 总关系式（原文公式 (36)）

$$
L(t)
=
\left(I_{n+2}\otimes \hat{R}^\top\right)
\begin{bmatrix}
K_v(t) \\
K_g(t) \\
1^\top \otimes K_p(t) - \Gamma(t)
\end{bmatrix}.
$$

用途：说明 `Observer.cpp` 里求出的总增益 $L$，最终要拆成速度、重力、位置和地标的更新矩阵。

### 4.2 提取 $K_v$（原文公式 (43)）

$$
K_v(t)
=
\begin{bmatrix}
I_{3n} & 0_{3n\times 3} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t).
$$

### 4.3 提取 $K_g$（原文公式 (44)）

$$
K_g(t)
=
\begin{bmatrix}
0_{3n\times 3} & I_{3n} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t).
$$

### 4.4 提取 $K_p$ 与 $\Gamma$ 的组合（原文公式 (45)）

$$
1^\top \otimes K_p(t) - \Gamma(t)
=
\begin{bmatrix}
0_{3n\times 3} & 0_{3n\times 3} & I_{3n}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t).
$$

用途：`K_p` 和 `\Gamma` 不唯一，但它们的组合必须满足这个式子。

如果实现里采用简单选法，也可以取

$$
K_p(t) = 0,
$$

然后把这部分全部吸收到 $\Gamma(t)$ 里。

---

## 5. 视觉更新时真正执行的状态更新

### 5.1 Algorithm 1 的离散状态更新（第 15 - 19 行）

$$
\hat{R}^+ = \hat{R}
$$

$$
\hat{p}^+ = \hat{p} + \hat{R}\sum_{j=1}^{n} K_j^p \sigma_j^p
$$

$$
\hat{v}^+ = \hat{v} + \hat{R}\sum_{j=1}^{n} K_j^v \sigma_j^p
$$

$$
\hat{g}^+ = \hat{g} + \hat{R}\sum_{j=1}^{n} K_j^g \sigma_j^p
$$

$$
\hat{p}_i^+ = \hat{p}_i + \hat{R}\sum_{j=1}^{n} \Gamma_{ij}\sigma_j^p
$$

用途：这是 `update()` 里真正修改状态的公式。

### 5.2 协方差 / Riccati 矩阵更新（Algorithm 1 第 20 行）

$$
P^+ = (I_{3n+6} - LC)P.
$$

用途：视觉信息注入后，同步压缩不确定度。

---

## 6. 最小实验清单

如果你现在只想实现 `Observer.cpp`，建议只按下面顺序逐项完成并自测：

1. 实现投影算子 $\pi(x)$（原文公式 (3)）
2. 实现 IMU 传播：
   - $\sigma_R = \hat{g}\times g$
   - 无视觉项版本的公式 (46) - (50)
3. 实现 $A(t)$ 构造（原文公式 (35)、(37)）
4. 实现 $P$ 传播：
   - $\dot{P} = AP + PA^\top + V$
5. 实现双目像素到 bearing：
   - 原文公式 (24)
6. 实现每个地标的 $\Xi_i$：
   - 原文公式 (27)
7. 实现每个地标的 $\tilde{y}_i$：
   - 原文公式 (26)
8. 实现总创新 $\sigma_p$：
   - 原文公式 (28)
9. 实现测量矩阵 $C(t)$：
   - 原文公式 (33)
10. 实现离散增益：
   - Algorithm 1 第 13 行
11. 实现 $L \rightarrow K_v,K_g,K_p,\Gamma$ 分解：
   - 原文公式 (36)、(43)、(44)、(45)
12. 实现离散状态更新：
   - Algorithm 1 第 15 - 19 行
13. 实现 $P$ 的离散更新：
   - Algorithm 1 第 20 行

这 13 项，就是 `Observer.cpp` 最小可运行版本真正需要的公式集合。
