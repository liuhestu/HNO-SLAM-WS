# Stereo VIO Geometric Observer Algorithm

本文档只保留论文 *Nonlinear Observer Design for Visual-Inertial Odometry* 中与**双目视觉**实现直接相关的理论框架，并改成更适合 Markdown 预览的写法：

- 公式统一使用 `$$ ... $$`
- 原文公式编号写在标题或正文中
- 不包含稳定性证明、引理和单目 / 相对位置测量

---

## 1. 状态与系统模型

### 1.1 状态所在李群（原文公式 (4)）

系统扩展状态定义在矩阵李群 $SE_{3+n}(3)$ 上：

$$
SE_{3+n}(3)
:=
\left\{
X = M(R,x_1,x_2,x_3,x_L)
\; \middle| \;
R \in SO(3),\;
x_1,x_2,x_3 \in \mathbb{R}^3,\;
x_L \in \mathbb{R}^{3 \times n}
\right\}.
$$

其中

$$
M(R,x_1,x_2,x_3,x_L)
:=
\begin{bmatrix}
R & x_1 & x_2 & x_3 & x_L \\
0_{(3+n)\times 3} & I_{3+n}
\end{bmatrix}.
$$

在 VIO 中取

$$
X = M(R,p,v,g,p_L),
\qquad
p_L = [p_1,\dots,p_n].
$$

因此代码里要维护的核心状态就是：

- $R$：姿态
- $p$：位置
- $v$：速度
- $g$：辅助重力状态
- $p_i$：第 $i$ 个地标位置

### 1.2 李代数块结构（原文公式 (5)）

$$
V(\Omega,x_1,x_2,x_3,x_L)
:=
\begin{bmatrix}
\Omega & x_1 & x_2 & x_3 & x_L \\
0_{(3+n)\times 3} & 0_{3+n}
\end{bmatrix},
\qquad
\Omega \in \mathfrak{so}(3).
$$

后续 IMU 群速度 $V$ 和创新项 $\Delta$ 都按这个块结构组织。

### 1.3 连续系统动力学（原文公式 (12) - (16)）

$$
\dot{R} = R[\omega^B]_\times
$$

$$
\dot{p} = v
$$

$$
\dot{v} = g + R a^B
$$

$$
\dot{p}_i = 0
$$

$$
\dot{g} = 0.
$$

其中：

- $\omega^B$ 是 IMU 机体系角速度
- $a^B$ 是 IMU 机体系表观加速度
- $g$ 是惯性系中的常值重力向量

### 1.4 群形式动力学（原文公式 (17)）

$$
\dot{X} = [X,H] + X V
$$

李括号定义为

$$
[X_1,X_2] = X_1 X_2 - X_2 X_1.
$$

矩阵 $H$ 定义为

$$
H =
\begin{bmatrix}
0_3 & 0_{3\times(3+n)} \\
0_{(3+n)\times 3} & S
\end{bmatrix},
$$

其中

$$
S =
\begin{bmatrix}
0 & 0 & 0 & \cdots & 0 & 0 \\
1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 0 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & \cdots & 0 & 0
\end{bmatrix}
\in \mathbb{R}^{(3+n)\times(3+n)}.
$$

IMU 输入对应的群速度为

$$
V
=
V\left(
[\omega^B]_\times,\;
0_{3\times 1},\;
a^B,\;
0_{3\times 1},\;
0_{3\times n}
\right).
$$

这一步的目的，是把原始动力学统一写进群结构中，便于后续观测器设计。

---

## 2. 双目测量模型

### 2.1 双目 bearing 观测（原文公式 (20)）

设 $(R_{c_1},p_{c_1})$、$(R_{c_2},p_{c_2})$ 是从机体系 $\{B\}$ 到左右相机坐标系 $\{C_1\},\{C_2\}$ 的外参，则第 $i$ 个地标在第 $q$ 个相机中的单位视线向量为

$$
\bar{y}_i^q
=
\frac{
R_{c_q}^{\top}\left(R^\top (p_i-p)-p_{c_q}\right)
}{
\left\|
R_{c_q}^{\top}\left(R^\top (p_i-p)-p_{c_q}\right)
\right\|
},
\qquad q \in \{1,2\}.
$$

### 2.2 群形式双目测量（原文公式 (21)）

$$
\bar{y}_i^q
=
\frac{
X_{c_q}^{-1}\left(X^{-1}r_i-p_{c_q}\right)
}{
\left\|
X_{c_q}^{-1}\left(X^{-1}r_i-p_{c_q}\right)
\right\|
}.
$$

其中

$$
X_{c_q} := M(R_{c_q},0_{3\times 1},0_{3\times 1},0_{3\times 1},0_{3\times n}),
$$

$$
r_i := \begin{bmatrix} 1 & 0 & 0 & -e_i^\top \end{bmatrix}^\top \in \mathbb{R}^{3+n},
$$

$e_i$ 是 $\mathbb{R}^n$ 的第 $i$ 个标准基。

### 2.3 像素到 bearing（原文公式 (24)）

如果前端给的是像素坐标，则先通过相机内参 $K$ 转成单位 bearing：

$$
\bar{y}_i
=
\frac{K^{-1} z_i}{\|K^{-1} z_i\|},
\qquad
z_i = [u_i,v_i,1]^\top .
$$

对双目实现而言，这一步要分别在左右目上执行，得到 $\bar{y}_i^1$ 与 $\bar{y}_i^2$。

---

## 3. 观测器总方程

### 3.1 连续观测器（原文公式 (25)）

$$
\dot{\hat{X}} = [\hat{X},H] + \hat{X}V + \Delta \hat{X}.
$$

其中

$$
\hat{X} = M(\hat{R},\hat{p},\hat{v},\hat{g},\hat{p}_L),
\qquad
\hat{p}_L = [\hat{p}_1,\dots,\hat{p}_n].
$$

这里 $\Delta$ 是创新项，负责把视觉和重力信息注入估计器。

### 3.2 投影算子（原文公式 (3)）

$$
\pi(x) = I - \frac{x x^\top}{\|x\|^2},
\qquad x \neq 0.
$$

在双目测量里，它用于去掉视线方向，只保留与 bearing 约束一致的正交误差分量。

### 3.3 每个地标的创新项（原文公式 (26)）

对双目测量，单地标创新项写成

$$
\tilde{y}_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q)\,
(\hat{X}^{-1}r_i - p_{c_q}),
$$

并统一写成

$$
\tilde{y}_i
=
\Xi_i\left(\hat{X}^{-1}r_i - X^{-1}r_i\right).
$$

### 3.4 双目对应的修正矩阵（原文公式 (27)）

对双目测量，

$$
\Xi_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q).
$$

它的含义是：左右两个相机分别提供一个 bearing 约束，再把它们加起来形成该地标的总方向约束。

### 3.5 总平移创新（原文公式 (28)）

定义选择矩阵

$$
Q := \begin{bmatrix} I_3 & 0_{3\times(3+n)} \end{bmatrix}.
$$

然后将所有地标创新项堆叠成总平移创新：

$$
\sigma_p
=
\left(
Q[\tilde{y}_1,\tilde{y}_2,\dots,\tilde{y}_n]
\right)^\vee .
$$

其中 $(\cdot)^\vee$ 表示按列堆叠成向量，因此

$$
\sigma_p \in \mathbb{R}^{3n}.
$$

它是视觉更新阶段最核心的残差信号。

### 3.6 创新项总结构（原文公式 (29)）

$$
\Delta
=
V\left(
k_R[\sigma_R]_\times,\;
K_p(t)\sigma_p,\;
K_v(t)\sigma_p,\;
K_g(t)\sigma_p,\;
(\Gamma(t)\sigma_p)^\wedge
\right).
$$

其中：

- $\sigma_R$ 是旋转修正项
- $\sigma_p$ 是视觉驱动的平移修正项
- $K_p,K_v,K_g,\Gamma$ 是由 Riccati / Kalman 型增益分解得到的校正矩阵

---

## 4. 平移子系统

### 4.1 平移误差状态（原文公式 (31)）

论文定义平移误差状态为

$$
x =
\begin{bmatrix}
(R^\top \tilde{v})^\top \\
(R^\top \tilde{g})^\top \\
\left(R^\top(1 \otimes \tilde{p} - \tilde{p}_L)\right)^{\vee\top}
\end{bmatrix}^{\top}.
$$

从实现角度看，它包含三部分：

- 机体系速度误差
- 机体系重力误差
- 机体系相对地标位置误差

### 4.2 平移创新与误差的关系（原文公式 (32)）

$$
\sigma_p = C(t)x.
$$

这说明视觉创新 $\sigma_p$ 可以看成平移误差状态的线性输出。

### 4.3 输出矩阵 $C(t)$（原文公式 (33)）

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

对双目测量，$\Xi_i$ 来自上面的双目公式。

### 4.4 平移误差闭环形式（原文公式 (34)）

$$
\dot{x} = (A(t) - L(t)C(t))x.
$$

其中 $A(t)$ 是传播矩阵，$L(t)$ 是增益矩阵。

### 4.5 传播矩阵 $A(t)$（原文公式 (35) 与 (37)）

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

这一步在代码中用于传播 $P(t)$。

---

## 5. 旋转子系统

### 5.1 旋转修正项

论文选取

$$
\sigma_R := \hat{g} \times g.
$$

也就是说，姿态校正只依赖“估计重力方向”和“真实重力方向”的失配。

### 5.2 显式观测器形式（原文公式 (46) - (50)）

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

这些是最适合直接翻译成代码的连续状态方程。

---

## 6. Riccati / Kalman 型增益构造

### 6.1 连续 Riccati 方程（原文公式 (9)）

$$
\dot{P}(t)
=
A(t)P(t) + P(t)A^\top(t)
- P(t)C^\top(t)Q^{-1}(t)C(t)P(t)
+ V(t).
$$

其中：

- $P(t)$ 是平移误差系统的矩阵变量
- $Q(t)$ 是测量权重 / 噪声矩阵
- $V(t)$ 是过程权重 / 噪声矩阵

### 6.2 连续增益

论文理论部分使用

$$
L(t) = P(t)C^\top(t)Q^{-1}(t).
$$

### 6.3 增益与校正矩阵的关系（原文公式 (36)）

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

这说明：先算出总增益 $L(t)$，再由它分解出

- $K_v(t)$
- $K_g(t)$
- $K_p(t)$
- $\Gamma(t)$

### 6.4 从 $L(t)$ 提取 $K_v$、$K_g$、$K_p$、$\Gamma$（原文公式 (43) - (45)）

$$
K_v(t)
=
\begin{bmatrix}
I_{3n} & 0_{3n\times 3} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t),
$$

$$
K_g(t)
=
\begin{bmatrix}
0_{3n\times 3} & I_{3n} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t),
$$

$$
1^\top \otimes K_p(t) - \Gamma(t)
=
\begin{bmatrix}
0_{3n\times 3} & 0_{3n\times 3} & I_{3n}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t).
$$

在实际实现里，$K_p$ 与 $\Gamma$ 的分解不是唯一的；只要满足原文公式 (45) 即可。

---

## 7. 连续 - 离散混合实现（Algorithm 1）

论文最终推荐的工程实现不是纯连续注入，而是：

- IMU 连续传播
- 视觉在图像时刻离散更新

### 7.1 IMU 连续传播（Algorithm 1，第 5 - 10 行）

在两帧图像之间：

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
\dot{\hat{p}}_i = [\sigma_R]_\times \hat{p}_i
$$

$$
\dot{P} = A(t)P + PA^\top(t) + V(t).
$$

可以看到，传播阶段只用 IMU 和重力对齐，不注入视觉修正项 $\sigma_p$。

### 7.2 图像时刻计算视觉创新（Algorithm 1，第 12 行）

收到一帧双目图像后：

1. 从左右像素坐标计算 $\bar{y}_i^1,\bar{y}_i^2$（原文公式 (24)）
2. 用原文公式 (27) 构造每个地标的 $\Xi_i$
3. 用原文公式 (26) 构造每个地标的 $\tilde{y}_i$
4. 用原文公式 (28) 堆叠得到 $\sigma_p$
5. 用原文公式 (33) 构造 $C(t_k)$

### 7.3 图像时刻计算离散增益（Algorithm 1，第 13 行）

论文混合实现使用离散形式增益：

$$
L
=
P C^\top(t_k)\left(C(t_k) P C^\top(t_k) + Q(t_k)\right)^{-1}.
$$

它与连续增益 $L = P C^\top Q^{-1}$ 的思想一致，只是适配了“视觉按帧到来”的离散更新方式。

### 7.4 图像时刻状态更新（Algorithm 1，第 15 - 19 行）

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

也就是说，双目视觉到来时主要对

- 位置
- 速度
- 重力估计
- 地标位置

做一次离散校正，而姿态保持连续通道处理。

### 7.5 图像时刻的 $P$ 更新（Algorithm 1，第 20 行）

$$
P^+ = (I_{3n+6} - LC)P.
$$

它表示当前图像已经被吸收，不确定度被压缩。

---

## 8. 代码实现顺序总结

如果只看双目实现，建议严格按下面顺序组织代码：

1. 初始化 $\hat{R},\hat{p},\hat{v},\hat{g},\hat{p}_i,P$
2. 每个 IMU 时刻：
   - 计算 $\sigma_R = \hat{g}\times g$
   - 用原文公式 (46) - (50) 的无视觉项版本传播状态
   - 用原文公式 (35)、(37) 传播 $P$
3. 每帧双目图像到来时：
   - 用原文公式 (24) 从左右像素得到 $\bar{y}_i^1,\bar{y}_i^2$
   - 用原文公式 (27) 计算每个地标的 $\Xi_i$
   - 用原文公式 (26) 计算每个地标的 $\tilde{y}_i$
   - 用原文公式 (28) 堆叠得到 $\sigma_p$
   - 用原文公式 (33) 构造 $C(t_k)$
   - 计算离散增益 $L = P C^\top(CPC^\top + Q)^{-1}$
   - 用原文公式 (43) - (45) 分解出 $K_v,K_g,K_p,\Gamma$
   - 按 Algorithm 1 第 15 - 20 行更新 $\hat{p},\hat{v},\hat{g},\hat{p}_i,P$
4. 输出当前估计值：

$$
\hat{R},\ \hat{p},\ \hat{v},\ \hat{g},\ \hat{p}_i.
$$

这就是论文双目几何观测器在工程中的主计算链路。
# Stereo VIO Geometric Observer Algorithm

本文档仅整理论文 *Nonlinear Observer Design for Visual-Inertial Odometry* 中与**双目视觉**实现直接相关的理论框架，按实际编程顺序组织，忽略稳定性证明、引理与单目/相对位置测量。

---

## 1. 状态与系统模型

### 1.1 状态所在李群（公式 (4)）

系统扩展状态写在矩阵李群 \(SE_{3+n}(3)\) 上：

\[
SE_{3+n}(3)
:=
\left\{
X = M(R,x_1,x_2,x_3,x_L)
:\;
R \in SO(3),\;
x_1,x_2,x_3 \in \mathbb{R}^3,\;
x_L \in \mathbb{R}^{3 \times n}
\right\}.
\]

其中

\[
M(R,x_1,x_2,x_3,x_L)
:=
\begin{bmatrix}
R & x_1 & x_2 & x_3 & x_L \\
0_{(3+n)\times 3} & I_{3+n}
\end{bmatrix}.
\]

在 VIO 中取

\[
X = M(R,p,v,g,p_L),
\qquad
p_L = [p_1,\dots,p_n].
\]

这意味着代码里要维护的核心变量是：

- \(R\)：姿态
- \(p\)：位置
- \(v\)：速度
- \(g\)：辅助重力状态
- \(p_i\)：第 \(i\) 个地标位置

### 1.2 李代数块结构（公式 (5)）

\[
V(\Omega,x_1,x_2,x_3,x_L)
:=
\begin{bmatrix}
\Omega & x_1 & x_2 & x_3 & x_L \\
0_{(3+n)\times 3} & 0_{3+n}
\end{bmatrix},
\qquad
\Omega \in \mathfrak{so}(3).
\]

后续 IMU 群速度 \(V\) 和创新项 \(\Delta\) 都按这个块结构组织。

### 1.3 连续系统动力学（公式 (12) - (16)）

\[
\dot{R} = R[\omega^B]_\times
\tag{12}
\]

\[
\dot{p} = v
\tag{13}
\]

\[
\dot{v} = g + R a^B
\tag{14}
\]

\[
\dot{p}_i = 0
\tag{15}
\]

\[
\dot{g} = 0.
\tag{16}
\]

其中：

- \(\omega^B\) 是 IMU 机体系角速度
- \(a^B\) 是 IMU 机体系表观加速度
- \(g\) 是惯性系中的常值重力向量

### 1.4 群形式动力学（公式 (17)）

\[
\dot{X} = [X,H] + X V
\tag{17}
\]

其中李括号定义为

\[
[X_1,X_2] = X_1 X_2 - X_2 X_1,
\]

矩阵 \(H\) 定义为

\[
H =
\begin{bmatrix}
0_3 & 0_{3\times(3+n)} \\
0_{(3+n)\times 3} & S
\end{bmatrix},
\]

\[
S =
\begin{bmatrix}
0 & 0 & 0 & \cdots & 0 & 0 \\
1 & 0 & 0 & \cdots & 0 & 0 \\
0 & 1 & 0 & \cdots & 0 & 0 \\
0 & 0 & 0 & \cdots & 0 & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
0 & 0 & 0 & \cdots & 0 & 0
\end{bmatrix}
\in \mathbb{R}^{(3+n)\times(3+n)},
\]

而 IMU 输入对应的群速度为

\[
V
=
V\!\left(
[\omega^B]_\times,\;
0_{3\times 1},\;
a^B,\;
0_{3\times 1},\;
0_{3\times n}
\right).
\]

这一步的目的只是把 \((12)\) - \((16)\) 统一写进群结构中，便于后续观测器设计。

---

## 2. 双目测量模型

### 2.1 双目 bearing 观测（公式 (20)）

设 \((R_{c_1},p_{c_1})\)、\((R_{c_2},p_{c_2})\) 是从机体系 \(\{B\}\) 到左右相机坐标系 \(\{C_1\},\{C_2\}\) 的外参，则第 \(i\) 个地标在第 \(q\) 个相机中的单位视线向量为

\[
\bar{y}_i^q
=
\frac{
R_{c_q}^{\top}\!\left(R^\top (p_i-p)-p_{c_q}\right)
}{
\left\|
R_{c_q}^{\top}\!\left(R^\top (p_i-p)-p_{c_q}\right)
\right\|
},
\qquad q \in \{1,2\}.
\tag{20}
\]

### 2.2 群形式双目测量（公式 (21)）

\[
\bar{y}_i^q
=
\frac{
X_{c_q}^{-1}\!\left(X^{-1}r_i-p_{c_q}\right)
}{
\left\|
X_{c_q}^{-1}\!\left(X^{-1}r_i-p_{c_q}\right)
\right\|
},
\tag{21}
\]

其中

\[
X_{c_q} := M(R_{c_q},0_{3\times 1},0_{3\times 1},0_{3\times 1},0_{3\times n}),
\]

\[
r_i := \begin{bmatrix} 1 & 0 & 0 & -e_i^\top \end{bmatrix}^\top \in \mathbb{R}^{3+n},
\]

\(e_i\) 是 \(\mathbb{R}^n\) 的第 \(i\) 个标准基。

### 2.3 像素到 bearing（公式 (24)）

如果前端给的是像素坐标，则先通过相机内参 \(K\) 转成单位 bearing：

\[
\bar{y}_i
=
\frac{K^{-1} z_i}{\|K^{-1} z_i\|},
\qquad
z_i = [u_i,v_i,1]^\top .
\tag{24}
\]

对双目实现而言，这一步要分别在左右目上执行，得到 \(\bar{y}_i^1\) 与 \(\bar{y}_i^2\)。

---

## 3. 观测器总方程

### 3.1 连续观测器（公式 (25)）

\[
\dot{\hat{X}} = [\hat{X},H] + \hat{X}V + \Delta \hat{X}
\tag{25}
\]

其中

\[
\hat{X} = M(\hat{R},\hat{p},\hat{v},\hat{g},\hat{p}_L),
\qquad
\hat{p}_L = [\hat{p}_1,\dots,\hat{p}_n].
\]

这里 \(\Delta\) 是创新项，负责把视觉和重力信息注入估计器。

### 3.2 每个地标的创新项（公式 (26)）

论文先定义单地标创新 \(\tilde{y}_i\)，对双目测量有

\[
\tilde{y}_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q)\,
(\hat{X}^{-1}r_i - p_{c_q}),
\]

并统一写成

\[
\tilde{y}_i
=
\Xi_i\left(\hat{X}^{-1}r_i - X^{-1}r_i\right).
\tag{26}
\]

这里使用的投影算子为（公式 (3)）

\[
\pi(x) = I - \frac{x x^\top}{\|x\|^2},
\qquad x \neq 0.
\tag{3}
\]

### 3.3 双目对应的修正矩阵（公式 (27)）

对双目测量，

\[
\Xi_i
=
\sum_{q=1}^{2}
\pi(X_{c_q}\bar{y}_i^q).
\tag{27}
\]

它的含义是：左右两个相机分别提供一个 bearing 约束，再把它们加起来形成该地标的总方向约束。

### 3.4 总平移创新（公式 (28)）

定义选择矩阵

\[
Q := \begin{bmatrix} I_3 & 0_{3\times(3+n)} \end{bmatrix}.
\]

然后将所有地标创新项堆叠成总平移创新：

\[
\sigma_p
=
\left(
Q[\tilde{y}_1,\tilde{y}_2,\dots,\tilde{y}_n]
\right)^\vee .
\tag{28}
\]

这里 \((\cdot)^\vee\) 表示按列堆叠成向量，因此

\[
\sigma_p \in \mathbb{R}^{3n}.
\]

它是视觉更新阶段最核心的残差信号。

### 3.5 创新项总结构（公式 (29)）

\[
\Delta
=
V\!\left(
k_R[\sigma_R]_\times,\;
K_p(t)\sigma_p,\;
K_v(t)\sigma_p,\;
K_g(t)\sigma_p,\;
(\Gamma(t)\sigma_p)^\wedge
\right).
\tag{29}
\]

其中：

- \(\sigma_R\) 是旋转修正项
- \(\sigma_p\) 是视觉驱动的平移修正项
- \(K_p,K_v,K_g,\Gamma\) 是由 Riccati / Kalman 型增益分解得到的校正矩阵

---

## 4. 平移子系统

### 4.1 平移误差状态（公式 (31)）

论文定义平移误差状态为

\[
x =
\begin{bmatrix}
(R^\top \tilde{v})^\top \\
(R^\top \tilde{g})^\top \\
\left(R^\top(1 \otimes \tilde{p} - \tilde{p}_L)\right)^{\vee\top}
\end{bmatrix}^{\top}.
\tag{31}
\]

从实现角度看，它包含三部分：

- 机体系速度误差
- 机体系重力误差
- 机体系相对地标位置误差

### 4.2 平移创新与误差的关系（公式 (32)）

\[
\sigma_p = C(t)x.
\tag{32}
\]

这说明视觉创新 \(\sigma_p\) 可以看成平移误差状态的线性输出。

### 4.3 输出矩阵 \(C(t)\)（公式 (33)）

\[
C(t)
=
\begin{bmatrix}
0_3 & 0_3 & \Pi_1 & 0_3 & \cdots & 0_3 \\
0_3 & 0_3 & 0_3 & \Pi_2 & \cdots & 0_3 \\
\vdots & \vdots & \vdots & \vdots & \ddots & \vdots \\
0_3 & 0_3 & 0_3 & 0_3 & \cdots & \Pi_n
\end{bmatrix},
\tag{33}
\]

其中

\[
\Pi_i := Q \Xi_i Q^\top.
\]

对双目测量，\(\Xi_i\) 来自公式 \((27)\)。

### 4.4 平移误差闭环形式（公式 (34)）

\[
\dot{x} = (A(t) - L(t)C(t))x.
\tag{34}
\]

其中 \(A(t)\) 是传播矩阵，\(L(t)\) 是增益矩阵。

### 4.5 传播矩阵 \(A(t)\)（公式 (35) 与 (37)）

\[
A(t)
=
\begin{bmatrix}
D(t) & 0_{6\times 3n} \\
B_n \otimes I_3 & I_n \otimes (-[\omega^B]_\times)
\end{bmatrix},
\tag{35}
\]

其中

\[
D(t)
=
\begin{bmatrix}
-[\omega^B]_\times & I_3 \\
0_{3\times 3} & -[\omega^B]_\times
\end{bmatrix},
\qquad
B_n = \begin{bmatrix} 1^\top & 0_{n\times 1} \end{bmatrix}.
\tag{37}
\]

这一步在代码中用于传播 \(P(t)\)。

---

## 5. 旋转子系统

### 5.1 旋转修正项

论文选取

\[
\sigma_R := \hat{g} \times g.
\]

也就是说，姿态校正只依赖“估计重力方向”和“真实重力方向”的失配。

### 5.2 显式观测器形式（公式 (46) - (50)）

\[
\dot{\hat{R}}
=
\hat{R}[\omega^B + \hat{R}^\top \sigma_R]_\times
\tag{46}
\]

\[
\dot{\hat{p}}
=
[\sigma_R]_\times \hat{p}
 + \hat{v}
 + \hat{R}\sum_{j=1}^{n} K_j^p \sigma_j^p
\tag{47}
\]

\[
\dot{\hat{v}}
=
[\sigma_R]_\times \hat{v}
 + \hat{g}
 + \hat{R}a^B
 + \hat{R}\sum_{j=1}^{n} K_j^v \sigma_j^p
\tag{48}
\]

\[
\dot{\hat{g}}
=
[\sigma_R]_\times \hat{g}
 + \hat{R}\sum_{j=1}^{n} K_j^g \sigma_j^p
\tag{49}
\]

\[
\dot{\hat{p}}_i
=
[\sigma_R]_\times \hat{p}_i
 + \hat{R}\sum_{j=1}^{n} \Gamma_{ij}\sigma_j^p.
\tag{50}
\]

这些是最适合直接翻译成代码的连续状态方程。

---

## 6. Riccati / Kalman 型增益构造

### 6.1 连续 Riccati 方程（公式 (9)）

\[
\dot{P}(t)
=
A(t)P(t) + P(t)A^\top(t)
 - P(t)C^\top(t)Q^{-1}(t)C(t)P(t)
 + V(t),
\tag{9}
\]

其中：

- \(P(t)\) 是平移误差系统的矩阵变量
- \(Q(t)\) 是测量权重 / 噪声矩阵
- \(V(t)\) 是过程权重 / 噪声矩阵

### 6.2 连续增益

论文理论部分使用

\[
L(t) = P(t)C^\top(t)Q^{-1}(t).
\]

### 6.3 增益与校正矩阵的关系（公式 (36)）

\[
L(t)
=
\left(I_{n+2}\otimes \hat{R}^\top\right)
\begin{bmatrix}
K_v(t) \\
K_g(t) \\
1^\top \otimes K_p(t) - \Gamma(t)
\end{bmatrix}.
\tag{36}
\]

这说明：先算出总增益 \(L(t)\)，再由它分解出

- \(K_v(t)\)
- \(K_g(t)\)
- \(K_p(t)\)
- \(\Gamma(t)\)

### 6.4 从 \(L(t)\) 提取 \(K_v\)、\(K_g\)、\(K_p\)、\(\Gamma\)（公式 (43) - (45)）

\[
K_v(t)
=
\begin{bmatrix}
I_{3n} & 0_{3n\times 3} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t),
\tag{43}
\]

\[
K_g(t)
=
\begin{bmatrix}
0_{3n\times 3} & I_{3n} & 0_{3n\times 3}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t),
\tag{44}
\]

\[
1^\top \otimes K_p(t) - \Gamma(t)
=
\begin{bmatrix}
0_{3n\times 3} & 0_{3n\times 3} & I_{3n}
\end{bmatrix}
\left(I_{n+2}\otimes \hat{R}\right)L(t).
\tag{45}
\]

在实际实现里，\(K_p\) 与 \(\Gamma\) 的分解不是唯一的；只要满足公式 \((45)\) 即可。

---

## 7. 连续 - 离散混合实现（Algorithm 1）

论文最终推荐的工程实现不是纯连续注入，而是：

- IMU 连续传播
- 视觉在图像时刻离散更新

### 7.1 IMU 连续传播（Algorithm 1，第 5 - 10 行）

在两帧图像之间：

\[
\dot{\hat{R}} = \hat{R}[\omega^B + \hat{R}^\top \sigma_R]_\times
\]

\[
\dot{\hat{p}} = [\sigma_R]_\times \hat{p} + \hat{v}
\]

\[
\dot{\hat{v}} = [\sigma_R]_\times \hat{v} + \hat{g} + \hat{R}a^B
\]

\[
\dot{\hat{g}} = [\sigma_R]_\times \hat{g}
\]

\[
\dot{\hat{p}}_i = [\sigma_R]_\times \hat{p}_i
\]

\[
\dot{P} = A(t)P + PA^\top(t) + V(t).
\]

可以看到，传播阶段只用 IMU 和重力对齐，不注入视觉修正项 \(\sigma_p\)。

### 7.2 图像时刻计算视觉创新（Algorithm 1，第 12 行）

收到一帧双目图像后：

1. 从左右像素坐标计算 \(\bar{y}_i^1,\bar{y}_i^2\)（公式 \((24)\)）
2. 用公式 \((27)\) 构造每个地标的 \(\Xi_i\)
3. 用公式 \((26)\) 构造每个地标的 \(\tilde{y}_i\)
4. 用公式 \((28)\) 堆叠得到 \(\sigma_p\)
5. 用公式 \((33)\) 构造 \(C(t_k)\)

### 7.3 图像时刻计算离散增益（Algorithm 1，第 13 行）

论文混合实现使用离散形式增益：

\[
L
=
P C^\top(t_k)\left(C(t_k) P C^\top(t_k) + Q(t_k)\right)^{-1}.
\]

它与连续增益 \(L = P C^\top Q^{-1}\) 的思想一致，只是适配了“视觉按帧到来”的离散更新方式。

### 7.4 图像时刻状态更新（Algorithm 1，第 15 - 19 行）

\[
\hat{R}^+ = \hat{R}
\]

\[
\hat{p}^+ = \hat{p} + \hat{R}\sum_{j=1}^{n} K_j^p \sigma_j^p
\]

\[
\hat{v}^+ = \hat{v} + \hat{R}\sum_{j=1}^{n} K_j^v \sigma_j^p
\]

\[
\hat{g}^+ = \hat{g} + \hat{R}\sum_{j=1}^{n} K_j^g \sigma_j^p
\]

\[
\hat{p}_i^+ = \hat{p}_i + \hat{R}\sum_{j=1}^{n} \Gamma_{ij}\sigma_j^p
\]

也就是说，双目视觉到来时主要对

- 位置
- 速度
- 重力估计
- 地标位置

做一次离散校正，而姿态保持连续通道处理。

### 7.5 图像时刻的 \(P\) 更新（Algorithm 1，第 20 行）

\[
P^+ = (I_{3n+6} - LC)P.
\]

它表示当前图像已经被吸收，不确定度被压缩。

---

## 8. 代码实现顺序总结

如果只看双目实现，建议严格按下面顺序组织代码：

1. 初始化 \(\hat{R},\hat{p},\hat{v},\hat{g},\hat{p}_i,P\)
2. 每个 IMU 时刻：
   - 计算 \(\sigma_R = \hat{g}\times g\)
   - 用公式 \((46)\) - \((50)\) 的无视觉项版本传播状态
   - 用公式 \((35)\)、\((37)\) 传播 \(P\)
3. 每帧双目图像到来时：
   - 用公式 \((24)\) 从左右像素得到 \(\bar{y}_i^1,\bar{y}_i^2\)
   - 用公式 \((27)\) 计算每个地标的 \(\Xi_i\)
   - 用公式 \((26)\) 计算每个地标的 \(\tilde{y}_i\)
   - 用公式 \((28)\) 堆叠得到 \(\sigma_p\)
   - 用公式 \((33)\) 构造 \(C(t_k)\)
   - 计算离散增益 \(L = P C^\top(CPC^\top + Q)^{-1}\)
   - 用公式 \((43)\) - \((45)\) 分解出 \(K_v,K_g,K_p,\Gamma\)
   - 按 Algorithm 1 第 15 - 20 行更新 \(\hat{p},\hat{v},\hat{g},\hat{p}_i,P\)
4. 输出当前估计值：
   \[
   \hat{R},\ \hat{p},\ \hat{v},\ \hat{g},\ \hat{p}_i.
   \]

这就是论文双目几何观测器在工程中的主计算链路。
