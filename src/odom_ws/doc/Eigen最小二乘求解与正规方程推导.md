# Eigen 最小二乘求解与正规方程推导

> 对应代码：`src/calib_odom/src/Odom_Calib.cpp` 中的 `OdomCalib::Solve()`  
> 目标：理解 `topRows()`、`rank()`、`solve()`、`allFinite()` 的作用，以及最小二乘公式 `(AᵀA)⁻¹Aᵀb` 的由来。

---

## 1. 先看完整代码

```cpp
Eigen::Matrix3d OdomCalib::Solve()
{
    if(now_len == 0)
    {
        throw std::runtime_error("Cannot solve odometry calibration without samples.");
    }

    const Eigen::Index rows = static_cast<Eigen::Index>(now_len) * 3;
    const Eigen::MatrixXd observations = A.topRows(rows);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(observations);
    if(decomposition.rank() < 9)
    {
        throw std::runtime_error(
            "Cannot solve odometry calibration: the collected motions do not identify all 9 matrix parameters.");
    }

    const Eigen::VectorXd solution = decomposition.solve(b.head(rows));
    if(!solution.allFinite())
    {
        throw std::runtime_error("Cannot solve odometry calibration: least-squares result is not finite.");
    }

    Eigen::Matrix3d correct_matrix;
    correct_matrix.row(0) = solution.segment<3>(0).transpose();
    correct_matrix.row(1) = solution.segment<3>(3).transpose();
    correct_matrix.row(2) = solution.segment<3>(6).transpose();
    return correct_matrix;
}
```

这段代码的目标是从已经收集的观测数据中，求出里程计校正矩阵：

```text
M = [m00  m01  m02
     m10  m11  m12
     m20  m21  m22]
```

它满足的拟合模型是：

```text
scan ≈ M · odom
```

其中：

```text
odom = [ox, oy, oθ]ᵀ：轮式里程计得到的相对位姿增量
scan = [sx, sy, sθ]ᵀ：激光 PI-ICP 匹配得到的相对位姿增量
M                         ：待估计的 3×3 校正矩阵
```

将 `M` 按行展开为长度为 9 的未知参数向量：

```text
x = [m00, m01, m02, m10, m11, m12, m20, m21, m22]ᵀ
```

多组样本叠加后得到：

```text
A x ≈ b
```

代码要做的就是求最小二乘意义下最好的 `x`，再将它还原成矩阵 `M`。

---

## 2. `now_len == 0`：为什么没有样本不能求解

```cpp
if(now_len == 0)
{
    throw std::runtime_error("Cannot solve odometry calibration without samples.");
}
```

`now_len` 是已经成功写入方程组的有效样本组数。

每一组样本对应：

```text
一个 odom 相对位姿增量
+
一个 scan 相对位姿增量
```

并贡献三条标量方程：

```text
scan_x     ≈ M 的第 0 行 · odom
scan_y     ≈ M 的第 1 行 · odom
scan_yaw   ≈ M 的第 2 行 · odom
```

如果：

```text
now_len = 0
```

就没有任何 `A` 和 `b` 中的真实观测数据。因此没有方程，也没有办法推断 9 个未知参数。

---

## 3. `rows = now_len * 3`：为什么每个样本占三行

```cpp
const Eigen::Index rows = static_cast<Eigen::Index>(now_len) * 3;
```

一个样本的原始模型是：

```text
scan = M · odom
```

其中两边都是三维向量：

```text
scan = [sx, sy, sθ]ᵀ
odom = [ox, oy, oθ]ᵀ
```

展开后有三条标量方程：

```text
sx ≈ m00·ox + m01·oy + m02·oθ
sy ≈ m10·ox + m11·oy + m12·oθ
sθ ≈ m20·ox + m21·oy + m22·oθ
```

所以：

```text
1 组有效样本 → 3 行方程
N 组有效样本 → 3N 行方程
```

例如：

```text
now_len = 100
rows    = 100 × 3 = 300
```

那么这一次最小二乘实际使用：

```text
A：300 × 9
x：  9 × 1
b：300 × 1
```

`Eigen::Index` 是 Eigen 用来表示矩阵行、列索引和尺寸的整数类型。这里使用 `static_cast` 是为了将普通整数 `now_len` 显式转换为 Eigen 所需的索引类型。

---

## 4. `topRows(rows)`：只取已经写入的有效方程

代码：

```cpp
const Eigen::MatrixXd observations = A.topRows(rows);
```

`topRows(n)` 的意思是：

```text
从矩阵最上方开始，取前 n 行。
```

### 4.1 为什么不能直接用完整的 `A`

程序启动时，预先分配最大容量：

```cpp
Odom_calib.Set_data_len(12000);
```

所以完整矩阵的尺寸为：

```text
A：36000 × 9
b：36000 × 1
```

因为：

```text
12000 组样本 × 每组 3 条方程 = 36000 行
```

但“预先分配空间”不代表每一行都有有效数据。

例如当前只采集了 100 组样本：

```text
now_len = 100
rows = 300
```

真正有意义的只有：

```text
A 的第 0 到第 299 行
b 的第 0 到第 299 个元素
```

后面的预分配空间仍是初始化时的零，不应参与求解。

因此：

```cpp
A.topRows(rows)
```

表示：

```text
只取 A 中已被 Add_Data() 写入的前 3N 行。
```

### 4.2 小例子

设完整的 `A` 预留了 12 行，但当前只有 2 组样本：

```text
A = [第 0 行：第 0 组样本的方程 1
     第 1 行：第 0 组样本的方程 2
     第 2 行：第 0 组样本的方程 3
     第 3 行：第 1 组样本的方程 1
     第 4 行：第 1 组样本的方程 2
     第 5 行：第 1 组样本的方程 3
     第 6 行：尚未使用，值为 0
     ...
     第11行：尚未使用，值为 0]
```

此时：

```cpp
A.topRows(6)
```

只取：

```text
第 0 行到第 5 行
```

也就是两组真实观测对应的六条方程。

---

## 5. `b.head(rows)`：取得右侧对应的有效观测值

代码：

```cpp
decomposition.solve(b.head(rows));
```

`A` 是二维矩阵，所以使用：

```cpp
A.topRows(rows)
```

而 `b` 是一维列向量，因此使用：

```cpp
b.head(rows)
```

`head(n)` 的意思是：

```text
取向量开头连续的 n 个元素。
```

如果已采集 `N` 组样本，则：

```text
b.head(3N)
```

取得：

```text
b = [sx₀, sy₀, sθ₀,
     sx₁, sy₁, sθ₁,
     ...,
     sxₙ₋₁, syₙ₋₁, sθₙ₋₁]ᵀ
```

因此：

```text
A.topRows(rows) ：方程左侧
b.head(rows)    ：方程右侧
```

两者行数相同，能够组成完整的：

```text
A x ≈ b
```

---

## 6. `CompleteOrthogonalDecomposition`：为什么需要矩阵分解

代码：

```cpp
Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(observations);
```

含义是：

```text
对有效观测矩阵 observations（即 A）做完全正交分解。
```

它不是直接计算矩阵逆，而是将矩阵拆分成具有良好数值性质的正交结构，然后完成最小二乘求解和秩判断。

本作业通常是超定方程组：

```text
A：3N × 9
x：  9 × 1
b：3N × 1
```

只要样本数大于 3，通常就有：

```text
3N > 9
```

即：

```text
方程数 > 未知数个数
```

此时一般不存在一个 `x`，能让每一条带噪声的方程严格满足 `Ax=b`；真正的目标是：

```text
min ||Ax-b||²
```

也就是找到使整体误差平方和最小的参数向量 `x`。

---

## 7. `rank()`：样本是否足以确定 9 个参数

代码：

```cpp
if(decomposition.rank() < 9)
{
    throw std::runtime_error(...);
}
```

### 7.1 什么是矩阵秩

矩阵的秩（rank）可以直观理解成：

```text
矩阵中有多少个相互独立的信息方向。
```

本作业的未知量为：

```text
x = [m00, m01, m02, m10, m11, m12, m20, m21, m22]ᵀ
```

因此有：

```text
9 个未知参数。
```

要唯一确定这 9 个参数，必须希望：

```text
rank(A) = 9
```

这称为 `A` 满列秩（full column rank）。

### 7.2 为什么秩不足时不能相信结果

若：

```text
rank(A) < 9
```

说明采集的运动数据有重复、缺少变化，不能独立约束所有参数。

例如机器人只做直线前进，每组里程计运动都近似：

```text
odom = [某个正数, 0, 0]ᵀ
```

那么只包含前进信息，无法可靠估计：

```text
与横向运动有关的参数
与角度变化有关的参数
平移、旋转之间的耦合参数
```

这时求解器可能仍能产生一个数值矩阵，但会有很多不同的 `M` 同样能够解释现有数据；返回的只是某种数学上的选择，而非可靠的物理标定结果。

所以代码明确拒绝：

```cpp
rank(A) < 9
```

### 7.3 如何减少秩亏风险

采样时应尽量覆盖：

```text
直线前进
后退
左转
右转
弧线运动
不同转弯半径
不同速度
```

这样 `[ox, oy, oθ]` 的变化会更丰富，观测矩阵更可能达到：

```text
rank(A) = 9
```

---

## 8. `solve()`：实际计算最小二乘解

代码：

```cpp
const Eigen::VectorXd solution = decomposition.solve(b.head(rows));
```

这里：

```text
observations = A.topRows(rows)
b.head(rows)   = 有效观测向量
solution       = 待求参数向量 x
```

`solve()` 求解的是：

```text
min ||A x-b||²
```

输出的 `solution` 是：

```text
solution = [m00, m01, m02, m10, m11, m12, m20, m21, m22]ᵀ
```

也就是校正矩阵 `M` 的 9 个元素。

`solve()` 并不要求每一行都严格满足：

```text
Ax = b
```

它会在所有可能的参数向量中，选择使残差平方和最小的那个。

---

## 9. `allFinite()`：检查结果有没有非法浮点数

代码：

```cpp
if(!solution.allFinite())
{
    throw std::runtime_error("Cannot solve odometry calibration: least-squares result is not finite.");
}
```

`allFinite()` 的意思是：

```text
向量中的每一个元素都必须是有限浮点数。
```

以下是正常、有限的数字：

```text
0
-0.25
1.0
3.1415926
```

以下是不正常、非有限的浮点数：

```text
NaN   ：Not a Number，非数值
+Inf  ：正无穷
-Inf  ：负无穷
```

如果解中出现 `NaN` 或 `Inf`，继续计算：

```cpp
correct_matrix * odom_inc
```

会使后续路径也被污染成无效数值，RViz 中可能无法正常显示。因此这一步是数值安全检查。

可能原因包括：

```text
输入观测本身包含 NaN 或 Inf
传感器或扫描匹配结果异常
某处计算出现非法浮点运算
矩阵数值条件极差
```

---

## 10. `solution` 怎样还原成 `correct_matrix`

求解得到的 `solution` 是长度为 9 的列向量：

```text
solution = [x0, x1, x2, x3, x4, x5, x6, x7, x8]ᵀ
```

代码：

```cpp
correct_matrix.row(0) = solution.segment<3>(0).transpose();
correct_matrix.row(1) = solution.segment<3>(3).transpose();
correct_matrix.row(2) = solution.segment<3>(6).transpose();
```

对应的恢复结果是：

```text
correct_matrix = [x0  x1  x2
                  x3  x4  x5
                  x6  x7  x8]
```

也就是：

```text
M = [m00  m01  m02
     m10  m11  m12
     m20  m21  m22]
```

这里：

```cpp
solution.segment<3>(0)
```

取出的是 `3×1` 列向量：

```text
[x0]
[x1]
[x2]
```

而：

```cpp
correct_matrix.row(0)
```

是 `1×3` 行向量区域：

```text
[x0  x1  x2]
```

所以要使用 `.transpose()` 将列向量排成行向量后再赋值。

---

# 11. 最小二乘公式 `(AᵀA)⁻¹Aᵀb` 如何推导

下面推导常见的最小二乘闭式解：

```text
x = (AᵀA)⁻¹Aᵀb
```

---

## 11.1 从 `Ax=b` 到 `Ax≈b`

若没有噪声且方程恰好可解，希望得到：

```text
Ax = b
```

但本作业中：

```text
A：3N × 9
x：9 × 1
b：3N × 1
```

通常方程数远大于未知数数目；同时里程计和激光匹配都有噪声。因此一般不存在某个 `x`，能使所有方程严格成立。

更准确的关系是：

```text
Ax ≈ b
```

定义残差向量：

```text
r = Ax-b
```

其中每一个元素都是“一条方程的预测值与实际观测值之差”。

---

## 11.2 为什么最小化残差平方和

最小二乘法定义目标函数：

```text
J(x) = ||Ax-b||²
```

`||·||` 表示向量的二范数（长度），平方后就是所有残差的平方和：

```text
J(x)
= (Ax-b)ᵀ(Ax-b)
= r₁² + r₂² + r₃² + ... + r₃N²
```

例如：

```text
r = [0.1, -0.2, 0.05]ᵀ
```

则：

```text
||r||²
= 0.1² + (-0.2)² + 0.05²
= 0.0525
```

我们的目标是：

```text
找到一个 x，使 J(x) 最小。
```

平方有两个重要效果：

```text
正负误差不会相互抵消；
较大的误差会受到更强的惩罚。
```

---

## 11.3 展开目标函数

从：

```text
J(x) = (Ax-b)ᵀ(Ax-b)
```

开始展开：

```text
J(x)
= (Ax)ᵀ(Ax) - (Ax)ᵀb - bᵀ(Ax) + bᵀb
```

利用：

```text
(Ax)ᵀ = xᵀAᵀ
```

可得到：

```text
J(x)
= xᵀAᵀAx - xᵀAᵀb - bᵀAx + bᵀb
```

其中：

```text
xᵀAᵀb
```

和：

```text
bᵀAx
```

都是 `1×1` 标量，互为转置，数值相同。因此可合并为：

```text
J(x) = xᵀAᵀAx - 2bᵀAx + bᵀb
```

最后的：

```text
bᵀb
```

不包含 `x`，对求最优 `x` 没有影响，它只是一个常数。

---

## 11.4 对 `x` 求导

最小值点满足：

```text
∂J(x) / ∂x = 0
```

对：

```text
J(x) = xᵀAᵀAx - 2bᵀAx + bᵀb
```

求导，得到：

```text
∂J(x) / ∂x = 2AᵀAx - 2Aᵀb
```

令导数为零：

```text
2AᵀAx - 2Aᵀb = 0
```

两边同时除以 2：

```text
AᵀAx = Aᵀb
```

这就是最小二乘的**正规方程**（normal equation）。

---

## 11.5 从正规方程得到闭式解

若：

```text
AᵀA 可逆
```

则从：

```text
AᵀAx = Aᵀb
```

两边左乘 `(AᵀA)⁻¹`：

```text
(AᵀA)⁻¹AᵀAx = (AᵀA)⁻¹Aᵀb
```

因为：

```text
(AᵀA)⁻¹(AᵀA) = I
```

所以：

```text
x = (AᵀA)⁻¹Aᵀb
```

这就是最小二乘的经典闭式解。

---

## 12. 几何意义：`Ax` 是 `b` 的最佳投影

由正规方程：

```text
AᵀAx = Aᵀb
```

移项可得：

```text
Aᵀ(Ax-b) = 0
```

又因为：

```text
r = Ax-b
```

所以：

```text
Aᵀr = 0
```

这个式子表示：

```text
最优解时，残差 r 与 A 的每一列都正交。
```

直观理解：

```text
b  ：所有激光观测叠成的高维向量
Ax ：由校正参数预测出来、且能够由 A 表示的向量
r  ：b 与 Ax 之间的误差向量
```

最优的 `Ax` 是 `b` 在 `A` 列空间上的最佳投影。误差 `r` 已经垂直于所有可继续调整的方向，因此再改变参数 `x` 也无法进一步缩短这个误差。

---

## 13. 为什么代码不直接写 `(AᵀA)⁻¹Aᵀb`

虽然数学推导得到：

```text
x = (AᵀA)⁻¹Aᵀb
```

但工程代码不应直接写：

```cpp
(A.transpose() * A).inverse() * A.transpose() * b
```

原因如下：

| 问题 | 后果 |
|---|---|
| 形成 `AᵀA` 会放大条件数 | 原本接近相关、接近秩亏的数据会更不稳定 |
| 显式计算 `.inverse()` | 通常数值稳定性更差，也没有必要 |
| `rank(A) < 9` 时，`AᵀA` 不可逆 | 无法获得唯一的 9 参数解 |
| 传感器数据有浮点误差 | 直接公式更容易产生不可靠结果 |

因此实际代码采用：

```cpp
Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> decomposition(observations);
const Eigen::VectorXd solution = decomposition.solve(b.head(rows));
```

它完成的数学目标仍然是：

```text
min ||Ax-b||²
```

但通过正交分解实现，通常更稳定，也能通过：

```cpp
decomposition.rank()
```

检查观测数据是否足以确定全部 9 个参数。

---

## 14. 一句话总结

```text
topRows(rows)：只取 A 中已经写入的前 3N 条有效方程；
head(rows)   ：取 b 中与这些方程一一对应的前 3N 个观测值；
rank()       ：检查样本是否足以独立确定 9 个矩阵参数，要求 rank(A)=9；
solve()      ：通过正交分解求 min ||Ax-b||² 的最小二乘解；
allFinite()  ：检查结果中没有 NaN、+Inf、-Inf 等无效数值。

(AᵀA)⁻¹Aᵀb 来自最小化 J(x)=||Ax-b||²：
对 J(x) 求导并令导数为零，得到正规方程 AᵀAx=Aᵀb；
当 AᵀA 可逆时，进一步得到 x=(AᵀA)⁻¹Aᵀb。
```
