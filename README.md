# IMU-GPS-Fusion
Sensor fusion using a particle filter based off of "Particle Filter Data Fusion Enhancements for MEMS-IMU/GPS" by afei Ren, Xizhen Ke. https://file.scirp.org/pdf/IIM20100700004_41189508.pdf

The particle filter is implemented using the SMCTC library. https://warwick.ac.uk/fac/sci/statistics/staff/academic-research/johansen/smctc/

正在收集工作区信息# 这个IMU-GPS融合粒子滤波器的工作原理

这个代码实现了一个基于粒子滤波的IMU和GPS传感器融合系统。我来解释一下数据流和整个系统的工作原理：

## 基本原理

粒子滤波是一种贝叶斯滤波方法，通过维护一组带权重的样本(粒子)来估计状态分布。这个系统融合了IMU(惯性测量单元)和GPS两种传感器数据来估计机器人/车辆的位置、速度、姿态等状态。

## 状态向量

系统状态向量包含15个分量：
- 0-2: 姿态角(欧拉角，表示方向)
- 3-5: 角速度
- 6-8: 位置(x, y, z)
- 9-11: 速度
- 12-14: 加速度

## 传感器数据流

1. **IMU数据**：从BNO055传感器读取
   - 提供了姿态角、角速度和加速度数据
   - 在`main.cpp`中通过`bno.getVector()`等函数读取数据
   - 然后通过`y_imu->SetMeasurement()`设置到`y_imu`全局变量中

2. **GPS数据**：从GPSD服务读取
   - 提供位置和速度数据
   - 在`main.cpp`中通过`gpsd_data->fix`读取
   - 通过`y_gps->SetMeasurement()`设置到`y_gps`全局变量中

## 粒子滤波工作流程

1. **初始化**：`fInitialise()`函数创建N个初始粒子
   - 每个粒子代表系统可能的状态
   - 初始状态基于IMU的初始读数加上随机噪声

2. **预测步骤**：分为两种移动模型
   - `IMUKernel()`：当有新的IMU数据时，使用IMU运动模型
   - `GPSKernel()`：当有新的GPS数据时，使用GPS运动模型
   - 运动模型基于上一时刻的状态进行预测，并添加过程噪声

3. **更新步骤**：
   - 通过似然函数计算每个粒子与实际测量的匹配程度
   - `logLikelihoodIMU()` 和 `logLikelihoodGPS()`计算粒子与测量的对数似然
   - 更新粒子的权重：`pFrom.AddToLogWeight(logLikelihood...(...))`

4. **重采样**：
   - 在`sampler::IterateEss()`中检查有效样本数量(ESS)
   - 当ESS低于阈值时进行重采样，保留高权重粒子，舍弃低权重粒子

5. **输出估计**：
   - 通过`sampler::Integrate()`计算粒子的加权平均值
   - 在`main.cpp`中通过`integrand_mean_x`等函数获取最终的位置估计

## 数据传入流程

1. 在`main.cpp`中创建全局变量：
   ```cpp
   GPS_obs * y_gps; 
   IMU_obs * y_imu;
   ```

2. 在主循环中读取传感器数据并更新这些全局变量：
   ```cpp
   // 更新IMU数据
   y_imu->SetMeasurement(Theta, Omega, Acc, timestamp_msec/1000.0);
   // 更新GPS数据
   y_gps->SetMeasurement(Pos, Velocity, xerror, yerror, zerror, serror, cerror, timestamp_msec/1000);
   ```

3. 然后执行粒子滤波迭代：
   ```cpp
   Sampler.Iterate();
   ```

4. `fSelect()`函数根据哪种数据新到来选择使用哪个内核(IMU或GPS)

5. 最后获取并输出估计结果：
   ```cpp
   double xm = Sampler.Integrate(integrand_mean_x, NULL);
   double ym = Sampler.Integrate(integrand_mean_y, NULL);
   ```

## 最终结果

粒子滤波的最终结果是对状态向量的估计，特别是位置(x,y,z)和姿态。在`main.cpp`中，它输出：
```cpp
cout << xm << "," << ym << "," << sqrt(xv) << "," << sqrt(yv) << endl;
```
这里输出了:
- xm, ym: x和y位置的估计值
- sqrt(xv), sqrt(yv): x和y位置估计的标准差，表示不确定性

通过融合IMU和GPS，系统可以提供比单独使用任一传感器更准确、更平滑的位置和姿态估计。

----

# 粒子滤波预测步骤详解

在IMU-GPS融合粒子滤波器中，预测步骤主要发生在`IMUKernel()`和`GPSKernel()`函数中。让我详细解释这一步骤的输入、参数和输出：

## 预测步骤的输入

1. **上一时刻的粒子状态**：
   - 从`pFrom.GetValue()`获取，包含15个状态分量（姿态角、角速度、位置、速度、加速度）
   - 这是上一次迭代后的粒子状态

2. **传感器测量数据**：
   - IMU数据：通过全局变量`y_imu`提供的姿态角、角速度和加速度
   - GPS数据：通过全局变量`y_gps`提供的位置和速度

3. **时间增量**：
   - 当前测量时间与上一次状态更新之间的时间差
   - 在IMU核函数中通过`y_imu->GetTime() - prev_x[i][15]`计算
   - 在GPS核函数中通过`y_gps->GetTime() - prev_x[i][15]`计算

## 预测步骤的参数

1. **过程噪声参数**：
   - `q_gyro`：陀螺仪噪声系数
   - `q_accel`：加速度计噪声系数
   - `q_pos`：位置噪声系数
   - `q_vel`：速度噪声系数

2. **随机扰动生成器**：
   - `Noise`：随机数生成器实例，用于添加状态转移噪声
   - 使用高斯分布生成随机噪声

3. **状态转移模型参数**：
   - 重力常数
   - 姿态转换矩阵
   - 坐标转换参数

## 预测步骤的具体实现

1. **IMU运动模型**(`IMUKernel()`):
   ```cpp
   // 计算时间增量
   double dt = y_imu->GetTime() - prev_x[i][15];
   
   // 更新姿态角（基于角速度和时间增量）
   Theta[0] = prev_x[i][0] + prev_x[i][3] * dt + Noise.Normal(0.0, q_gyro);
   Theta[1] = prev_x[i][1] + prev_x[i][4] * dt + Noise.Normal(0.0, q_gyro);
   Theta[2] = prev_x[i][2] + prev_x[i][5] * dt + Noise.Normal(0.0, q_gyro);
   
   // 更新角速度
   Omega[0] = prev_x[i][3] + Noise.Normal(0.0, q_gyro);
   Omega[1] = prev_x[i][4] + Noise.Normal(0.0, q_gyro);
   Omega[2] = prev_x[i][5] + Noise.Normal(0.0, q_gyro);
   
   // 更新位置（基于速度和时间增量）
   Pos[0] = prev_x[i][6] + prev_x[i][9] * dt + 0.5 * prev_x[i][12] * dt * dt + Noise.Normal(0.0, q_pos);
   Pos[1] = prev_x[i][7] + prev_x[i][10] * dt + 0.5 * prev_x[i][13] * dt * dt + Noise.Normal(0.0, q_pos);
   Pos[2] = prev_x[i][8] + prev_x[i][11] * dt + 0.5 * prev_x[i][14] * dt * dt + Noise.Normal(0.0, q_pos);
   
   // 更新速度（基于加速度和时间增量）
   Velocity[0] = prev_x[i][9] + prev_x[i][12] * dt + Noise.Normal(0.0, q_vel);
   Velocity[1] = prev_x[i][10] + prev_x[i][13] * dt + Noise.Normal(0.0, q_vel);
   Velocity[2] = prev_x[i][11] + prev_x[i][14] * dt + Noise.Normal(0.0, q_vel);
   
   // 更新加速度
   Acc[0] = prev_x[i][12] + Noise.Normal(0.0, q_accel);
   Acc[1] = prev_x[i][13] + Noise.Normal(0.0, q_accel);
   Acc[2] = prev_x[i][14] + Noise.Normal(0.0, q_accel);
   ```

2. **GPS运动模型**(`GPSKernel()`):
   - 类似于IMU模型，但更新方式基于GPS观测数据
   - 考虑GPS测量的位置和速度数据

## 预测步骤的输出

预测步骤的输出是一组更新后的粒子，每个粒子包含：

1. **更新后的状态向量**：
   - 16维向量，前15个分量是系统状态，最后一个是时间戳
   - 状态包括：姿态角(3)、角速度(3)、位置(3)、速度(3)、加速度(3)

2. **更新的时间戳**：
   - `prev_x[i][15] = y_imu->GetTime()`（IMU更新）
   - `prev_x[i][15] = y_gps->GetTime()`（GPS更新）

预测步骤结束后，这些更新后的粒子被传递给更新步骤，在那里它们的权重将根据粒子状态与实际传感器测量之间的匹配程度进行调整。

## 数据流向

1. **输入** → 上一时刻粒子状态 + 传感器数据 + 时间增量
2. **处理** → 应用运动模型 + 添加随机噪声
3. **输出** → 新的粒子状态，传递给更新步骤

这种预测-更新循环是粒子滤波器实现贝叶斯滤波的核心机制，通过随机粒子集合来近似表示状态的概率分布。

---
正在收集工作区信息# IMU和磁场数据融合的粒子滤波器实现方案

你可以成功地将这个IMU-GPS融合系统改造为IMU-磁场融合系统。这是完全可行的，因为粒子滤波的一个主要优势就是能处理非线性和不易求逆的观测模型。

## 需要修改的部分

### 1. 状态向量结构
可以保持原有的15维状态向量:
- 0-2: 姿态角
- 3-5: 角速度
- 6-8: 位置
- 9-11: 速度
- 12-14: 加速度

### 2. 观测模型修改
创建一个新的`MagneticField_obs`类替代`GPS_obs`类:

```cpp
class MagneticField_obs {
public:
  MagneticField_obs(double time);
  arma::vec measurementMag;  // 磁场测量值(Bx,By,Bz)
  arma::mat CovarianceMatrixR;  // 测量噪声协方差
  
  // 从位置计算预期磁场的函数
  arma::vec PredictMagField(arma::vec position);
  
  // 设置新的磁场测量
  void SetMeasurement(arma::vec MagField, double errorX, double errorY, double errorZ, double currentT);
  
  bool newData;
  double deltaT, currentTime, lastTime;
};
```

### 3. 似然函数
创建新的对数似然函数计算粒子权重:

```cpp
double logLikelihoodMagnetic(const cv_state & X, const MagneticField_obs & y_mag) {
  // 根据粒子位置预测磁场值
  arma::vec predictedMag = y_mag.PredictMagField(X.stateSpace.subvec(6,8));
  
  // 计算测量残差
  arma::mat temp = y_mag.measurementMag - predictedMag;
  
  // 计算马氏距离，得到对数似然
  arma::mat covi = arma::inv(y_mag.CovarianceMatrixR);
  arma::mat weight = arma::trans(temp) * covi * temp;
  
  return (-0.5 * weight(0,0));
}
```

### 4. 内核函数
创建新的磁场内核函数:

```cpp
void MagneticKernel(long lTime, smc::particle<cv_state> & pFrom, smc::rng *pRng) {
  cv_state * k = pFrom.GetValuePointer();
  
  // 运动学模型更新 (类似于IMUKernel，但可能需要调整噪声参数)
  k->stateSpace(0) = k->stateSpace(0) + y_imu->deltaT*k->stateSpace(3) + pRng->Normal(0,sqrt(1));
  // ...更新其他状态分量
  
  // 根据磁场似然更新粒子权重
  pFrom.AddToLogWeight(logLikelihoodMagnetic(*k, *y_mag));
}
```

### 5. 选择函数修改

```cpp
long fSelect(long lTime, const smc::particle<cv_state> & p, smc::rng *pRng) {
  if (y_mag->newData == 1) {
    return 1;  // 使用MagneticKernel
  }
  else if (y_imu->newData == 1) {
    return 0;  // 使用IMUKernel
  }
  else {
    return -1;
  }
}
```

## 关于磁场位置函数B=b(x)

**重要:** 你不需要求解逆函数来获取位置。这正是粒子滤波器的优势所在：

1. 对于每个粒子，系统知道其估计位置x
2. 通过函数b(x)计算该位置处的预期磁场值B_predicted
3. 将B_predicted与实际测量值B_measured比较
4. 差异越小的粒子获得越高的权重

## 磁场模型的实现方式

有几种方式实现磁场模型b(x):

1. **解析模型**: 如果磁场分布可以用数学公式表示
   ```cpp
   arma::vec PredictMagField(arma::vec pos) {
     // 例如:简单的磁偶极子模型
     double r = sqrt(pos(0)*pos(0) + pos(1)*pos(1) + pos(2)*pos(2));
     // ...计算磁场
     return B;
   }
   ```

2. **查找表+插值**: 如果有预先测量的磁场数据网格
   ```cpp
   arma::vec PredictMagField(arma::vec pos) {
     // 查找最近的网格点
     // 使用三线性插值获取估计磁场
     return interpolatedB;
   }
   ```

3. **数据驱动模型**: 使用预先训练的回归模型

## 总结与建议

1. 粒子滤波器完全可以用于融合IMU和磁场信息，无需求解从磁场到位置的逆问题

2. 关键是要提供准确的磁场模型b(x)，它可以预测任意位置的磁场

3. 需要根据你的磁场分布特性调整噪声参数，可能需要比GPS更多的粒子以保持多模态分布

4. 初始化时可能需要在更大范围内撒粒子，因为磁场可能存在多个位置产生相似的测量值

5. 注意磁场可能会受到环境变化影响，考虑增加一个在线校准机制

这种方法不仅适用于室内定位，也适用于存在磁异常的环境导航，只要你能建立合适的磁场-位置关系模型。

---

# IMU-磁场融合粒子滤波器实现方案

## 1. 系统数学模型

### 1.1 状态空间

定义系统状态向量 $\mathbf{x}_t \in \mathbb{R}^{15}$:

$$\mathbf{x}_t = [\boldsymbol{\theta}_t, \boldsymbol{\omega}_t, \mathbf{p}_t, \mathbf{v}_t, \mathbf{a}_t]^T$$

其中:
- $\boldsymbol{\theta}_t = [\theta_x, \theta_y, \theta_z]^T$ 是姿态角（欧拉角）
- $\boldsymbol{\omega}_t = [\omega_x, \omega_y, \omega_z]^T$ 是角速度
- $\mathbf{p}_t = [p_x, p_y, p_z]^T$ 是位置
- $\mathbf{v}_t = [v_x, v_y, v_z]^T$ 是速度
- $\mathbf{a}_t = [a_x, a_y, a_z]^T$ 是加速度

### 1.2 系统动态方程

状态转移方程为非线性函数 $\mathbf{x}_t = f(\mathbf{x}_{t-1}) + \mathbf{w}_t$，其中 $\mathbf{w}_t$ 是过程噪声。具体分量更新如下:

$$\boldsymbol{\theta}_t = \boldsymbol{\theta}_{t-1} + \boldsymbol{\omega}_{t-1} \cdot \Delta t + \mathbf{w}_{\theta}$$

$$\boldsymbol{\omega}_t = \boldsymbol{\omega}_{t-1} + \mathbf{w}_{\omega}$$

$$\mathbf{p}_t = \mathbf{p}_{t-1} + \mathbf{v}_{t-1} \cdot \Delta t + \frac{1}{2} \mathbf{a}_{t-1} \cdot \Delta t^2 + \mathbf{w}_{p}$$

$$\mathbf{v}_t = \mathbf{v}_{t-1} + \mathbf{a}_{t-1} \cdot \Delta t + \mathbf{w}_{v}$$

$$\mathbf{a}_t = \mathbf{a}_{t-1} + \mathbf{w}_{a}$$

其中 $\Delta t$ 是时间增量，$\mathbf{w}_{\theta}$, $\mathbf{w}_{\omega}$, $\mathbf{w}_{p}$, $\mathbf{w}_{v}$, $\mathbf{w}_{a}$ 是各状态分量的过程噪声。

### 1.3 观测模型

系统有两种观测:

1. **IMU观测**:
   $$\mathbf{z}^{IMU}_t = h_{IMU}(\mathbf{x}_t) + \mathbf{v}^{IMU}_t = [\boldsymbol{\theta}_t, \boldsymbol{\omega}_t, \mathbf{a}_t]^T + \mathbf{v}^{IMU}_t$$

2. **磁场观测**:
   $$\mathbf{z}^{MAG}_t = h_{MAG}(\mathbf{x}_t) + \mathbf{v}^{MAG}_t = b(\mathbf{p}_t) + \mathbf{v}^{MAG}_t$$

其中 $b(\mathbf{p})$ 是位置 $\mathbf{p}$ 处的磁场值，$\mathbf{v}^{IMU}_t$ 和 $\mathbf{v}^{MAG}_t$ 是观测噪声。

## 2. 粒子滤波算法

### 2.1 算法概述

粒子滤波通过随机粒子集合来表示状态的后验概率分布:

$$p(\mathbf{x}_t|\mathbf{z}_{1:t}) \approx \sum_{i=1}^{N} w_t^i \delta(\mathbf{x}_t - \mathbf{x}_t^i)$$

其中 $\mathbf{x}_t^i$ 是第 $i$ 个粒子，$w_t^i$ 是其权重，$\delta$ 是狄拉克函数。

### 2.2 算法步骤

1. **初始化**:
   - 生成 $N$ 个粒子 $\{\mathbf{x}_0^i, w_0^i\}_{i=1}^N$
   - 初始权重均匀分布: $w_0^i = \frac{1}{N}$

2. **预测**:
   - 对每个粒子，应用状态转移方程: $\mathbf{x}_t^i \sim p(\mathbf{x}_t|\mathbf{x}_{t-1}^i)$

3. **更新**:
   - 根据观测更新粒子权重: $\tilde{w}_t^i = w_{t-1}^i \cdot p(\mathbf{z}_t|\mathbf{x}_t^i)$
   - 权重归一化: $w_t^i = \frac{\tilde{w}_t^i}{\sum_{j=1}^N \tilde{w}_t^j}$

4. **重采样**:
   - 计算有效样本数 $N_{eff} = \frac{1}{\sum_{i=1}^N (w_t^i)^2}$
   - 若 $N_{eff} < N_{threshold}$，则进行重采样

5. **状态估计**:
   - 计算加权均值: $\hat{\mathbf{x}}_t = \sum_{i=1}^N w_t^i \mathbf{x}_t^i$

## 3. IMU-磁场融合实现

### 3.1 IMU核函数（预测步骤）

当新的IMU数据到来时:

```
IMUKernel(粒子 p, 时间 t):
    获取粒子当前状态 x = p.GetValue()
    计算时间增量 dt = t_current - x.timestamp
    
    // 更新状态
    x.θ = x.θ + x.ω * dt + Normal(0, q_gyro)
    x.ω = x.ω + Normal(0, q_gyro)
    x.p = x.p + x.v * dt + 0.5 * x.a * dt² + Normal(0, q_pos)
    x.v = x.v + x.a * dt + Normal(0, q_vel)
    x.a = x.a + Normal(0, q_accel)
    x.timestamp = t_current
    
    // 更新权重（如果同时有IMU测量值）
    p.AddToLogWeight(logLikelihoodIMU(x, y_imu))
```

### 3.2 磁场核函数（更新步骤）

当新的磁场数据到来时:

```
MagneticKernel(粒子 p, 时间 t):
    获取粒子当前状态 x = p.GetValue()
    计算时间增量 dt = t_current - x.timestamp
    
    // 更新状态（同IMU核函数）
    x.θ = x.θ + x.ω * dt + Normal(0, q_gyro)
    x.ω = x.ω + Normal(0, q_gyro)
    x.p = x.p + x.v * dt + 0.5 * x.a * dt² + Normal(0, q_pos)
    x.v = x.v + x.a * dt + Normal(0, q_vel)
    x.a = x.a + Normal(0, q_accel)
    x.timestamp = t_current
    
    // 根据磁场似然更新权重
    p.AddToLogWeight(logLikelihoodMagnetic(x, y_mag))
```

### 3.3 对数似然函数

对于磁场更新:

```
logLikelihoodMagnetic(状态 x, 观测 z_mag):
    // 根据粒子位置预测磁场值
    B_predicted = b(x.p)  // 使用位置-磁场映射函数
    
    // 计算测量残差
    residual = z_mag.B - B_predicted
    
    // 计算马氏距离
    mahalanobis = residual' * inv(R_mag) * residual
    
    // 返回对数似然
    return -0.5 * mahalanobis
```

对于IMU更新:

```
logLikelihoodIMU(状态 x, 观测 z_imu):
    // 计算姿态、角速度和加速度的残差
    residual_θ = z_imu.θ - x.θ
    residual_ω = z_imu.ω - x.ω
    residual_a = z_imu.a - x.a
    
    // 计算总残差向量
    residual = [residual_θ; residual_ω; residual_a]
    
    // 计算马氏距离
    mahalanobis = residual' * inv(R_imu) * residual
    
    // 返回对数似然
    return -0.5 * mahalanobis
```

### 3.4 磁场模型 b(p)

磁场模型有三种可能的实现方式:

1. **解析模型**:
   ```
   PredictMagField(位置 p):
       // 使用物理模型（如多极子模型）计算磁场
       B = computePhysicalModel(p)
       return B
   ```

2. **查找表+插值**:
   ```
   PredictMagField(位置 p):
       // 找到周围的网格点
       [idx_x1, idx_y1, idx_z1] = findLowerGridPoint(p)
       [idx_x2, idx_y2, idx_z2] = findUpperGridPoint(p)
       
       // 计算插值权重
       wx = (p.x - grid_x[idx_x1]) / (grid_x[idx_x2] - grid_x[idx_x1])
       wy = (p.y - grid_y[idx_y1]) / (grid_y[idx_y2] - grid_y[idx_y1])
       wz = (p.z - grid_z[idx_z1]) / (grid_z[idx_z2] - grid_z[idx_z1])
       
       // 三线性插值
       B = trilinearInterpolation(magneticFieldGrid, wx, wy, wz,
                                idx_x1, idx_y1, idx_z1, 
                                idx_x2, idx_y2, idx_z2)
       return B
   ```

3. **数据驱动模型**:
   ```
   PredictMagField(位置 p):
       // 使用训练好的回归模型预测磁场
       B = regressionModel.predict(p)
       return B
   ```

## 4. 初始化与参数设置

### 4.1 粒子初始化

初始粒子分布应覆盖可能的位置区域:

```
InitializeParticles(数量 N):
    particles = []
    
    for i = 1 to N:
        // 根据先验知识或初始IMU读数确定初始状态
        θ_init = y_imu.θ + Normal(0, init_θ_std)
        ω_init = y_imu.ω + Normal(0, init_ω_std)
        a_init = y_imu.a + Normal(0, init_a_std)
        
        // 位置和速度可能需要更大的初始不确定性
        p_init = initialPosGuess + Normal(0, init_p_std)
        v_init = initialVelGuess + Normal(0, init_v_std)
        
        // 创建粒子
        particle = createParticle([θ_init, ω_init, p_init, v_init, a_init])
        particles.append(particle)
    
    return particles
```

### 4.2 噪声参数设置

过程噪声协方差矩阵需要根据IMU设备的精度特性设置:
- $q_{\theta}$: 姿态角噪声方差
- $q_{\omega}$: 角速度噪声方差
- $q_{p}$: 位置噪声方差
- $q_{v}$: 速度噪声方差
- $q_{a}$: 加速度噪声方差

观测噪声协方差矩阵需要根据传感器精度设置:
- $R_{IMU}$: IMU观测噪声协方差矩阵
- $R_{MAG}$: 磁场观测噪声协方差矩阵

## 5. 高级实现细节

### 5.1 处理姿态角奇异性

欧拉角表示可能遇到万向锁问题，处理方法:
```
// 确保欧拉角在正确范围内
normalizeEulerAngles(θ):
    θ.x = normalizeAngle(θ.x)
    θ.y = normalizeAngle(θ.y)
    θ.z = normalizeAngle(θ.z)
    return θ
```

### 5.2 自适应重采样

设置自适应重采样阈值:
```
// 计算有效样本数
calculateEffectiveSampleSize(weights):
    return 1 / sum(weights^2)
    
// 自适应重采样
if calculateEffectiveSampleSize(weights) < N * threshold:
    particles = resample(particles, weights)
    // 重置权重
    for each particle in particles:
        particle.weight = 1/N
```

### 5.3 多模态处理

如果磁场映射存在多个相似位置:
```
// 检测多模态分布
detectModes(particles):
    // 使用聚类算法检测粒子团
    clusters = performClustering(particles.positions)
    return clusters

// 如果检测到多个模式，可以分别跟踪
if len(detectModes(particles)) > 1:
    // 特殊处理多模态情况，例如增加粒子数量或使用多个滤波器
```

## 6. 完整算法流程

