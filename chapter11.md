# Chapter 11: 多传感器融合算法

在现实应用中，单一传感器往往无法满足复杂环境下的感知需求。激光雷达虽然具有高精度测距能力，但在纹理信息、速度测量、全天候工作等方面存在局限。本章将详细介绍激光雷达与其他传感器的融合算法，包括相机、毫米波雷达、IMU和GPS的融合方法，以及在实际应用中的标定、同步和数据关联技术。通过多传感器融合，我们可以实现互补优势，提高感知系统的鲁棒性和准确性。

## 11.1 激光雷达+相机融合

激光雷达与相机的融合是最常见的多传感器组合，结合了激光雷达的精确深度信息和相机的丰富纹理信息。这种融合在自动驾驶、机器人导航和三维重建等领域具有广泛应用。

### 11.1.1 投影矩阵与坐标转换

将激光雷达点云投影到相机图像需要经过多个坐标系转换，理解这些转换的数学原理至关重要。

**坐标系定义：**
- 激光雷达坐标系：X轴向前，Y轴向左，Z轴向上（右手系）
- 相机坐标系：X轴向右，Y轴向下，Z轴向前（光轴方向）
- 图像坐标系：原点在左上角，u轴向右，v轴向下

**激光雷达坐标系到相机坐标系：**
```
P_camera = R_lidar_to_camera · P_lidar + t_lidar_to_camera
```

其中R是3×3旋转矩阵，t是3×1平移向量。旋转矩阵可通过欧拉角或四元数表示：

欧拉角表示（ZYX顺序）：
```
R = R_z(yaw) · R_y(pitch) · R_x(roll)

R_x(φ) = [1    0       0    ]
         [0  cos(φ) -sin(φ)]
         [0  sin(φ)  cos(φ)]

R_y(θ) = [ cos(θ)  0  sin(θ)]
         [   0     1    0   ]
         [-sin(θ)  0  cos(θ)]

R_z(ψ) = [cos(ψ) -sin(ψ)  0]
         [sin(ψ)  cos(ψ)  0]
         [  0       0      1]
```

**相机坐标系到图像坐标系：**
透视投影模型：
```
λ[u]   [fx  s  cx] [X_camera]
 [v] = [0  fy  cy] [Y_camera]
 [1]   [0   0   1] [Z_camera]
```

其中：
- fx, fy：焦距（像素单位）
- cx, cy：主点坐标
- s：倾斜参数（通常为0）
- λ = Z_camera：深度值

**畸变校正：**
实际相机存在径向和切向畸变：
```
径向畸变：
x_distorted = x(1 + k1·r² + k2·r⁴ + k3·r⁶)
y_distorted = y(1 + k1·r² + k2·r⁴ + k3·r⁶)

切向畸变：
x_distorted = x + 2p1·xy + p2(r² + 2x²)
y_distorted = y + p1(r² + 2y²) + 2p2·xy
```

其中r² = x² + y²，(x,y)是归一化相机坐标。

**完整投影流程：**
1. 3D点从激光雷达坐标系转换到相机坐标系
2. 检查点是否在相机前方（Z_camera > 0）
3. 归一化：x = X_camera/Z_camera, y = Y_camera/Z_camera
4. 应用畸变模型
5. 投影到像素坐标

**计算实例1（标准情况）：**
激光雷达点P_lidar = [10, 2, -0.5]^T (m)，外参标定结果：
- 旋转角：roll=0°, pitch=5°, yaw=0°
- 平移：t = [0.1, 0, 0.2]^T (m)
- 相机内参：fx=fy=1000, cx=640, cy=360
- 畸变系数：k1=-0.1, k2=0.05, p1=p2=0

计算过程：
1. 构建旋转矩阵（pitch=5°≈0.0873rad）：
   ```
   R = [1      0         0     ]
       [0   0.9962   -0.0872 ]
       [0   0.0872    0.9962 ]
   ```

2. 转换到相机坐标系：
   ```
   P_camera = R·P_lidar + t
            = [10.1, 2.036, 0.674]^T
   ```

3. 归一化坐标：
   ```
   x = 10.1/0.674 = 14.985
   y = 2.036/0.674 = 3.021
   ```

4. 畸变校正：
   ```
   r² = 14.985² + 3.021² = 233.65
   畸变因子 = 1 + (-0.1)×233.65 + 0.05×233.65² = -729.8
   ```
   
   极端畸变表明点在图像边缘，需要更合理的参数。

**计算实例2（实际车载场景）：**
激光雷达检测到前方车辆点P_lidar = [20, -1.5, 0.8]^T：
- 外参：roll=0.5°, pitch=-2°, yaw=1°
- 平移：t = [0.05, -0.08, 0.15]^T
- 内参：fx=1200, fy=1200, cx=960, cy=540

详细计算：
1. 组合旋转矩阵：
   ```
   R = [ 0.9997  -0.0175   0.0174]
       [ 0.0178   0.9996  -0.0087]
       [-0.0171   0.0093   0.9998]
   ```

2. 坐标转换：
   ```
   P_camera = [20.02, -1.45, 0.51]^T
   ```

3. 投影（忽略畸变）：
   ```
   u = 1200×20.02/0.51 + 960 = 48082
   v = 1200×(-1.45)/0.51 + 540 = -2872
   ```

点在图像范围外，说明需要更大的视场角相机。

**边界条件处理：**
```python
def project_lidar_to_image(P_lidar, K, R, t, dist_coeffs, img_width, img_height):
    # 转换到相机坐标系
    P_cam = R @ P_lidar + t
    
    # 检查深度
    if P_cam[2] <= 0:
        return None  # 点在相机后方
    
    # 归一化
    x, y = P_cam[0]/P_cam[2], P_cam[1]/P_cam[2]
    
    # 畸变校正
    r2 = x*x + y*y
    if r2 > 100:  # 防止数值溢出
        return None
    
    # 投影
    u = K[0,0]*x + K[0,2]
    v = K[1,1]*y + K[1,2]
    
    # 边界检查
    if 0 <= u < img_width and 0 <= v < img_height:
        return (int(u), int(v))
    return None
```

### 11.1.2 时间同步策略

激光雷达和相机的采样频率不同，需要精确的时间同步。时间不同步会导致运动物体的错位，严重影响融合精度。

**传感器时序特性：**
- 激光雷达：连续旋转扫描，每个点有独立时间戳
- 相机：离散采样，整帧或逐行曝光
- 典型频率：激光雷达10-20Hz，相机30-60Hz

**硬件同步方案：**

1. **PPS（Pulse Per Second）同步：**
   - GPS提供1Hz脉冲信号，精度±50ns
   - 所有传感器锁定到同一PPS源
   - 时间戳格式：GPS时间 + 亚秒偏移
   
   ```
   t_sensor = t_GPS_week × 604800 + t_GPS_second + Δt_subsecond
   ```

2. **主从触发模式：**
   - 激光雷达作为主设备，输出同步脉冲
   - 相机接收触发信号，延迟Δt后曝光
   - 触发延迟计算：
   ```
   Δt_trigger = t_lidar_center - t_exposure/2 - t_trigger_delay
   ```

3. **IEEE 1588 PTP同步：**
   - 精确时间协议，精度可达亚微秒级
   - 主时钟广播，从设备同步
   - 延迟补偿：
   ```
   t_offset = (t_m2 - t_m1 + t_s1 - t_s2) / 2
   t_delay = (t_m2 - t_m1 + t_s2 - t_s1) / 2
   ```

**软件同步算法：**

1. **最近邻匹配：**
   ```python
   def find_nearest_timestamp(target_time, timestamps):
       idx = np.searchsorted(timestamps, target_time)
       if idx == 0:
           return 0
       if idx == len(timestamps):
           return len(timestamps) - 1
       before = timestamps[idx - 1]
       after = timestamps[idx]
       if after - target_time < target_time - before:
           return idx
       return idx - 1
   ```

2. **线性插值：**
   对于运动补偿，需要插值传感器位姿：
   ```
   α = (t_target - t_before) / (t_after - t_before)
   P_interpolated = (1 - α) × P_before + α × P_after
   R_interpolated = Slerp(R_before, R_after, α)
   ```

   其中Slerp是球面线性插值：
   ```
   Slerp(q0, q1, α) = sin((1-α)Ω)/sin(Ω) × q0 + sin(αΩ)/sin(Ω) × q1
   Ω = arccos(q0·q1)
   ```

**曝光模型详解：**

1. **全局快门（Global Shutter）：**
   - 所有像素同时曝光
   - 时间戳对应曝光中心：
   ```
   t_center = t_trigger + t_delay + t_exposure/2
   ```
   
   运动模糊模型：
   ```
   I_blur = ∫[0 to t_exp] I(t) dt / t_exposure
   ```

2. **卷帘快门（Rolling Shutter）：**
   - 逐行扫描，产生果冻效应
   - 每行时间戳：
   ```
   t_row(y) = t_start + (y/H) × t_readout
   t_readout ≈ 1/fps - t_exposure
   ```
   
   畸变校正：
   ```
   对于每行y：
   Δt = t_row(y) - t_reference
   x_corrected = x - v_x × Δt
   ```

**时间同步质量评估：**

1. **同步误差测量：**
   使用旋转标定板，计算投影误差：
   ```
   ε_sync = ||P_projected - P_detected||
   ```
   
   理论误差模型：
   ```
   ε = v × Δt_sync + ω × r × Δt_sync
   ```
   其中v是线速度，ω是角速度，r是到旋转中心距离。

2. **实例计算：**
   车辆以20m/s行驶，相机30Hz，激光雷达10Hz：
   - 最大时间差：1/10 = 100ms
   - 不同步最大误差：20 × 0.1 = 2m
   - 使用插值后：误差 < 20 × 0.001 = 2cm

**多传感器时间对齐流程：**

```python
class MultiSensorSync:
    def __init__(self, sync_tolerance=0.05):  # 50ms容差
        self.sync_tolerance = sync_tolerance
        self.sensor_queues = {}
    
    def add_measurement(self, sensor_id, timestamp, data):
        if sensor_id not in self.sensor_queues:
            self.sensor_queues[sensor_id] = deque()
        self.sensor_queues[sensor_id].append((timestamp, data))
        return self.try_sync()
    
    def try_sync(self):
        if len(self.sensor_queues) < 2:
            return None
        
        # 找到最新的共同时间戳
        latest_times = {sid: q[-1][0] for sid, q in self.sensor_queues.items() if q}
        sync_time = min(latest_times.values())
        
        # 检查是否所有传感器都有接近的数据
        synced_data = {}
        for sid, queue in self.sensor_queues.items():
            closest = self.find_closest(queue, sync_time)
            if abs(closest[0] - sync_time) > self.sync_tolerance:
                return None
            synced_data[sid] = closest[1]
        
        return sync_time, synced_data
```

### 11.1.3 特征级融合 - PointPainting

PointPainting是一种有效的特征级融合方法，将图像语义信息"绘制"到点云上，显著提升3D检测性能。

**算法原理：**

PointPainting利用成熟的2D语义分割网络增强3D点云，核心思想是将每个3D点投影到图像平面，获取对应像素的语义信息，然后将这些信息作为额外特征附加到点云上。

**详细流程：**

1. **图像语义分割：**
   使用预训练的语义分割网络（如DeepLab、Mask R-CNN）：
   ```
   输入：RGB图像 I ∈ R^(H×W×3)
   输出：语义概率图 S ∈ R^(H×W×C)
   其中C是类别数，S[i,j,k]表示像素(i,j)属于类别k的概率
   ```

2. **点云投影与语义映射：**
   ```python
   def point_painting(points, image, seg_model, K, R, t):
       # 语义分割
       semantic_probs = seg_model(image)  # H×W×C
       
       # 初始化增强特征
       num_classes = semantic_probs.shape[2]
       painted_features = np.zeros((len(points), num_classes))
       
       for i, point in enumerate(points):
           # 投影到图像
           p_cam = R @ point[:3] + t
           if p_cam[2] <= 0:
               continue
           
           u = int(K[0,0] * p_cam[0] / p_cam[2] + K[0,2])
           v = int(K[1,1] * p_cam[1] / p_cam[2] + K[1,2])
           
           if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
               painted_features[i] = semantic_probs[v, u]
       
       # 拼接原始点云和语义特征
       return np.hstack([points, painted_features])
   ```

3. **特征编码策略：**
   
   a) **One-hot编码：**
   ```
   class_id = argmax(semantic_probs)
   one_hot = [0, ..., 1, ..., 0]  # 仅class_id位置为1
   ```
   
   b) **概率分布编码：**
   ```
   features = semantic_probs  # 保留所有类别概率
   ```
   
   c) **Top-k编码：**
   ```
   top_k_classes = argsort(semantic_probs)[-k:]
   features = [class_ids, probabilities]
   ```

4. **处理投影歧义：**
   
   多个点可能投影到同一像素，需要深度排序：
   ```python
   def resolve_projection_ambiguity(points, projections):
       pixel_dict = defaultdict(list)
       
       for i, (u, v, depth) in enumerate(projections):
           pixel_dict[(u, v)].append((i, depth))
       
       # 保留最近的点
       valid_indices = []
       for pixel, point_list in pixel_dict.items():
           nearest = min(point_list, key=lambda x: x[1])
           valid_indices.append(nearest[0])
       
       return valid_indices
   ```

**性能优化：**

1. **批处理投影：**
   ```python
   # 向量化投影计算
   P_cam = (R @ points.T).T + t
   valid_mask = P_cam[:, 2] > 0
   P_cam = P_cam[valid_mask]
   
   uv = K[:2, :2] @ P_cam[:, :2].T / P_cam[:, 2] + K[:2, 2:3]
   ```

2. **GPU加速：**
   ```cuda
   __global__ void projectPoints(float* points, float* K, float* RT, 
                                 int* uv_coords, int num_points) {
       int idx = blockIdx.x * blockDim.x + threadIdx.x;
       if (idx >= num_points) return;
       
       // 3D变换
       float3 p = make_float3(points[idx*3], points[idx*3+1], points[idx*3+2]);
       float3 p_cam = matmul(RT, p);
       
       if (p_cam.z > 0) {
           uv_coords[idx*2] = K[0] * p_cam.x / p_cam.z + K[2];
           uv_coords[idx*2+1] = K[4] * p_cam.y / p_cam.z + K[5];
       }
   }
   ```

**实验验证：**

在KITTI数据集上的性能提升：
- 基线（仅点云）：Car AP@0.7 = 79.8%
- PointPainting：Car AP@0.7 = 82.1% (+2.3%)
- 对小物体提升更明显：Pedestrian AP提升+4.5%

**扩展应用：**

1. **实例级PointPainting：**
   使用实例分割替代语义分割：
   ```
   features = [semantic_class, instance_id, confidence]
   ```

2. **时序PointPainting：**
   融合多帧语义信息：
   ```
   semantic_t = α × semantic_t + (1-α) × semantic_{t-1}
   ```

3. **自监督PointPainting：**
   使用伪标签训练：
   ```
   pseudo_labels = threshold(teacher_model(image), τ)
   painted_points = point_painting(points, pseudo_labels)
   ```

### 11.1.4 深度补全算法

相机图像缺乏深度信息，而激光雷达点云稀疏（64线激光雷达仅覆盖约5%的图像像素），深度补全旨在生成稠密深度图，这对于许多下游任务如3D重建、增强现实等至关重要。

**问题定义：**
- 输入：稀疏深度图D_sparse ∈ R^(H×W)，RGB图像I ∈ R^(H×W×3)
- 输出：稠密深度图D_dense ∈ R^(H×W)
- 目标：min ||D_dense - D_gt||，同时保持边缘对齐

**稀疏深度图生成：**

```python
def generate_sparse_depth(points, K, R, t, img_height, img_width):
    depth_map = np.zeros((img_height, img_width))
    confidence_map = np.zeros((img_height, img_width))
    
    # 投影点云
    P_cam = (R @ points[:, :3].T).T + t
    valid = P_cam[:, 2] > 0
    P_cam = P_cam[valid]
    
    # 计算图像坐标
    uv = K[:2, :2] @ P_cam[:, :2].T / P_cam[:, 2] + K[:2, 2:3]
    u, v = uv[0].astype(int), uv[1].astype(int)
    
    # 处理重叠：Z-buffer
    for i in range(len(u)):
        if 0 <= u[i] < img_width and 0 <= v[i] < img_height:
            if depth_map[v[i], u[i]] == 0 or P_cam[i, 2] < depth_map[v[i], u[i]]:
                depth_map[v[i], u[i]] = P_cam[i, 2]
                confidence_map[v[i], u[i]] = P_cam[i, 3]  # 反射强度作为置信度
    
    return depth_map, confidence_map
```

**经典方法：**

1. **图像引导的各向异性扩散：**
   
   PDE形式：
   ```
   ∂D/∂t = div(g(∇I)·∇D)
   ```
   
   离散化实现：
   ```
   D^(n+1) = D^n + λ·[g_N·(D_N-D) + g_S·(D_S-D) + g_E·(D_E-D) + g_W·(D_W-D)]
   ```
   
   其中传导系数：
   ```
   g_N = exp(-||I - I_N||²/k²)
   g_S = exp(-||I - I_S||²/k²)
   g_E = exp(-||I - I_E||²/k²)
   g_W = exp(-||I - I_W||²/k²)
   ```
   
   参数选择：
   - λ = 0.25（稳定性条件）
   - k = 0.1×(I_max - I_min)（边缘阈值）
   - 迭代次数：100-500

2. **双边滤波深度补全：**
   
   完整公式：
   ```
   D_dense(p) = Σ_{q∈Ω(p)} w_s(p,q)·w_r(I_p,I_q)·w_d(D_q)·D_sparse(q) / W
   ```
   
   权重函数：
   - 空间权重：w_s(p,q) = exp(-||p-q||²/2σ_s²)
   - 颜色权重：w_r(I_p,I_q) = exp(-||I_p-I_q||²/2σ_r²)
   - 深度置信度：w_d(D_q) = exp(-|D_q|/σ_d) if D_q > 0, else 0
   - 归一化：W = Σw_s·w_r·w_d
   
   多尺度策略：
   ```python
   def multiscale_bilateral_filter(D_sparse, I, scales=[1, 2, 4, 8]):
       D_result = D_sparse.copy()
       
       for scale in scales:
           # 下采样
           D_s = downsample(D_result, scale)
           I_s = downsample(I, scale)
           
           # 双边滤波
           D_s = bilateral_filter(D_s, I_s, σ_s=3*scale, σ_r=0.1)
           
           # 上采样并融合
           D_up = upsample(D_s, scale)
           mask = D_result > 0
           D_result[~mask] = D_up[~mask]
       
       return D_result
   ```

3. **马尔可夫随机场（MRF）方法：**
   
   能量函数：
   ```
   E(D) = Σ_p φ_d(D_p) + λ·Σ_{p,q∈N} φ_s(D_p, D_q, I_p, I_q)
   ```
   
   数据项：
   ```
   φ_d(D_p) = {
       (D_p - D_sparse(p))² if D_sparse(p) > 0
       0                     otherwise
   }
   ```
   
   平滑项：
   ```
   φ_s(D_p, D_q, I_p, I_q) = min(|D_p - D_q|, τ_d) × exp(-||I_p - I_q||/τ_i)
   ```

**深度学习方法：**

1. **卷积神经网络架构：**
   ```
   输入：[D_sparse, I] ∈ R^(H×W×4)
   编码器：ResNet骨干网络提取多尺度特征
   解码器：上采样 + 跳跃连接
   输出：D_dense ∈ R^(H×W×1)
   ```

2. **损失函数设计：**
   ```
   L_total = λ_1·L_depth + λ_2·L_smooth + λ_3·L_normal
   
   L_depth = ||D_pred - D_gt||_1
   L_smooth = Σ|∇D_pred|·exp(-|∇I|)
   L_normal = 1 - cos(n_pred, n_gt)
   ```

**实际计算示例：**

场景：5×5窗口深度补全
```
稀疏深度图：        RGB强度图：
[0  0  0  0  0]    [120 125 130 135 140]
[0  10 0  0  0]    [115 120 125 130 135]
[0  0  ?  0  0]    [110 115 120 125 130]
[0  0  0  12 0]    [105 110 115 120 125]
[0  0  0  0  0]    [100 105 110 115 120]
```

计算中心点(2,2)的深度：
1. 已知深度点：(1,1)=10m, (3,3)=12m
2. 空间距离：d_1 = √2 ≈ 1.41, d_2 = √2 ≈ 1.41
3. 颜色差异：ΔI_1 = |120-120| = 0, ΔI_2 = |120-120| = 0
4. 权重（σ_s=2, σ_r=10）：
   ```
   w_s1 = exp(-2/8) = 0.779
   w_s2 = exp(-2/8) = 0.779
   w_r1 = exp(0) = 1.0
   w_r2 = exp(0) = 1.0
   ```
5. 补全结果：
   ```
   D(2,2) = (0.779×10 + 0.779×12)/(0.779+0.779) = 11m
   ```

**性能评估指标：**
- MAE：平均绝对误差 = (1/N)Σ|D_pred - D_gt|
- RMSE：均方根误差 = √[(1/N)Σ(D_pred - D_gt)²]
- δ_t：阈值精度 = % of pixels where max(D_pred/D_gt, D_gt/D_pred) < t

**优化技巧：**
1. 稀疏卷积：仅在有效深度位置计算
2. 置信度传播：从高置信度区域向外扩散
3. 边缘保持：使用法向量一致性约束

### 11.1.5 融合架构设计

多传感器融合架构的选择直接影响系统性能，需要在精度、效率和可扩展性之间权衡。

**前融合（Early Fusion）：**

在原始数据层面融合，保留最多的原始信息：

1. **数据级融合架构：**
   ```
   Raw LiDAR Points ──┐
                      ├─→ Joint Representation → Processing → Output
   Raw Camera Image ──┘
   ```

2. **实现方式：**
   
   a) **图像增强点云：**
   ```python
   # 为每个3D点添加RGB和纹理特征
   enhanced_points = []
   for point in lidar_points:
       u, v = project_to_image(point, K, R, t)
       if valid_pixel(u, v):
           rgb = image[v, u]
           gradient = compute_gradient(image, u, v)
           features = np.hstack([point, rgb, gradient])
           enhanced_points.append(features)
   ```
   
   b) **深度增强图像：**
   ```python
   # 生成RGBD图像
   depth_map = project_lidar_to_depth(lidar_points, K, R, t)
   rgbd_image = np.dstack([rgb_image, depth_map])
   ```

3. **优缺点分析：**
   - 优点：信息保留完整，理论性能上限高
   - 缺点：数据维度高，计算复杂度O(N×M)
   - 适用场景：计算资源充足，追求最高精度

**后融合（Late Fusion）：**

各传感器独立处理后融合结果：

1. **决策级融合架构：**
   ```
   LiDAR → Detector_L → Boxes_L ──┐
                                  ├─→ Fusion → Final Boxes
   Camera → Detector_C → Boxes_C ─┘
   ```

2. **融合算法：**
   
   a) **贝叶斯融合：**
   ```python
   def bayesian_fusion(detections_lidar, detections_camera):
       # 计算联合概率
       P_joint = {}
       for box_l in detections_lidar:
           for box_c in detections_camera:
               iou = compute_iou(box_l, box_c)
               if iou > threshold:
                   # P(object|L,C) = P(L|object)P(C|object)P(object) / P(L,C)
                   p_joint = box_l.conf * box_c.conf * prior[box_l.class]
                   P_joint[(box_l, box_c)] = p_joint
       
       # 非极大值抑制
       return nms(P_joint)
   ```
   
   b) **匈牙利匹配：**
   ```python
   # 构建代价矩阵
   cost_matrix = np.zeros((len(det_l), len(det_c)))
   for i, box_l in enumerate(det_l):
       for j, box_c in enumerate(det_c):
           cost_matrix[i,j] = matching_cost(box_l, box_c)
   
   # 最优匹配
   row_ind, col_ind = linear_sum_assignment(cost_matrix)
   ```

3. **置信度融合策略：**
   ```
   conf_fused = w_l × conf_l + w_c × conf_c + w_lc × conf_l × conf_c
   ```
   其中权重通过验证集学习。

**深度融合（Deep Fusion）：**

在特征层面进行融合，兼顾效率和性能：

1. **多级特征融合架构：**
   ```python
   class DeepFusionNetwork(nn.Module):
       def __init__(self):
           self.image_backbone = ResNet50()
           self.lidar_backbone = PointNet++()
           self.fusion_modules = nn.ModuleList([
               FusionBlock(256, 256),  # Early fusion
               FusionBlock(512, 512),  # Mid fusion
               FusionBlock(1024, 1024) # Late fusion
           ])
       
       def forward(self, image, points):
           # 提取多尺度特征
           img_feats = self.image_backbone(image)
           pts_feats = self.lidar_backbone(points)
           
           # 多级融合
           fused_feats = []
           for i, fusion in enumerate(self.fusion_modules):
               f = fusion(img_feats[i], pts_feats[i])
               fused_feats.append(f)
           
           return self.head(fused_feats)
   ```

2. **注意力融合机制：**
   ```python
   class AttentionFusion(nn.Module):
       def __init__(self, dim):
           super().__init__()
           self.q_img = nn.Linear(dim, dim)
           self.k_pts = nn.Linear(dim, dim)
           self.v_pts = nn.Linear(dim, dim)
       
       def forward(self, img_feat, pts_feat):
           # Cross-attention
           Q = self.q_img(img_feat)
           K = self.k_pts(pts_feat)
           V = self.v_pts(pts_feat)
           
           attn = torch.softmax(Q @ K.T / √dim, dim=-1)
           fused = attn @ V
           
           return fused + img_feat  # Residual
   ```

3. **自适应融合权重：**
   ```python
   class AdaptiveFusion(nn.Module):
       def __init__(self, channels):
           super().__init__()
           self.weight_net = nn.Sequential(
               nn.Conv2d(channels*2, channels, 1),
               nn.ReLU(),
               nn.Conv2d(channels, 2, 1),
               nn.Softmax(dim=1)
           )
       
       def forward(self, feat_img, feat_pts):
           concat = torch.cat([feat_img, feat_pts], dim=1)
           weights = self.weight_net(concat)  # [B, 2, H, W]
           
           fused = weights[:,0:1] * feat_img + weights[:,1:2] * feat_pts
           return fused
   ```

**混合融合架构：**

结合多种融合策略的优势：

```python
class HybridFusion:
    def __init__(self):
        self.early_fusion = PointPainting()
        self.deep_fusion = DeepFusionNet()
        self.late_fusion = DecisionFusion()
    
    def forward(self, image, lidar):
        # 1. 早期融合增强输入
        painted_points = self.early_fusion(lidar, image)
        
        # 2. 深度网络处理
        detections_deep = self.deep_fusion(image, painted_points)
        
        # 3. 独立检测器
        detections_img = self.img_detector(image)
        detections_pts = self.pts_detector(lidar)
        
        # 4. 后期融合
        final_detections = self.late_fusion([
            detections_deep,
            detections_img,
            detections_pts
        ])
        
        return final_detections
```

**性能对比（KITTI数据集）：**

| 融合策略 | Car AP@0.7 | Pedestrian AP@0.5 | Cyclist AP@0.5 | FPS |
|---------|-----------|------------------|----------------|-----|
| 仅激光雷达 | 79.8% | 52.1% | 67.3% | 20 |
| 仅相机 | 74.2% | 45.6% | 58.9% | 30 |
| 前融合 | 84.3% | 58.2% | 73.1% | 8 |
| 后融合 | 82.1% | 55.7% | 70.8% | 15 |
| 深度融合 | 85.6% | 60.3% | 74.9% | 12 |
| 混合融合 | 86.9% | 61.8% | 76.2% | 10 |

**工程实践建议：**

1. **实时系统（>10Hz）：** 优先后融合，计算并行化
2. **高精度需求：** 深度融合或混合融合
3. **传感器异步：** 后融合，独立处理时序
4. **故障容错：** 后融合，便于传感器切换
5. **边缘计算：** 轻量级前融合或后融合

## 11.2 激光雷达+毫米波雷达融合

毫米波雷达能够直接测量目标的径向速度，且在恶劣天气下工作稳定，与激光雷达形成良好互补。

### 11.2.1 速度信息融合

毫米波雷达通过多普勒效应测量径向速度：
```
f_d = 2v_r f_0/c
v_r = f_d·c/(2f_0)
```

其中f_0是载波频率（如77GHz），v_r是径向速度。

**卡尔曼滤波融合框架：**

状态向量：X = [x, y, z, vx, vy, vz]^T

状态转移方程：
```
X_k = F·X_{k-1} + w
F = [I₃  Δt·I₃]
    [0₃    I₃ ]
```

激光雷达观测模型（只有位置）：
```
Z_lidar = H_lidar·X + v_lidar
H_lidar = [I₃ 0₃]
```

毫米波雷达观测模型（位置+径向速度）：
```
Z_radar = h_radar(X) + v_radar
h_radar(X) = [x, y, z, (x·vx + y·vy + z·vz)/√(x²+y²+z²)]^T
```

**融合更新步骤：**
1. 预测：
   ```
   X̂_k|k-1 = F·X̂_{k-1}
   P_k|k-1 = F·P_{k-1}·F^T + Q
   ```

2. 激光雷达更新：
   ```
   K_lidar = P_k|k-1·H_lidar^T·(H_lidar·P_k|k-1·H_lidar^T + R_lidar)^{-1}
   X̂_k = X̂_k|k-1 + K_lidar·(Z_lidar - H_lidar·X̂_k|k-1)
   ```

3. 毫米波雷达更新（使用EKF）：
   ```
   H_radar = ∂h_radar/∂X|_{X̂_k}
   K_radar = P_k·H_radar^T·(H_radar·P_k·H_radar^T + R_radar)^{-1}
   X̂_k = X̂_k + K_radar·(Z_radar - h_radar(X̂_k))
   ```

**计算实例：**
目标在(10, 5, 0)m，速度(2, -1, 0)m/s：
- 激光雷达测量：(10.1, 4.9, 0.05)m
- 毫米波雷达测量：径向速度 v_r = (10×2 + 5×(-1))/√125 = 1.34m/s

径向速度验证：
```
v_r_true = (x·vx + y·vy)/r = (10×2 + 5×(-1))/11.18 = 1.34m/s ✓
```

### 11.2.2 检测置信度融合

不同传感器对不同目标的检测能力不同，需要融合置信度：

**D-S证据理论融合：**
基本概率分配（BPA）：
- m₁(A)：激光雷达认为是车辆的概率
- m₂(A)：毫米波雷达认为是车辆的概率

组合规则：
```
m(A) = Σ_{B∩C=A} m₁(B)·m₂(C) / (1-K)
K = Σ_{B∩C=∅} m₁(B)·m₂(C)
```

**示例计算：**
对于某目标：
- 激光雷达：m₁(车)=0.7, m₁(人)=0.2, m₁(不确定)=0.1
- 毫米波：m₂(车)=0.8, m₂(人)=0.1, m₂(不确定)=0.1

融合结果：
```
K = m₁(车)·m₂(人) + m₁(人)·m₂(车) = 0.7×0.1 + 0.2×0.8 = 0.23
m(车) = (0.7×0.8 + 0.7×0.1 + 0.1×0.8)/(1-0.23) = 0.71/0.77 = 0.922
m(人) = (0.2×0.1 + 0.2×0.1 + 0.1×0.1)/(1-0.23) = 0.05/0.77 = 0.065
```

### 11.2.3 恶劣天气互补

**雨雾衰减模型：**

激光雷达衰减（905nm）：
```
α_lidar_rain ≈ 0.2×R^0.6 dB/km (R: mm/h)
α_lidar_fog ≈ 13×V^{-0.6} dB/km (V: 能见度m)
```

毫米波雷达衰减（77GHz）：
```
α_radar_rain ≈ 0.003×R^1.2 dB/km
α_radar_fog ≈ 0.4 dB/km (轻雾)
```

**互补策略：**
1. 晴天：激光雷达权重0.8，毫米波0.2
2. 小雨（5mm/h）：激光雷达权重0.6，毫米波0.4
3. 大雨（50mm/h）：激光雷达权重0.3，毫米波0.7
4. 浓雾（V<50m）：激光雷达权重0.1，毫米波0.9

**自适应权重计算：**
```
w_lidar = exp(-α_lidar×R) / (exp(-α_lidar×R) + exp(-α_radar×R))
w_radar = 1 - w_lidar
```

### 11.2.4 数据关联算法

激光雷达和毫米波雷达的检测需要正确关联：

**最近邻关联：**
马氏距离：
```
d²_ij = (z_i - ẑ_j)^T·S^{-1}·(z_i - ẑ_j)
```
其中S是新息协方差矩阵。

**门限判断：**
```
d²_ij < χ²_α(n)
```
对于3维位置，95%置信度：χ²_{0.05}(3) = 7.815

**全局最优关联（匈牙利算法）：**
构建代价矩阵C，其中C_ij = d²_ij，求解：
```
min Σ_i Σ_j C_ij·x_ij
s.t. Σ_i x_ij = 1, Σ_j x_ij = 1, x_ij ∈ {0,1}
```

## 11.3 激光雷达+IMU融合

IMU（惯性测量单元）提供高频率的加速度和角速度测量，可以有效补偿激光雷达的运动畸变，提高SLAM精度。

### 11.3.1 紧耦合SLAM框架

**误差状态卡尔曼滤波（ESKF）：**

状态向量（15维）：
```
X = [δp, δv, δθ, δb_a, δb_g]^T
```
- δp：位置误差
- δv：速度误差  
- δθ：姿态误差（SO(3)李代数）
- δb_a：加速度计偏置误差
- δb_g：陀螺仪偏置误差

**连续时间运动方程：**
```
ṗ = v
v̇ = R(a - b_a - n_a) - g
Ṙ = R·[ω - b_g - n_g]×
ḃ_a = n_ba
ḃ_g = n_bg
```

**误差状态传播：**
```
δẋ = F·δx + G·n

F = [0   I   0     0      0   ]
    [0   0  -R[a]× -R     0   ]
    [0   0  -[ω]×  0     -I   ]
    [0   0   0     0      0   ]
    [0   0   0     0      0   ]
```

其中[a]×表示反对称矩阵：
```
[a]× = [ 0   -a_z  a_y]
       [ a_z  0   -a_x]
       [-a_y  a_x  0  ]
```

**离散化（中值积分）：**
```
δx_k = (I + F·Δt)·δx_{k-1} + G·n·√Δt
P_k = Φ·P_{k-1}·Φ^T + Q_d
```

### 11.3.2 IMU预积分理论

预积分避免了每次优化时重新积分IMU数据：

**位置预积分：**
```
Δp_{ij} = ∫_i^j ∫_i^t R_s(a_s - b_a)dsdt - (t_j-t_i)²g/2
```

**速度预积分：**
```
Δv_{ij} = ∫_i^j R_t(a_t - b_a)dt - (t_j-t_i)g
```

**旋转预积分：**
```
ΔR_{ij} = ∏_{k=i}^{j-1} Exp((ω_k - b_g)Δt)
```

其中Exp是SO(3)指数映射：
```
Exp(ω) = I + sin(||ω||)/||ω||·[ω]× + (1-cos(||ω||))/||ω||²·[ω]×²
```

**预积分测量的雅可比：**
对于偏置变化的一阶近似：
```
Δp̃_{ij} ≈ Δp_{ij} + J_p^{ba}·δb_a + J_p^{bg}·δb_g
Δṽ_{ij} ≈ Δv_{ij} + J_v^{ba}·δb_a + J_v^{bg}·δb_g
ΔR̃_{ij} ≈ ΔR_{ij}·Exp(J_R^{bg}·δb_g)
```

**计算实例：**
两个关键帧之间，IMU采样100Hz，持续0.5秒：
- 平均加速度：a = [0.1, 0, 9.85] m/s²（包含重力）
- 平均角速度：ω = [0, 0, 0.1] rad/s
- 偏置：b_a = [0.01, 0.01, 0], b_g = [0.001, 0.001, 0.002]

预积分结果：
```
Δv = ∫(a - b_a)dt - tg = [0.045, -0.005, 0.125] m/s
Δp = ∫∫(a - b_a)dtdt - t²g/2 = [0.011, -0.001, 0.031] m
ΔR = Exp((ω - b_g)t) ≈ Exp([0, 0, 0.049])
```

### 11.3.3 运动畸变校正

激光雷达扫描期间的运动导致点云畸变：

**线性插值方法：**
对于时刻t_i的点P_i，校正到参考时刻t_0：
```
P_0 = R_{0i}^{-1}(P_i - t_{0i})
```

其中R_{0i}和t_{0i}通过IMU积分获得：
```
R_{0i} = R_0·∏_{k=0}^{i-1} Exp(ω_k·Δt)
t_{0i} = Σ_{k=0}^{i-1} v_k·Δt + 0.5·a_k·Δt²
```

**去畸变算法流程：**
1. 对每个激光点，记录其时间戳t_i
2. 从IMU获取[t_0, t_i]期间的测量
3. 积分得到相对位姿变换
4. 将点变换到统一坐标系

**实际案例：**
车辆以20m/s行驶，激光雷达10Hz旋转：
- 一圈扫描时间：100ms
- 位移：20×0.1 = 2m
- 不校正误差：最大2m

使用IMU（200Hz）校正：
- 每5ms更新一次位姿
- 最大误差降至：20×0.005 = 0.1m

### 11.3.4 状态估计优化

**图优化框架：**
构建因子图，包含：
- IMU因子：基于预积分
- 激光雷达因子：点到面距离
- 回环因子：位姿约束

**代价函数：**
```
min Σ||r_IMU||²_{Σ_IMU} + Σ||r_LiDAR||²_{Σ_LiDAR} + Σ||r_loop||²_{Σ_loop}
```

**IMU残差：**
```
r_IMU = [R_i^T(p_j - p_i - v_i·Δt - g·Δt²/2) - Δp_{ij}]
        [R_i^T(v_j - v_i - g·Δt) - Δv_{ij}        ]
        [Log(ΔR_{ij}^T·R_i^T·R_j)                 ]
```

**LiDAR残差（点到面）：**
```
r_LiDAR = n^T·(R·p + t - q)
```
其中n是平面法向量，q是平面上的点。

**优化求解：**
使用Levenberg-Marquardt算法：
```
(J^T·W·J + λI)·Δx = -J^T·W·r
```

### 11.3.5 传感器标定

**IMU-LiDAR外参标定：**
旋转外参R_IL，平移外参t_IL

标定优化问题：
```
min Σ||p_L^j - R_IL·R_IB·(p_L^i - t_IL) - t_IL - R_IL·t_IB||²
```

**时间偏移标定：**
IMU和LiDAR可能存在时间偏移t_d：
```
t_IMU = t_LiDAR + t_d
```

通过最大化角速度相关性估计：
```
t_d = argmax Corr(ω_IMU(t), ω_LiDAR(t-τ))
```

## 11.4 激光雷达+GPS/RTK融合

GPS/RTK提供全局定位信息，可以消除SLAM的累积误差，实现大范围高精度定位。

### 11.4.1 全局定位融合框架

**图优化模型：**
节点：机器人位姿X_i = [x, y, z, roll, pitch, yaw]^T
边：
- 里程计边：相邻帧间的相对位姿
- GPS边：绝对位置约束
- 回环边：闭环检测约束

**代价函数：**
```
E = Σ||X_j ⊖ X_i ⊖ Z_{ij}^{odom}||²_{Σ_{ij}} 
  + Σ||p_i - Z_i^{GPS}||²_{Σ_{GPS}}
  + Σ||X_j ⊖ X_i ⊖ Z_{ij}^{loop}||²_{Σ_{loop}}
```

其中⊖表示SE(3)上的误差运算。

**GPS测量模型：**
```
Z_GPS = p_true + R_ENU·[σ_N·n_N, σ_E·n_E, σ_U·n_U]^T
```
- σ_N, σ_E, σ_U：北东天方向的标准差
- RTK定位：σ_N = σ_E ≈ 0.01m, σ_U ≈ 0.02m
- 普通GPS：σ_N = σ_E ≈ 2-5m, σ_U ≈ 5-10m

**ENU坐标转换：**
```
[E]   [-sin(lon)          cos(lon)           0    ] [X_ECEF]
[N] = [-sin(lat)cos(lon) -sin(lat)sin(lon)  cos(lat)] [Y_ECEF]
[U]   [cos(lat)cos(lon)   cos(lat)sin(lon)  sin(lat)] [Z_ECEF]
```

### 11.4.2 多路径效应处理

城市环境中的GPS信号受建筑物反射影响：

**多路径检测指标：**
1. C/N₀（载噪比）：< 35 dB-Hz表示信号弱
2. 仰角：< 15°的卫星易受多路径影响
3. DOP值：PDOP > 4表示几何构型差

**鲁棒估计方法：**
使用Huber损失函数：
```
ρ(e) = {
    0.5·e²,           |e| ≤ δ
    δ(|e| - 0.5δ),   |e| > δ
}
```

权重函数：
```
w(e) = {
    1,        |e| ≤ δ
    δ/|e|,    |e| > δ
}
```

**示例计算：**
GPS测量值与预测值偏差e = 5m，阈值δ = 2m：
- 二次损失：L = 0.5×5² = 12.5
- Huber损失：L = 2×(5-0.5×2) = 8
- 权重：w = 2/5 = 0.4

### 11.4.3 城市峡谷问题

高楼环境导致GPS信号遮挡和反射：

**可见性预测：**
使用激光雷达构建的3D地图预测卫星可见性：
```
Visible(sat_i) = RayCast(p_receiver, p_satellite) == clear
```

**天空视野因子（SVF）：**
```
SVF = Σ_visible cos(θ_elevation) / Σ_all cos(θ_elevation)
```

**融合策略：**
1. SVF > 0.7：正常融合GPS
2. 0.3 < SVF < 0.7：降低GPS权重
3. SVF < 0.3：仅依赖激光雷达SLAM

**自适应协方差调整：**
```
Σ_GPS_adjusted = Σ_GPS_base × (2 - SVF)²
```

### 11.4.4 松耦合vs紧耦合

**松耦合架构：**
- GPS/RTK独立解算位置
- 作为位置约束加入SLAM
- 优点：模块化，计算简单
- 缺点：未充分利用原始观测

**紧耦合架构：**
- 使用GPS原始伪距/载波相位观测
- 与激光雷达联合优化
- 优点：精度高，鲁棒性好
- 缺点：计算复杂

**伪距观测方程：**
```
ρ = ||p_sat - p_rec|| + c(dt_rec - dt_sat) + I + T + ε
```
- I：电离层延迟
- T：对流层延迟
- ε：多路径和噪声

### 11.4.5 初始化与收敛

**全局初始化：**
1. 等待GPS锁定（至少4颗卫星）
2. 静止30秒收集IMU数据估计初始姿态
3. 使用GPS位置和IMU姿态初始化

**局部到全局坐标转换：**
激光雷达SLAM通常在局部坐标系工作，需要估计到全局坐标系的变换：
```
T_global_local = argmin Σ||T·p_i^local - p_i^GPS||²
```

使用SVD求解：
1. 计算质心：p̄_local, p̄_GPS
2. 去质心：p'_i = p_i - p̄
3. 计算H = Σp'_local·p'_GPS^T
4. SVD分解：H = UΣV^T
5. 旋转：R = VU^T
6. 平移：t = p̄_GPS - R·p̄_local

**收敛监测：**
```
converged = (||Δposition|| < 0.1m) && (||Δorientation|| < 1°)
```

## 本章小结

本章详细介绍了激光雷达与相机、毫米波雷达、IMU和GPS/RTK的融合算法。主要知识点包括：

1. **激光雷达+相机融合**：
   - 投影矩阵计算：P = K[R|t]
   - 时间同步策略：硬件触发和软件插值
   - PointPainting特征融合
   - 深度补全算法

2. **激光雷达+毫米波雷达融合**：
   - 卡尔曼滤波框架融合速度信息
   - D-S证据理论融合检测置信度
   - 恶劣天气下的互补策略
   - 数据关联算法

3. **激光雷达+IMU融合**：
   - 误差状态卡尔曼滤波（ESKF）
   - IMU预积分理论：Δp, Δv, ΔR
   - 运动畸变校正
   - 紧耦合SLAM优化

4. **激光雷达+GPS/RTK融合**：
   - 图优化融合框架
   - 多路径效应和城市峡谷问题处理
   - 松耦合vs紧耦合架构
   - 坐标系转换和初始化

多传感器融合的核心是充分利用各传感器的优势，通过合理的数学框架实现信息互补，提高系统的鲁棒性和精度。在实际应用中，需要根据具体场景选择合适的融合策略。

## 练习题

### 基础题

1. **坐标转换计算**
   激光雷达检测到点P_lidar = [5, 3, 1]m，已知外参：R = Rz(30°), t = [0.2, 0.1, 0.3]m，相机内参fx=fy=800, cx=640, cy=480。计算该点在图像中的坐标。
   
   *Hint: 先进行坐标系转换，再投影到图像平面*
   
   <details>
   <summary>答案</summary>
   
   1. 旋转矩阵Rz(30°) = [[0.866, -0.5, 0], [0.5, 0.866, 0], [0, 0, 1]]
   2. P_camera = R·P_lidar + t = [3.03, 3.10, 1.3]m
   3. u = 800×3.03/1.3 + 640 = 2505, v = 800×3.10/1.3 + 480 = 2387
   </details>

2. **卡尔曼滤波预测**
   目标当前状态x=[10, 5, 2, -1]（位置和速度），Δt=0.1s，计算预测状态。
   
   *Hint: 使用匀速运动模型*
   
   <details>
   <summary>答案</summary>
   
   F = [[1, 0, 0.1, 0], [0, 1, 0, 0.1], [0, 0, 1, 0], [0, 0, 0, 1]]
   x_pred = F·x = [10.2, 4.9, 2, -1]
   </details>

3. **IMU预积分计算**
   IMU测量加速度a=[0.1, 0, 9.81]m/s²，角速度ω=[0, 0, 0.1]rad/s，时间间隔0.5s，计算速度和位置变化。
   
   *Hint: 忽略重力，假设初始速度为0*
   
   <details>
   <summary>答案</summary>
   
   Δv = a×t = [0.05, 0, 4.905]m/s
   Δp = 0.5×a×t² = [0.0125, 0, 1.226]m
   </details>

### 挑战题

4. **深度补全算法设计**
   设计一个基于双边滤波的深度补全算法，考虑颜色相似性和空间距离。给出权重计算公式和实现伪代码。
   
   *Hint: 参考双边滤波的数学形式*
   
   <details>
   <summary>答案</summary>
   
   权重：w(p,q) = exp(-||p-q||²/2σ_s²) × exp(-||I_p-I_q||²/2σ_r²)
   深度：D(p) = Σw(p,q)×D(q) / Σw(p,q)
   伪代码：遍历窗口内所有像素，计算权重，加权平均
   </details>

5. **多传感器时间同步**
   激光雷达10Hz，相机30Hz，IMU 200Hz。设计一个时间同步方案，确保数据对齐误差小于5ms。
   
   *Hint: 考虑硬件触发和软件插值*
   
   <details>
   <summary>答案</summary>
   
   1. 使用GPS的PPS信号作为公共时钟
   2. 激光雷达触发相机，确保3:1同步
   3. IMU数据通过插值对齐到激光雷达时刻
   4. 维护时间戳缓冲区，使用最近邻匹配
   </details>

6. **城市峡谷GPS融合策略**
   在城市峡谷环境中，设计一个自适应的GPS权重调整策略，考虑卫星数量、DOP值和信号强度。
   
   *Hint: 使用多个指标综合评估GPS质量*
   
   <details>
   <summary>答案</summary>
   
   质量因子Q = w1×(n_sat/12) + w2×(4/PDOP) + w3×(CNR/50)
   GPS权重 = min(Q, 1.0)
   当Q < 0.3时，完全依赖激光雷达SLAM
   </details>

7. **IMU零偏在线估计**
   设计一个算法，在车辆静止时在线估计IMU的加速度计和陀螺仪零偏。
   
   *Hint: 静止时理论加速度应为重力，角速度应为0*
   
   <details>
   <summary>答案</summary>
   
   静止检测：||a|| ∈ [9.7, 9.9] && ||ω|| < 0.01
   加速度计零偏：b_a = mean(a) - g_ref
   陀螺仪零偏：b_g = mean(ω)
   使用滑动窗口避免异常值
   </details>

8. **多传感器标定验证**
   设计一个实验方案，验证激光雷达-相机外参标定的准确性，要求精度达到像素级。
   
   *Hint: 使用标定板或自然特征点*
   
   <details>
   <summary>答案</summary>
   
   1. 使用棋盘格标定板，激光雷达提取平面，相机检测角点
   2. 投影激光雷达平面边缘到图像，计算与检测边缘的偏差
   3. 统计多个位置的重投影误差RMS
   4. 误差应小于2像素，否则重新标定
   </details>