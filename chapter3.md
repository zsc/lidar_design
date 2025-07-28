# Chapter 3: 激光雷达类型与扫描机制

激光雷达的扫描机制决定了其性能特征、成本和应用场景。本章将深入探讨从传统机械扫描到现代固态技术的各种扫描方案，通过详细的数学分析和计算实例，帮助读者理解不同技术路线的原理、优势和局限性。我们将学习如何计算关键性能参数，如角分辨率、扫描频率和点云密度分布，为激光雷达系统设计和选型提供理论基础。

## 3.1 机械扫描系统

机械扫描激光雷达是最早商业化的技术路线，通过电机驱动激光器和探测器旋转实现360°全景扫描。虽然存在体积大、可靠性等挑战，但其成熟度高、性能稳定，仍是许多应用的首选方案。

### 3.1.1 工作原理

机械扫描系统的核心是旋转平台，激光发射器和接收器安装在平台上，通过匀速旋转实现水平方向扫描。垂直方向通过多个激光器以不同角度排列，或使用摆镜实现。

**基本参数关系：**

1. **角分辨率计算**
   水平角分辨率由旋转速度和脉冲频率决定：
   
   $$\Delta\theta = \frac{360°}{f_{rotation} \times t_{pulse}}$$
   
   其中：
   - $f_{rotation}$：旋转频率（Hz）
   - $t_{pulse}$：每转脉冲数

   **计算实例1：** Velodyne VLP-16工作在10Hz，每转发射28,800个脉冲
   $$\Delta\theta = \frac{360°}{10 \times 28,800} = 0.125°$$

2. **点云密度分布**
   点云密度随距离变化：
   
   $$\rho(r,\theta,\phi) = \frac{1}{r \cdot \Delta\theta \cdot \Delta\phi}$$
   
   其中：
   - $r$：目标距离
   - $\Delta\phi$：垂直角分辨率

   **计算实例2：** 在50m处，水平0.2°、垂直2°分辨率
   $$\rho = \frac{1}{50 \times 0.00349 \times 0.0349} = 164 \text{ points/m²}$$

### 3.1.2 多线激光雷达设计

多线激光雷达通过垂直排列多个激光器实现垂直方向覆盖。关键设计参数包括：

1. **垂直视场角分布**
   非均匀分布优化近距离分辨率：
   
   $$\phi_i = \phi_{min} + \frac{i}{N-1}(\phi_{max} - \phi_{min}) \cdot f(i)$$
   
   其中$f(i)$为分布函数，常用：
   - 线性：$f(i) = 1$
   - 对数：$f(i) = \log(1 + ki)/\log(1 + k(N-1))$

   **计算实例3：** 16线激光雷达，垂直FOV -15°到+15°，对数分布（k=0.5）
   
   第8线（中间）角度：
   $$f(8) = \frac{\log(1 + 0.5 \times 8)}{\log(1 + 0.5 \times 15)} = 0.527$$
   $$\phi_8 = -15° + \frac{8}{15} \times 30° \times 0.527 = -6.58°$$

2. **数据率计算**
   总数据率由线数、旋转频率和角分辨率决定：
   
   $$R_{data} = N_{lines} \times f_{rotation} \times \frac{360°}{\Delta\theta} \times B_{point}$$
   
   **计算实例4：** 64线，10Hz，0.1°分辨率，每点48比特
   $$R_{data} = 64 \times 10 \times 3600 \times 48 = 110.6 \text{ Mbps}$$

### 3.1.3 机械误差补偿

机械系统的精度受多种因素影响：

1. **编码器误差**
   角度测量误差传播到距离误差：
   
   $$\sigma_x = r \cdot \sigma_\theta$$
   
   **计算实例5：** 100m处，编码器精度0.01°
   $$\sigma_x = 100 \times 0.01 \times \frac{\pi}{180} = 17.5 \text{ mm}$$

2. **旋转同步误差**
   时间戳误差导致的角度误差：
   
   $$\Delta\theta_{sync} = \omega \cdot \Delta t = 2\pi f_{rotation} \cdot \Delta t$$
   
   **计算实例6：** 10Hz旋转，1ms同步误差
   $$\Delta\theta_{sync} = 2\pi \times 10 \times 0.001 = 0.0628 \text{ rad} = 3.6°$$

3. **振动补偿**
   振动引起的测量误差：
   
   $$z_{vibration}(t) = A \sin(2\pi f_{vib} t + \phi)$$
   
   对测距的影响：
   $$\Delta r = z_{vibration} \cos(\alpha)$$
   
   其中$\alpha$为激光束与振动方向夹角。

### 3.1.4 扫描模式优化

1. **变速扫描**
   根据场景调整扫描速度提高效率：
   
   $$\omega(t) = \omega_0 \left(1 + k \cdot \text{ROI}(t)\right)$$
   
   ROI（感兴趣区域）函数定义重要区域。

2. **自适应采样**
   根据目标距离调整脉冲频率：
   
   $$f_{pulse}(r) = f_0 \cdot \min\left(1, \frac{r_{ref}}{r}\right)^2$$
   
   保持远距离点云密度。

### 3.1.5 功耗分析

机械扫描系统功耗主要来自：

1. **电机功耗**
   $$P_{motor} = \tau \cdot \omega + P_{friction}$$
   
   其中：
   - $\tau$：负载扭矩
   - $\omega$：角速度
   - $P_{friction}$：摩擦损耗

   **计算实例7：** 转动惯量0.1 kg·m²，10Hz，摩擦系数0.01
   $$P_{motor} = 0.1 \times (2\pi \times 10)^2 \times 0.01 = 3.95 \text{ W}$$

2. **总功耗估算**
   $$P_{total} = P_{laser} \times N + P_{motor} + P_{electronics}$$
   
   典型值：激光器200mW/线，电子系统5W
   
   **计算实例8：** 16线系统
   $$P_{total} = 0.2 \times 16 + 3.95 + 5 = 12.15 \text{ W}$$

### 3.1.6 可靠性分析

机械部件的寿命估算：

1. **轴承寿命（L10）**
   $$L_{10} = \left(\frac{C}{P}\right)^3 \times 10^6 \text{ 转}$$
   
   其中：
   - $C$：基本额定动载荷
   - $P$：等效动载荷

   **计算实例9：** C=5000N，P=500N，10Hz连续运行
   $$L_{10} = \left(\frac{5000}{500}\right)^3 \times 10^6 = 10^9 \text{ 转}$$
   $$T_{life} = \frac{10^9}{10 \times 3600 \times 24 \times 365} = 3.17 \text{ 年}$$

2. **MTBF计算**
   考虑多个失效模式：
   $$\frac{1}{MTBF_{total}} = \sum_i \frac{1}{MTBF_i}$$

## 3.2 MEMS扫描

MEMS（微机电系统）扫描技术通过微镜面的振动实现激光束偏转，具有体积小、功耗低、可靠性高的优势，是目前混合固态激光雷达的主流方案。

### 3.2.1 MEMS微镜原理

MEMS微镜通过静电、电磁或压电驱动实现镜面偏转：

1. **谐振频率计算**
   单轴MEMS镜的谐振频率：
   
   $$f_0 = \frac{1}{2\pi}\sqrt{\frac{k}{I}}$$
   
   其中：
   - $k$：扭转弹簧刚度（N·m/rad）
   - $I$：镜面转动惯量（kg·m²）

   **计算实例1：** 直径5mm圆形硅镜，厚度200μm，扭转梁宽50μm
   
   转动惯量：
   $$I = \frac{1}{2}m r^2 = \frac{1}{2} \times \rho V \times r^2$$
   $$I = \frac{1}{2} \times 2330 \times \pi \times (2.5 \times 10^{-3})^2 \times 200 \times 10^{-6} \times (2.5 \times 10^{-3})^2$$
   $$I = 1.44 \times 10^{-13} \text{ kg·m²}$$
   
   扭转刚度（矩形梁）：
   $$k = \frac{2Gwt^3}{3L}$$
   
   G = 50 GPa（硅），w = 50μm，t = 200μm，L = 500μm
   $$k = \frac{2 \times 50 \times 10^9 \times 50 \times 10^{-6} \times (200 \times 10^{-6})^3}{3 \times 500 \times 10^{-6}} = 2.67 \times 10^{-5} \text{ N·m/rad}$$
   
   谐振频率：
   $$f_0 = \frac{1}{2\pi}\sqrt{\frac{2.67 \times 10^{-5}}{1.44 \times 10^{-13}}} = 2.17 \text{ kHz}$$

2. **扫描角度与驱动电压**
   静电驱动的偏转角：
   
   $$\theta = \frac{\epsilon_0 A V^2}{2kd^2}$$
   
   其中：
   - $\epsilon_0$：真空介电常数
   - $A$：电极面积
   - $V$：驱动电压
   - $d$：电极间距

   **计算实例2：** 电极面积4mm²，间距20μm，k=2.67×10⁻⁵ N·m/rad
   
   100V驱动电压下：
   $$\theta = \frac{8.85 \times 10^{-12} \times 4 \times 10^{-6} \times 100^2}{2 \times 2.67 \times 10^{-5} \times (20 \times 10^{-6})^2} = 0.166 \text{ rad} = 9.5°$$

### 3.2.2 扫描模式

1. **谐振扫描**
   在谐振频率驱动，角度随时间变化：
   
   $$\theta(t) = \theta_{max} \sin(2\pi f_0 t)$$
   
   角速度：
   $$\omega(t) = 2\pi f_0 \theta_{max} \cos(2\pi f_0 t)$$
   
   **最大角速度：** $\omega_{max} = 2\pi f_0 \theta_{max}$

   **计算实例3：** f₀=2kHz，θ_max=20°
   $$\omega_{max} = 2\pi \times 2000 \times 0.349 = 4,390 \text{ rad/s}$$

2. **Lissajous扫描**
   双轴MEMS，不同频率驱动：
   
   $$\begin{cases}
   x(t) = A_x \sin(2\pi f_x t + \phi_x) \\
   y(t) = A_y \sin(2\pi f_y t + \phi_y)
   \end{cases}$$
   
   填充率取决于频率比$f_x/f_y$。

   **计算实例4：** 计算覆盖率
   
   对于$f_x:f_y = 43:41$的Lissajous图案，在1秒内：
   - 水平线数：43
   - 垂直线数：41
   - 交叉点数：43×41 = 1,763
   
   有效覆盖率约85-90%。

### 3.2.3 点云生成与畸变校正

1. **非线性扫描补偿**
   谐振扫描的非线性需要校正：
   
   实际角度：$\theta(t) = \theta_{max} \sin(\omega t)$
   
   期望均匀采样：$\theta_{uniform}(i) = -\theta_{max} + \frac{2\theta_{max}}{N-1} \times i$
   
   触发时刻：
   $$t_i = \frac{1}{\omega} \arcsin\left(\frac{\theta_{uniform}(i)}{\theta_{max}}\right)$$

   **计算实例5：** 100个采样点，±20°扫描，2kHz
   
   第50个点（中心）：
   $$t_{50} = \frac{1}{2\pi \times 2000} \arcsin(0) = 0$$
   
   第25个点：
   $$\theta_{25} = -20° + \frac{40°}{99} \times 24 = -10.3°$$
   $$t_{25} = \frac{1}{4000\pi} \arcsin(-0.515) = -4.14 \times 10^{-5} \text{ s}$$

2. **动态范围优化**
   不同扫描位置的驻留时间：
   
   $$\Delta t(\theta) = \frac{\Delta\theta}{|\omega(\theta)|} = \frac{\Delta\theta}{2\pi f_0 \theta_{max} \sqrt{1-(\theta/\theta_{max})^2}}$$
   
   边缘驻留时间更长，提高信噪比。

### 3.2.4 MEMS控制系统

1. **品质因子Q**
   $$Q = \frac{f_0}{\Delta f} = \frac{\sqrt{kI}}{b}$$
   
   其中$b$为阻尼系数。

   **计算实例6：** 测得3dB带宽10Hz，f₀=2kHz
   $$Q = \frac{2000}{10} = 200$$

2. **相位延迟补偿**
   驱动信号与实际偏转的相位差：
   
   $$\phi = \arctan\left(\frac{2Q(f-f_0)}{f_0}\right)$$
   
   **计算实例7：** Q=200，在1990Hz驱动
   $$\phi = \arctan\left(\frac{2 \times 200 \times (1990-2000)}{2000}\right) = \arctan(-2) = -63.4°$$

3. **启动时间**
   从静止到稳定振幅的时间：
   
   $$\theta(t) = \theta_{steady}\left(1 - e^{-t/\tau}\right)$$
   
   其中$\tau = 2Q/(2\pi f_0)$

   **计算实例8：** Q=200，f₀=2kHz
   $$\tau = \frac{2 \times 200}{2\pi \times 2000} = 31.8 \text{ ms}$$
   
   达到95%振幅：$t_{95\%} = 3\tau = 95.5 \text{ ms}$

### 3.2.5 MEMS可靠性

1. **疲劳寿命**
   硅材料的疲劳极限应力约2GPa，最大应力：
   
   $$\sigma_{max} = \frac{3EI\theta_{max}}{L^2}$$
   
   E：杨氏模量（硅：170GPa）

   **计算实例9：** 梁长500μm，厚200μm，θ_max=20°
   $$\sigma_{max} = \frac{3 \times 170 \times 10^9 \times 200 \times 10^{-6} \times 0.349}{(500 \times 10^{-6})^2} = 142 \text{ MPa}$$
   
   安全系数：$SF = 2000/142 = 14.1$

2. **温度稳定性**
   频率温度系数：
   
   $$\frac{\Delta f}{f_0} = -\frac{1}{2}\alpha_{Si} \Delta T$$
   
   硅的热膨胀系数：$\alpha_{Si} = 2.6 \times 10^{-6}$/K

   **计算实例10：** 温度变化50°C
   $$\frac{\Delta f}{f_0} = -\frac{1}{2} \times 2.6 \times 10^{-6} \times 50 = -6.5 \times 10^{-5}$$
   $$\Delta f = 2000 \times (-6.5 \times 10^{-5}) = -0.13 \text{ Hz}$$

## 3.3 OPA相控阵

光学相控阵（Optical Phased Array）技术通过控制阵列中各单元的相位实现光束偏转，无需任何机械运动部件，是真正的全固态扫描方案。

### 3.3.1 相控阵基本原理

1. **光束偏转原理**
   N个相邻单元，间距d，相位差Δφ时的偏转角：
   
   $$\sin\theta = \frac{\lambda\Delta\phi}{2\pi d}$$
   
   其中：
   - $\lambda$：激光波长
   - $\Delta\phi$：相邻单元相位差
   - $d$：单元间距

   **计算实例1：** 1550nm激光，单元间距2μm，相位差π/4
   $$\sin\theta = \frac{1550 \times 10^{-9} \times \pi/4}{2\pi \times 2 \times 10^{-6}} = 0.0969$$
   $$\theta = 5.56°$$

2. **阵列因子**
   N个单元的远场强度分布：
   
   $$AF(\theta) = \left|\frac{\sin(N\psi/2)}{\sin(\psi/2)}\right|^2$$
   
   其中$\psi = kd\sin\theta - \Delta\phi$，$k = 2\pi/\lambda$

   **主瓣宽度（FWHM）：**
   $$\Delta\theta \approx \frac{0.886\lambda}{Nd\cos\theta_0}$$

   **计算实例2：** 128个单元，间距2μm，中心偏转角0°
   $$\Delta\theta = \frac{0.886 \times 1550 \times 10^{-9}}{128 \times 2 \times 10^{-6} \times 1} = 5.36 \text{ mrad} = 0.31°$$

3. **栅瓣抑制条件**
   避免高阶衍射的条件：
   
   $$d < \frac{\lambda}{1 + |\sin\theta_{max}|}$$

   **计算实例3：** 最大扫描角±30°，λ=1550nm
   $$d < \frac{1550 \times 10^{-9}}{1 + 0.5} = 1.03 \text{ μm}$$

### 3.3.2 相位调制技术

1. **热光调制**
   硅波导的热光系数：$dn/dT = 1.86 \times 10^{-4}$/K
   
   相位变化：
   $$\Delta\phi = \frac{2\pi}{\lambda} \cdot \frac{dn}{dT} \cdot \Delta T \cdot L$$

   **计算实例4：** 波导长度100μm，温度变化10K
   $$\Delta\phi = \frac{2\pi}{1550 \times 10^{-9}} \times 1.86 \times 10^{-4} \times 10 \times 100 \times 10^{-6} = 0.755 \text{ rad}$$

   功耗：
   $$P = \frac{kA\Delta T}{t_{SiO_2}}$$
   
   其中k为热导率，A为加热器面积，t为隔离层厚度。

2. **电光调制**
   载流子等离子色散效应：
   
   $$\Delta n = -8.8 \times 10^{-22}\Delta N_e - 8.5 \times 10^{-18}(\Delta N_h)^{0.8}$$

   **计算实例5：** PIN结构，电子浓度变化10¹⁸cm⁻³
   $$\Delta n = -8.8 \times 10^{-22} \times 10^{18} = -8.8 \times 10^{-4}$$
   
   100μm波导的相位变化：
   $$\Delta\phi = \frac{2\pi}{1550 \times 10^{-9}} \times (-8.8 \times 10^{-4}) \times 100 \times 10^{-6} = -0.357 \text{ rad}$$

### 3.3.3 二维扫描设计

1. **光栅耦合器阵列**
   垂直方向通过波长调谐：
   
   $$\theta_y = \arcsin\left(\frac{\lambda - \lambda_0}{\Lambda n_{eff}}\right)$$
   
   其中$\Lambda$为光栅周期，$n_{eff}$为有效折射率。

   **计算实例6：** 光栅周期600nm，n_eff=2.5，波长调谐20nm
   $$\theta_y = \arcsin\left(\frac{20 \times 10^{-9}}{600 \times 10^{-9} \times 2.5}\right) = 0.76°$$

2. **功率分配网络**
   1×N分束器的插入损耗：
   
   $$IL = 10\log_{10}(N) + IL_{excess}$$

   **计算实例7：** 128路分束，额外损耗0.5dB
   $$IL = 10\log_{10}(128) + 0.5 = 21.6 \text{ dB}$$

### 3.3.4 系统性能分析

1. **扫描速度**
   热光调制响应时间：
   
   $$\tau_{thermal} = \frac{\rho c_p V}{G_{th}}$$
   
   其中：
   - $\rho$：密度
   - $c_p$：比热容
   - $V$：加热体积
   - $G_{th}$：热导

   **计算实例8：** 硅波导10×1×0.22μm³，热导1μW/K
   $$\tau = \frac{2330 \times 710 \times 2.2 \times 10^{-18}}{1 \times 10^{-6}} = 3.6 \text{ μs}$$

2. **光学效率**
   总效率包括：
   - 耦合效率：~50% (-3dB)
   - 分束损耗：-21.6dB (128路)
   - 调制器损耗：~-2dB
   - 天线效率：~70% (-1.5dB)
   
   **总损耗：** -28.1dB

3. **相位误差影响**
   随机相位误差对主瓣效率的影响：
   
   $$\eta = \exp(-\sigma_\phi^2)$$

   **计算实例9：** RMS相位误差0.1rad
   $$\eta = \exp(-0.1^2) = 0.99 = 99\%$$

### 3.3.5 集成光子学实现

1. **硅光子平台**
   单模波导尺寸（TE模）：
   
   $$w \approx \frac{\lambda}{2\sqrt{n_{Si}^2 - n_{SiO_2}^2}}$$

   **计算实例10：** λ=1550nm，n_Si=3.48，n_SiO₂=1.44
   $$w \approx \frac{1550 \times 10^{-9}}{2\sqrt{3.48^2 - 1.44^2}} = 244 \text{ nm}$$

2. **片上集成度**
   128×128阵列所需面积：
   
   考虑间距2μm，调制器100μm，路由50μm：
   $$A = 128 \times (2 + 150) \times 10^{-6} = 19.5 \text{ mm}$$
   
   总芯片面积约20×20mm²。

### 3.3.6 先进OPA架构

1. **稀疏阵列设计**
   减少单元数同时保持分辨率：
   
   非均匀间距：$d_i = d_0(1 + \alpha i)$
   
   抑制栅瓣的Vernier阵列：
   $$d_1 = \frac{M\lambda}{2}, \quad d_2 = \frac{N\lambda}{2}$$
   
   其中M、N互质。
   
   **计算实例11：** M=7, N=8的Vernier阵列
   $$d_1 = \frac{7 \times 1550}{2} = 5.425 \text{ μm}$$
   $$d_2 = \frac{8 \times 1550}{2} = 6.2 \text{ μm}$$
   
   最小公倍数：LCM(7,8) = 56，对应扫描范围无栅瓣。

2. **级联调制器架构**
   提高调制效率：
   
   总相移：$\phi_{total} = \sum_{i=1}^{n} \phi_i$
   
   功耗优化分配：
   $$P_i = P_{total} \cdot \frac{\phi_i}{\phi_{total}}$$
   
   **计算实例12：** 4级级联，总相移2π
   
   等分配：每级π/2，假设单位相移功耗1mW/rad
   $$P_{stage} = 1 \times \frac{\pi}{2} = 1.57 \text{ mW}$$
   $$P_{total} = 4 \times 1.57 = 6.28 \text{ mW}$$

3. **自适应波束形成**
   根据目标反馈优化相位分布：
   
   迭代优化算法：
   $$\phi_{n+1} = \phi_n + \mu \nabla_\phi J(\phi)$$
   
   其中J为目标函数（如主瓣功率）。

### 3.3.7 OPA激光雷达系统设计

1. **收发分离架构**
   发射OPA + 接收透镜阵列：
   
   接收孔径与信噪比：
   $$SNR \propto \frac{A_{rx}}{R^2}$$
   
   **计算实例13：** 10cm接收孔径，100m处1cm²目标
   $$\Omega_{target} = \frac{10^{-4}}{100^2} = 10^{-8} \text{ sr}$$
   
   收集效率：
   $$\eta_{collect} = \frac{A_{rx}}{4\pi R^2} = \frac{\pi \times 0.05^2}{4\pi \times 100^2} = 6.25 \times 10^{-6}$$

2. **相干探测集成**
   片上集成本振光路：
   
   相干混频器设计：
   - 90°混频器：提取I/Q分量
   - 平衡探测器：抑制共模噪声
   
   **计算实例14：** 本振功率10mW，信号功率1nW
   
   拍频信号功率：
   $$P_{IF} = 2\sqrt{P_{LO} \cdot P_{sig}} = 2\sqrt{10^{-2} \times 10^{-9}} = 2 \times 10^{-5.5} = 63.2 \text{ nW}$$

3. **多波长复用**
   增加并行通道：
   
   波长间隔（避免串扰）：
   $$\Delta\lambda > \frac{\lambda^2}{n_g L}$$
   
   其中$n_g$为群折射率，L为器件长度。
   
   **计算实例15：** 硅波导n_g=4.2，器件长度1cm
   $$\Delta\lambda > \frac{(1550 \times 10^{-9})^2}{4.2 \times 0.01} = 57.2 \text{ nm}$$

### 3.3.8 OPA技术挑战与解决方案

1. **功耗优化**
   
   系统总功耗：
   $$P_{total} = N_{ch} \times (P_{mod} + P_{driver}) + P_{laser} + P_{control}$$
   
   **计算实例16：** 128通道系统
   - 调制器：5mW/通道
   - 驱动器：10mW/通道  
   - 激光器：100mW
   - 控制：200mW
   
   $$P_{total} = 128 \times 15 + 100 + 200 = 2220 \text{ mW}$$

2. **相位校准**
   
   初始相位误差来源：
   - 制造公差：~λ/20
   - 温度漂移：~0.1rad/K
   - 老化：~0.01rad/天
   
   校准算法：
   $$\phi_{cal,i} = \arg\max_{\phi_i} \left|\sum_j A_j e^{j(\phi_j + \delta_{ij})}\right|$$

3. **环境适应性**
   
   温度补偿：
   $$\phi_{comp}(T) = \phi_0 + \alpha(T-T_0)$$
   
   **计算实例17：** 温度系数0.1rad/K，工作范围-40°C到85°C
   $$\Delta\phi_{max} = 0.1 \times 125 = 12.5 \text{ rad}$$
   
   需要至少2π调制范围的2倍。

## 3.4 Flash激光雷达

Flash激光雷达采用面阵探测器同时捕获整个视场的深度信息，类似于TOF相机但具有更高的精度和更远的探测距离。这种无扫描架构具有高帧率、高可靠性的优势。

### 3.4.1 工作原理

1. **基本架构**
   Flash激光雷达核心组件：
   - 脉冲激光器：照明整个视场
   - 扩束光学系统：均匀照明
   - 接收透镜：成像光学系统
   - 面阵探测器：APD或SPAD阵列
   - 时间测量电路：每像素TDC

2. **照明均匀性**
   高斯光束扩展后的强度分布：
   
   $$I(r) = I_0 \exp\left(-\frac{2r^2}{w^2}\right)$$
   
   边缘与中心强度比：
   $$\frac{I_{edge}}{I_{center}} = \exp\left(-\frac{2r_{FOV}^2}{w^2}\right)$$
   
   **计算实例1：** 40°×30° FOV，要求边缘强度>80%中心
   
   对角半角：$r_{FOV} = \sqrt{20^2 + 15^2} = 25°$
   
   所需束腰：
   $$w = \sqrt{\frac{2r_{FOV}^2}{-\ln(0.8)}} = \sqrt{\frac{2 \times 25^2}{0.223}} = 75°$$

3. **像素级测距**
   每个像素独立测量TOF：
   
   距离分辨率：
   $$\Delta R = \frac{c \cdot \Delta t}{2}$$
   
   **计算实例2：** TDC分辨率50ps
   $$\Delta R = \frac{3 \times 10^8 \times 50 \times 10^{-12}}{2} = 7.5 \text{ mm}$$

### 3.4.2 面阵探测器设计

1. **SPAD阵列**
   
   像素结构参数：
   - 有效面积：10-50μm²
   - 填充因子：10-80%
   - 死时间：10-100ns
   
   **光子探测效率（PDE）：**
   $$PDE = QE \times FF \times P_{trigger}$$
   
   **计算实例3：** QE=50%，填充因子30%，触发概率90%
   $$PDE = 0.5 \times 0.3 \times 0.9 = 13.5\%$$

2. **APD阵列**
   
   线性模式APD的信噪比：
   $$SNR = \frac{M \cdot P_{sig}}{\sqrt{2q(I_{sig}M^2F + I_{dark})B + 4kTB/R}}$$
   
   其中：
   - M：雪崩增益
   - F：过剩噪声因子
   - B：带宽
   
   **计算实例4：** M=100，F=5，信号电流1nA，暗电流10pA，B=100MHz
   
   信号功率：$P_{sig} = 100 \times 1 \times 10^{-9} = 10^{-7}$ W
   
   噪声项：
   - 散粒噪声：$2q \times 1 \times 10^{-9} \times 100^2 \times 5 \times 10^8 = 1.6 \times 10^{-16}$ A²
   - 暗电流噪声：$2q \times 10 \times 10^{-12} \times 10^8 = 3.2 \times 10^{-20}$ A²
   - 热噪声（50Ω）：$4 \times 1.38 \times 10^{-23} \times 300 \times 10^8 / 50 = 3.3 \times 10^{-18}$ A²
   
   $$SNR = \frac{10^{-7}}{\sqrt{1.6 \times 10^{-16} + 3.3 \times 10^{-18}}} = 250$$

3. **阵列规模与数据率**
   
   总数据率：
   $$R_{data} = N_x \times N_y \times f_{frame} \times B_{pixel}$$
   
   **计算实例5：** 320×240阵列，30fps，12bit深度+4bit强度
   $$R_{data} = 320 \times 240 \times 30 \times 16 = 36.9 \text{ Mbps}$$

### 3.4.3 光学系统设计

1. **发射光学**
   
   扩束系统的发散角：
   $$\theta_{div} = \frac{D_{laser}}{f_{collimator}} \times M_{expander}$$
   
   **计算实例6：** 激光光斑3mm，准直焦距10mm，扩束比10×
   $$\theta_{div} = \frac{3}{10} \times 10 = 3 \text{ rad} = 172°$$
   
   实际设计需要光束整形器限制到所需FOV。

2. **接收光学**
   
   景深与F数关系：
   $$DOF = \frac{2Nc\delta}{f^2}$$
   
   其中：
   - N：F数
   - c：圈困直径（像素尺寸）
   - δ：对焦距离
   - f：焦距
   
   **计算实例7：** f=25mm，F/2.8，像素25μm，对焦10m
   $$DOF = \frac{2 \times 2.8 \times 25 \times 10^{-6} \times 10}{(25 \times 10^{-3})^2} = 2.24 \text{ m}$$

3. **视场与分辨率**
   
   角分辨率：
   $$\alpha = \arctan\left(\frac{p}{f}\right)$$
   
   其中p为像素间距。
   
   **计算实例8：** 25μm像素，25mm焦距
   $$\alpha = \arctan\left(\frac{25 \times 10^{-6}}{25 \times 10^{-3}}\right) = 0.001 \text{ rad} = 0.057°$$
   
   总视场：
   $$FOV = 2\arctan\left(\frac{D_{sensor}}{2f}\right)$$
   
   320×240阵列，像素25μm：
   $$FOV_H = 2\arctan\left(\frac{320 \times 25 \times 10^{-6}}{2 \times 25 \times 10^{-3}}\right) = 18.2°$$

### 3.4.4 时间测量架构

1. **全局快门vs滚动快门**
   
   全局快门的优势：
   - 无运动畸变
   - 简化时间戳
   
   功耗对比：
   $$P_{global} = N_{pixel} \times P_{TDC}$$
   $$P_{rolling} = N_{row} \times P_{TDC}$$
   
   **计算实例9：** 320×240阵列，TDC功耗1mW
   - 全局：$320 \times 240 \times 1 = 76.8$ W（不实际）
   - 滚动：$240 \times 1 = 240$ mW

2. **共享TDC架构**
   
   时间复用比：
   $$N_{share} = \frac{T_{frame}}{T_{measure} \times N_{pixel}}$$
   
   **计算实例10：** 33ms帧时间，单次测量100μs
   $$N_{share} = \frac{33 \times 10^{-3}}{100 \times 10^{-6} \times 76800} = 0.0043$$
   
   需要至少232个像素共享一个TDC。

3. **直方图法测距**
   
   多次采样构建直方图：
   $$H(t) = \sum_{i=1}^{N} \delta(t - t_i)$$
   
   峰值检测的信噪比改善：
   $$SNR_{hist} = SNR_{single} \times \sqrt{N}$$
   
   **计算实例11：** 单次SNR=3，累积100次
   $$SNR_{hist} = 3 \times \sqrt{100} = 30$$

### 3.4.5 性能优化

1. **环境光抑制**
   
   带通滤波器设计：
   - 中心波长：λ₀ ± 1nm
   - 带宽：10-40nm
   
   背景光子率：
   $$R_{bg} = \frac{E_{sun} \cdot A_{pixel} \cdot \Omega \cdot \Delta\lambda \cdot QE}{h\nu}$$
   
   **计算实例12：** 日光1000W/m²/μm，像素面积625μm²，立体角0.01sr，滤光片40nm
   
   $$R_{bg} = \frac{1000 \times 625 \times 10^{-12} \times 0.01 \times 40 \times 10^{-9} \times 0.5}{(6.626 \times 10^{-34} \times 3 \times 10^8)/(905 \times 10^{-9})}$$
   $$R_{bg} = 6.25 \times 10^8 \text{ photons/s}$$

2. **动态范围扩展**
   
   多次曝光HDR：
   $$I_{HDR} = \sum_{i} w_i(I) \cdot I_i \cdot g_i$$
   
   其中$w_i$为权重函数，$g_i$为增益。
   
   **计算实例13：** 3次曝光，增益比1:10:100
   
   动态范围扩展：$DR = 20\log_{10}(100) = 40$ dB

3. **智能像素架构**
   
   片上处理功能：
   - 首光子检测
   - 背景估计
   - 边缘检测
   
   功耗-性能权衡：
   $$FOM = \frac{Range \times Resolution}{Power \times Area}$$

### 3.4.6 Flash激光雷达应用

1. **近距离3D感知**
   
   典型应用场景：
   - 手势识别：0.3-2m
   - 人脸识别：0.5-1.5m  
   - 机器人避障：0.1-10m
   
   分辨率需求计算：
   $$\Delta x = R \cdot \tan(\Delta\theta)$$
   
   **计算实例14：** 1m处需要5mm分辨率
   $$\Delta\theta = \arctan\left(\frac{5 \times 10^{-3}}{1}\right) = 0.005 \text{ rad} = 0.29°$$
   
   所需像素数（30°FOV）：$N = 30/0.29 = 104$ 像素

2. **汽车短程传感**
   
   应用：
   - 自动泊车
   - 盲点检测
   - 开门防撞
   
   探测概率分析：
   $$P_d = \left(1 - e^{-\lambda_{sig}T}\right) \cdot e^{-\lambda_{bg}T}$$
   
   其中$\lambda_{sig}$为信号光子率，$\lambda_{bg}$为背景光子率，T为积分时间。

3. **无人机防撞**
   
   轻量化设计要求：
   - 重量：<100g
   - 功耗：<3W
   - 帧率：>10fps
   
   最小探测距离：
   $$R_{min} = \frac{v^2}{2a} + v \cdot t_{react}$$
   
   **计算实例15：** 飞行速度20m/s，减速度5m/s²，反应时间0.1s
   $$R_{min} = \frac{20^2}{2 \times 5} + 20 \times 0.1 = 40 + 2 = 42 \text{ m}$$

## 3.5 合成孔径激光雷达（SAL）

合成孔径激光雷达借鉴了微波SAR的原理，通过平台运动合成大孔径，实现超越物理孔径限制的高分辨率成像。SAL在远距离高分辨率成像方面具有独特优势。

### 3.5.1 SAL基本原理

1. **合成孔径概念**
   
   方位向分辨率：
   $$\delta_a = \frac{\lambda R}{2L}$$
   
   其中：
   - λ：激光波长
   - R：目标距离
   - L：合成孔径长度
   
   **计算实例1：** 1550nm激光，1km距离，飞行100m
   $$\delta_a = \frac{1550 \times 10^{-9} \times 1000}{2 \times 100} = 7.75 \text{ mm}$$
   
   对比实孔径（10cm直径）：
   $$\delta_{real} = 1.22\frac{\lambda R}{D} = 1.22 \times \frac{1550 \times 10^{-9} \times 1000}{0.1} = 18.9 \text{ mm}$$

2. **相干积累条件**
   
   相位稳定性要求：
   $$\Delta\phi < \frac{\pi}{4}$$
   
   对应位置精度：
   $$\Delta x < \frac{\lambda}{8} = \frac{1550 \times 10^{-9}}{8} = 194 \text{ nm}$$
   
   振动容限分析：
   $$a_{max} = \frac{4\pi^2 \Delta x}{T^2}$$
   
   **计算实例2：** 积分时间1s
   $$a_{max} = \frac{4\pi^2 \times 194 \times 10^{-9}}{1^2} = 7.65 \times 10^{-6} \text{ m/s²}$$

3. **多普勒历程**
   
   瞬时多普勒频率：
   $$f_d(t) = \frac{2v}{\lambda}\sin\theta(t)$$
   
   对于匀速直线运动：
   $$f_d(t) = \frac{2v}{\lambda} \cdot \frac{vt}{\sqrt{R^2 + (vt)^2}}$$
   
   **计算实例3：** v=100m/s，R=1km，t=0.5s
   $$f_d = \frac{2 \times 100}{1550 \times 10^{-9}} \times \frac{100 \times 0.5}{\sqrt{1000^2 + 50^2}} = 6.45 \text{ MHz}$$

### 3.5.2 SAL信号处理

1. **距离-多普勒算法**
   
   二维匹配滤波：
   $$s_{out}(r,a) = \iint s_{in}(t,\tau) h^*(t-\tau, a-vt) dt d\tau$$
   
   距离向压缩：
   $$s_r(t) = s(t) \otimes h_r^*(t)$$
   
   其中$h_r(t) = \exp(j\pi K_r t^2)$为线性调频参考信号。

2. **运动补偿**
   
   相位误差：
   $$\phi_{error} = \frac{4\pi}{\lambda}\Delta R$$
   
   **计算实例4：** 平台高度变化1cm
   $$\phi_{error} = \frac{4\pi}{1550 \times 10^{-9}} \times 0.01 = 81.3 \text{ rad}$$
   
   需要多次2π相位解缠。

3. **自聚焦算法**
   
   相位梯度自聚焦（PGA）：
   $$\hat{\phi} = \arg\left\{\sum_n s_n(r) \cdot s_n^*(r-\Delta r)\right\}$$
   
   收敛判据：
   $$\sigma_\phi < 0.1 \text{ rad}$$

### 3.5.3 逆合成孔径（ISAL）

1. **转台成像**
   
   目标旋转产生的多普勒：
   $$f_d = \frac{2\omega r \sin\alpha}{\lambda}$$
   
   其中：
   - ω：旋转角速度
   - r：散射点到旋转中心距离
   - α：视线角
   
   **计算实例5：** 目标10m，旋转1°/s，散射点距中心5m
   $$\omega = \frac{\pi}{180} = 0.0175 \text{ rad/s}$$
   $$f_d = \frac{2 \times 0.0175 \times 5}{1550 \times 10^{-9}} = 1.13 \times 10^8 \text{ Hz}$$

2. **横向分辨率**
   
   $$\delta_{cross} = \frac{\lambda}{2\Delta\theta}$$
   
   **计算实例6：** 观测角度变化20°
   $$\delta_{cross} = \frac{1550 \times 10^{-9}}{2 \times 20 \times \pi/180} = 2.22 \times 10^{-6} \text{ m} = 2.22 \text{ μm}$$
   
   在5m处的实际分辨率：
   $$\delta_{actual} = 5 \times 2.22 \times 10^{-6} = 11.1 \text{ μm}$$

3. **运动目标成像**
   
   补偿策略：
   - 平动补偿：质心跟踪
   - 转动估计：特征点匹配
   
   最小可检测转速：
   $$\omega_{min} = \frac{\lambda}{4TR_{max}}$$
   
   **计算实例7：** 积分时间0.1s，最大尺寸10m
   $$\omega_{min} = \frac{1550 \times 10^{-9}}{4 \times 0.1 \times 10} = 3.88 \times 10^{-10} \text{ rad/s}$$

### 3.5.4 激光器要求

1. **相干长度**
   
   最小相干长度：
   $$L_c > 2R_{max} + L_{synthetic}$$
   
   对应线宽：
   $$\Delta\nu < \frac{c}{2\pi L_c}$$
   
   **计算实例8：** R_max=10km，L_synthetic=1km
   $$L_c > 2 \times 10^4 + 10^3 = 21 \text{ km}$$
   $$\Delta\nu < \frac{3 \times 10^8}{2\pi \times 2.1 \times 10^4} = 2.27 \text{ MHz}$$

2. **频率稳定性**
   
   Allan方差要求：
   $$\sigma_y(\tau) < \frac{\delta_a}{2R\tau}$$
   
   **计算实例9：** 分辨率10mm，距离1km，积分时间1s
   $$\sigma_y(1s) < \frac{0.01}{2 \times 1000 \times 1} = 5 \times 10^{-6}$$

3. **功率需求**
   
   考虑大气传输和散斑平均：
   $$P_{req} = \frac{NEP \cdot \sqrt{B} \cdot R^2 \cdot (4\pi)}{A_{rx} \cdot \rho \cdot T_{atm}^2 \cdot \eta_{sys}}$$
   
   **计算实例10：** NEP=10⁻¹⁴W/√Hz，B=10MHz，R=5km，A_rx=0.01m²
   $$P_{req} = \frac{10^{-14} \times \sqrt{10^7} \times (5000)^2 \times 4\pi}{0.01 \times 0.1 \times 0.8^2 \times 0.5}$$
   $$P_{req} = 246 \text{ mW}$$

### 3.5.5 SAL应用

1. **空间态势感知**
   
   卫星细节成像：
   - 距离：100-1000km
   - 分辨率需求：<10cm
   - 合成孔径：>10km
   
   轨道相对运动利用：
   $$v_{rel} = \sqrt{\mu \left(\frac{2}{r} - \frac{1}{a}\right)}$$

2. **振动测量**
   
   微多普勒特征：
   $$f_{vib}(t) = \frac{2A\omega_{vib}}{\lambda}\cos(\omega_{vib}t)$$
   
   **计算实例11：** 振幅1mm，频率100Hz
   $$f_{vib,max} = \frac{2 \times 10^{-3} \times 2\pi \times 100}{1550 \times 10^{-9}} = 8.1 \times 10^8 \text{ Hz}$$

3. **穿透成像**
   
   叶簇穿透SAL（FOPEN）：
   - 多次散射建模
   - 极化分析
   - 相干变化检测
   
   穿透深度估算：
   $$d_{pen} = \frac{1}{2\alpha}$$
   
   其中α为衰减系数（~0.1-1 dB/m）。

### 3.5.6 SAL系统集成

1. **平台要求**
   
   位置测量精度（GPS/INS）：
   $$\sigma_{pos} < \frac{\lambda}{8\sqrt{2}} = \frac{1550 \times 10^{-9}}{8\sqrt{2}} = 137 \text{ nm}$$
   
   实际使用差分GPS和IMU融合。

2. **数据处理**
   
   计算复杂度：
   $$O(N_r N_a \log N_a)$$
   
   **计算实例12：** 10k×10k像素
   $$\text{运算次数} \approx 10^4 \times 10^4 \times \log_2(10^4) = 1.33 \times 10^9$$
   
   GPU加速必需。

3. **实时处理架构**
   
   流水线设计：
   - 数据获取：连续
   - 距离压缩：逐脉冲
   - 方位压缩：分块处理
   - 自聚焦：迭代优化

## 本章小结

本章系统介绍了激光雷达的五种主要扫描机制，每种技术都有其独特的优势和适用场景：

1. **机械扫描系统**：技术成熟，360°全景扫描，但体积大、可靠性受限
   - 关键公式：角分辨率 $\Delta\theta = 360°/(f_{rotation} \times t_{pulse})$
   - 点云密度：$\rho(r,\theta,\phi) = 1/(r \cdot \Delta\theta \cdot \Delta\phi)$

2. **MEMS扫描**：体积小、功耗低、可靠性高的混合固态方案
   - 谐振频率：$f_0 = (1/2\pi)\sqrt{k/I}$
   - 扫描角度：$\theta(t) = \theta_{max}\sin(2\pi f_0 t)$

3. **OPA相控阵**：真正全固态，无机械部件，但功耗和效率仍需优化
   - 光束偏转：$\sin\theta = \lambda\Delta\phi/(2\pi d)$
   - 栅瓣抑制：$d < \lambda/(1 + |\sin\theta_{max}|)$

4. **Flash激光雷达**：高帧率面阵成像，适合近距离3D感知
   - 景深公式：$DOF = 2Nc\delta/f^2$
   - 角分辨率：$\alpha = \arctan(p/f)$

5. **合成孔径激光雷达**：超高分辨率远距离成像
   - 方位分辨率：$\delta_a = \lambda R/(2L)$
   - 相干要求：$\Delta\phi < \pi/4$

选择合适的扫描机制需要综合考虑应用需求、成本预算、技术成熟度等因素。未来的发展趋势是向全固态、低成本、高集成度方向演进。

## 练习题

### 基础题

1. **机械扫描计算**
   一个16线激光雷达以20Hz旋转，每转发射57,600个脉冲。计算：
   a) 水平角分辨率
   b) 在100m处的点间距
   c) 每秒产生的点云数据量（每点64字节）
   
   *Hint: 使用角分辨率公式，注意单位换算*

2. **MEMS谐振频率**
   设计一个MEMS微镜，镜面直径3mm，厚度150μm，硅材料密度2330kg/m³。如果要求谐振频率为1.5kHz，计算所需的扭转弹簧刚度。
   
   *Hint: 先计算转动惯量，再用谐振频率公式*

3. **OPA栅瓣条件**
   对于905nm激光的OPA系统，如果要求扫描范围±45°，计算最大允许的阵元间距。如果实际间距为1.2μm，会在什么角度出现第一个栅瓣？
   
   *Hint: 使用栅瓣抑制条件公式*

4. **Flash激光雷达分辨率**
   一个Flash激光雷达使用640×480的SPAD阵列，像素尺寸20μm，配备焦距35mm的镜头。计算水平和垂直视场角，以及在50m处的空间分辨率。
   
   *Hint: 使用FOV和角分辨率公式*

### 挑战题

5. **多线激光雷达优化设计**
   设计一个32线车载激光雷达的垂直角度分布。要求：
   - 垂直FOV：-25°到+15°
   - 近距离（<30m）分辨率优于远距离
   - 使用对数分布函数
   
   计算并绘制每条激光线的角度，分析在10m、50m、100m处的垂直分辨率分布。
   
   *Hint: 考虑车辆应用中地面和障碍物检测的需求*

6. **MEMS Lissajous扫描优化**
   设计一个双轴MEMS扫描系统，要求在100ms内覆盖30°×20°的视场，覆盖率>90%。选择合适的X/Y轴频率比，计算所需的镜面偏转角度和驱动参数。
   
   *Hint: 考虑互质频率比，分析不同比值的覆盖效果*

7. **SAL系统设计**（开放性思考题）
   为月球轨道器设计一个合成孔径激光雷达系统，用于绘制月球表面10cm分辨率的地形图。考虑：
   - 轨道高度：100km
   - 飞行速度：1.6km/s
   - 可用功率：<500W
   
   分析系统参数选择、技术挑战和可能的解决方案。
   
   *Hint: 考虑真空环境、功率限制、数据传输等因素*

8. **混合扫描架构**（开放性思考题）
   提出一种结合MEMS和OPA优势的混合扫描架构。分析：
   - 如何分配两种技术的扫描任务
   - 系统级的优势和挑战
   - 可能的应用场景
   - 与纯MEMS或纯OPA方案的性能/成本对比
   
   *Hint: 考虑MEMS的大角度扫描能力和OPA的快速小角度调节*

<details>
<summary>练习题答案</summary>

1. **机械扫描计算**
   a) $\Delta\theta = 360°/(20 \times 57,600) = 0.3125°$
   b) 弧长 = $100 \times 0.3125 \times \pi/180 = 0.545$m
   c) 点云率 = $16 \times 20 \times 57,600 = 18.432$M点/秒
      数据率 = $18.432 \times 10^6 \times 64 = 1.18$Gbps

2. **MEMS谐振频率**
   转动惯量：$I = \frac{1}{2}mr^2 = \frac{1}{2} \times \rho \pi r^2 t \times r^2$
   $I = \frac{1}{2} \times 2330 \times \pi \times (1.5×10^{-3})^2 \times 150×10^{-6} \times (1.5×10^{-3})^2$
   $I = 2.78 \times 10^{-14}$ kg·m²
   
   所需刚度：$k = (2\pi f_0)^2 \times I = (2\pi \times 1500)^2 \times 2.78 \times 10^{-14} = 2.47 \times 10^{-6}$ N·m/rad

3. **OPA栅瓣条件**
   最大间距：$d_{max} = 905×10^{-9}/(1 + \sin45°) = 905×10^{-9}/1.707 = 530$nm
   
   实际间距1.2μm时，栅瓣出现在：
   $\sin\theta_g = \sin\theta_0 + \lambda/d = \sin\theta_0 + 905/1200 = \sin\theta_0 + 0.754$
   当$\theta_0 = 15.4°$时，$\sin\theta_g = 1$，即$\theta_g = 90°$

4. **Flash激光雷达分辨率**
   水平FOV：$2\arctan(640 \times 20×10^{-6}/(2 \times 35×10^{-3})) = 20.9°$
   垂直FOV：$2\arctan(480 \times 20×10^{-6}/(2 \times 35×10^{-3})) = 15.7°$
   
   角分辨率：$\arctan(20×10^{-6}/35×10^{-3}) = 0.0327°$
   50m处分辨率：$50 \times \tan(0.0327°) = 28.6$mm

5. **多线激光雷达优化设计**
   使用对数分布，k=0.3：
   第i线角度：$\phi_i = -25° + 40° \times \log(1+0.3i)/\log(1+0.3×31) \times i/31$
   
   10m处垂直分辨率最密集区域（i=20附近）：~15cm
   100m处相同区域：~1.5m

6. **MEMS Lissajous扫描优化**
   选择频率比fx:fy = 29:31（互质）
   100ms内：X轴2.9周期，Y轴3.1周期
   所需偏转角：±15°（X轴），±10°（Y轴）
   覆盖率：~92%

7. **SAL系统设计**
   合成孔径长度：$L = \lambda R/(2\delta_a) = 1550×10^{-9} × 100×10^3/(2 × 0.1) = 775$m
   积分时间：775/1600 = 0.48s
   
   主要挑战：
   - 0.48s内保持相位稳定性
   - 数据率：~10Gbps
   - 激光功率需求：~50W（考虑月面反射率）

8. **混合扫描架构**
   方案：MEMS粗扫 + OPA精扫
   - MEMS：60°×40°大视场，0.2°分辨率
   - OPA：±2°快速扫描，0.01°分辨率
   - 应用：MEMS发现目标，OPA精确跟踪
   
   优势：兼顾大视场和高分辨率
   挑战：两系统同步和标定
</details>