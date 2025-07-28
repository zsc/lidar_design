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