# ZYNQ-7000 EVM 欧拉视频放大与智能热力图监测系统


## 📖 项目简介

本项目是将麻省理工学院（MIT CSAIL）在计算机图形学顶尖会议 **SIGGRAPH 2012** 提出的经典 **欧拉视频放大（Eulerian Video Magnification, EVM）** 算法，完整工程化移植并创新重构至 **Xilinx ZYNQ-7000 SoC (XC7Z020)** 异构计算平台与 **PC 端 PyQt5 智能上位机** 的全栈软硬件协同系统。

针对微弱物理振动不可见、CMOS 传感器高频散粒噪点干扰等工程痛点，本项目创新构建了 **“空间 2x 降采样 (+6dB SNR) + 多尺度金字塔 + 时域双 IIR + 硬件时域死区软阈值降噪 (Temporal Noise Coring) + 伪彩能量场时间平滑 + 半透明自适应 Alpha 叠加 + 全双工 GUI 滑块/按键双向调参 + 极简显示屏”** 完整技术链路，实现了对微米级机械共振、结构微形变、生物微弱脉搏的毫秒级实时捕捉、放大与直观可视化。

---

## 🏛️ 系统总体架构与工程目录

根据系统软硬件协同方案架构，整套工程划分为 **4 大层次化模块体系**：

```
EVM_CAM/
├── 02_ZYNQ_PL_Hardware/            # 【第一层：ZYNQ-7020 硬件 PL 端 (Verilog HDL)】
│   ├── rtl/                        # 纯 Verilog HDL (IEEE 1364-2001) 硬件流水线
│   │   ├── camera/                 # ov5640_dvp_capture.v (DVP 字节抓取与 RGB888 解包)
│   │   ├── preprocess/             # rgb2yuv.v (提取亮度 Y), downscale_2x.v (2x 降采样提升 6dB SNR)
│   │   ├── evm/                    # line_buffer_5x5.v, laplacian_pyr.v (无乘法器空间差分),
│   │   │                           # iir_bandpass_coring.v (双IIR+硬件死区门限), amplify_add.v (增益饱和)
│   │   ├── heatmap/                # colormap_lut.v (0 BRAM 双色表 ROM), heatmap_overlay.v (Alpha 叠加与峰值遥测)
│   │   ├── display/                # display_mux.v (3 模式无缝选通，严格 4 拍对齐防撕裂)
│   │   ├── ps_pl_interface/        # axi_lite_regs.v (AXI4-Lite 从机总线，基地址 0x43C00000)
│   │   └── top/                    # evm_pl_top.v (顶层整合), tb_evm_pl_top.v (自动化 Testbench)
│   ├── constraints/                # pin_constraints.xdc (正点原子领航者物理引脚与时钟约束)
│   ├── scripts/                    # check_rtl_consistency.py (自动化 RTL 端口连线 100% 校验脚本)
│   └── hls_reference/              # evm_motion_hls.cpp, evm_motion_hls.h (Vivado HLS C++ 参考)
│
├── 03_ZYNQ_PS_Software/            # 【第二层：ZYNQ ARM Cortex-A9 软件 PS 端 】
│   ├── app/                        # main_ps_control.c (主控、GUI滑块协议与串口双模驱动、AXI 寄存器配置与遥测上报)
│   ├── drivers/                    # ov5640_sccb.c, ov5640_sccb.h (激活硬件 2D 去噪 ISP 与锁定原生增益/曝光)
│   └── c_fixed_point_model/        # Q4.12 定点化数学算法模型与测试套件
│       ├── evm_motion_model.c/h    # 定点化金字塔与时域差分平滑 C 实现
│       ├── test_evm_c_runner.c     # 30 帧合成微振动序列测试激励 (通过 GCC 自动化验证通过)
│       └── sim_fixed_point_evm.py  # Python 定点仿真与波形验证
│
├── 04_PC_Dashboard_PyQt5/          # 【第三层：PC 端 PyQt5 智能上位机 (Python)】
│   └── evm_monitor_pc.py           # 现代化仪表盘 (大屏全屏视频监视、4 项大字核心指标卡片、滑块调参与 FFT 频谱)
│
└── 05_Project_Documents/           # 【第四层：全套系统设计文档与开发指南】
    ├── 程序解读.md                  # MIT 原作 MATLAB 算法与数学机理长文解读
    ├── ZYNQ_EVM_运动放大移植开发指南.md  # ZYNQ 软硬件全流程工程移植手册 (含四重抗噪与 Vivado BD 指南)
    |—— ZYNQ_EVM_硬件接线与板级实操全联调手册 # 从硬件实物接线、跳线拨码、Vivado 硬件确认、Vitis 编译烧录到 PC 端 PyQt5 上位机全功能联调的完整操作细节
```

---

## ⚡ 核心功能与技术特色

### 1. 3 大精炼核心显示模式无缝选通
系统聚焦实际工程监测场景，彻底剔除冗余模式，支持通过 PC 界面按键或串口即时切换：
1. **模式 1：原始视频直通 (Bypass)** —— 原汁原味输出传感器画质；
2. **模式 2：局部/全屏运动微动放大 (Motion Magnified)** —— 针对有效微动信号乘以 $\alpha$ 倍线性放大并饱和叠加；
3. **模式 3：局部/全屏热力图半透明叠加 (Heatmap Overlay - 核心推荐)** —— 振动能量场经时域平滑后映射至 Jet / Viridis 伪彩色谱，以自适应 Alpha 权重叠加在原始画面上，动静分明。

### 2. 全双工 GUI 智能交互与 USB-HDMI 极速视频捕获
系统全面打通软硬件协同闭环，提供无缝大屏体验：
* **USB-HDMI 视频采集卡免驱直连**：板卡处理后的视频经板载 HDMI 接口输出，通过免驱 USB 采集卡接入 PC（UVC 协议），上位机以 60FPS 极速呈现硬件加速画面，电脑 CPU 占用率不足 2%；
* **全双工按钮与滑块双向控制**：界面提供模式切换按钮与 Alpha（$1\times \sim 150\times$）、Gate（$1 \sim 100$）抗噪门限滑动条，指令即时写入开发板 AXI 寄存器；

### 3. 四重软硬件协同抗噪体系（根除雪花杂斑）
普通 CMOS 在微动放大 30~50 倍时极易将传感器高频散粒噪点同步放大形成“暴风雪杂斑”。本项目构建了四重防线：
1. **源头滤波**：SCCB 配置激活 OV5640 片上 2D Denoise 硬件 ISP；
2. **锁死增益**：强制锁定最低原生模拟增益 1.0x (ISO 100) 并固定曝光，杜绝高 ISO 噪点；
3. **时域死区门限滤波 (Hardware Noise Coring)**：在 FPGA 硬件中通过单周期比较器：
   $$\Delta y_{\text{clean}} = \begin{cases} \text{sign}(\Delta y) \cdot (|\Delta y| - \text{Gate}), & |\Delta y| > \text{Gate} \\ 0, & |\Delta y| \le \text{Gate} \end{cases}$$
   低于噪声死区门限的本底白噪彻底归零，仅放大部分真实物理微动；
4. **空间降采样增益**：2x2 空间均值预处理在统计学上使高斯随机白噪方差减半（SNR 提升 6dB）。

### 4. 振动安全阈值监测预警 (VIBRATION ALARM)
硬件底层实时统计全帧最大微动峰值并通过 AXI-Lite 寄存器上报，上位机实时解包并绘制微动波形与 FFT 振动主频。一旦微动位移超过设定容许安全阈值，立即触发声光报警，助力工业探伤与防失衡预警。

---

## 🚀 快速上手与运行

### 1. 运行 PC 端 PyQt5 智能上位机
确保已安装 Python 3.8+ 及依赖库：
```bash
pip install opencv-python numpy PyQt5 pyqtgraph pyserial
```
启动上位机：
```bash
cd "04_PC_Dashboard_PyQt5"
python evm_monitor_pc.py
```
* 点击“启动摄像头/采集卡”选择输入源（输入 `1` 即可接入 USB-HDMI 采集卡 60FPS 硬件加速视频流）；
* 点击“连接开发板串口”接入 ZYNQ 串口，实现按钮选模与滑块动态调参；
* 在界面底部无缝切换显示模式、调色板，或拖动滑块调整 Alpha 倍数与抗噪门限；
* 观察视频正下方 4 项核心大字指标与右下角展开的实时振动位移波形与 FFT 频谱。

### 2. Vivado 硬件 PL 端工程搭建、顶层配置与仿真综合指南

> [!IMPORTANT]
> **顶层模块与文件归类核心规范**：
> 1. `tb_` 开头的文件（如 `tb_evm_pl_top.v`）为 **仿真测试激励平台 (Testbench)**，内部包含 `#10` 延时、`initial` 初始化等不可综合语法且无真实物理管脚，**绝对不能添加进 Design Sources 作为综合顶层**！Vivado 限制其作为设计顶层是标准的保护机制，并不代表 PL 架构存在问题。
> 2. Vivado 严格区分 **设计源文件 (Design Sources)** 与 **仿真源文件 (Simulation Sources)**，必须分流导入并针对不同任务设置对应顶层。

#### 步骤 2.1：创建工程与器件选型
* 打开 Xilinx Vivado (推荐 2019.2 或更高版本)；
* 新建 RTL Project，目标芯片选择正点原子领航者搭载的 **`xc7z020clg400-2`**。

#### 步骤 2.2：规范化添加源文件（关键分流操作）
1. **添加设计源文件 (Design Sources)**：
   * 点击 Sources板块中的 `Add Sources`，选择 **Add or create design sources**；
   * 添加 `02_ZYNQ_PL_Hardware/rtl/` 目录下除了 `tb_evm_pl_top.v` 之外的所有模块（涵盖 `camera/`, `preprocess/`, `evm/`, `heatmap/`, `display/`, `ps_pl_interface/` 及 `evm_pl_top.v`）。
2. **添加仿真源文件 (Simulation Sources)**：
   * 再次点击 `Add Sources`，选择 **Add or create simulation sources**；
   * 单独添加测试激励平台：`02_ZYNQ_PL_Hardware/rtl/top/tb_evm_pl_top.v`。
3. **添加物理管脚与时钟约束 (Constraints)**：
   * 点击 `Add Sources`，选择 **Add or create constraints**，添加 `02_ZYNQ_PL_Hardware/constraints/pin_constraints.xdc`。

#### 步骤 2.3：一致性与端口语法校验
在工程根目录下执行自动化校验脚本，确认各模块接口连线与位宽 100% 对齐：
```bash
cd "02_ZYNQ_PL_Hardware/scripts"
python check_rtl_consistency.py
```
*(预期输出：`ALL MODULES INSTANTIATED WITH 100% PORT CONSISTENCY!`)*

#### 步骤 2.4：根据任务正确设置“顶层文件 (Top Module)”

| 任务场景 | 应选顶层模块 (Top Module) | Vivado 具体操作路径 | 操作与说明 |
| :--- | :--- | :--- | :--- |
| **A. 行为级逻辑仿真**<br>*(Run Simulation)* | **`tb_evm_pl_top`**<br>(对应 `tb_evm_pl_top.v`) | 展开 `Sources -> Simulation Sources -> sim_1`<br>右键点击 `tb_evm_pl_top.v` $\rightarrow$ **Set as Top** | 点击左侧 `Flow Navigator -> Run Simulation -> Run Behavioral Simulation`，观察虚拟 720P 图像流注入、IIR 滤波及伪彩叠加波形。 |
| **B. 纯 PL 硬件综合评估**<br>*(Run Synthesis)* | **`evm_pl_top`**<br>(对应 `evm_pl_top.v`) | 展开 `Sources -> Design Sources`<br>右键点击 `evm_pl_top.v` $\rightarrow$ **Set as Top** | 点击左侧 `Flow Navigator -> Run Synthesis`，评估 FPGA 片上 LUT、BRAM、DSP48E1 硬件资源消耗及建立/保持时序收敛情况。 |
| **C. ZYNQ 全系统联调**<br>*(Generate Bitstream)* | **`system_wrapper`**<br>(自动生成的 BD 包装) | 在 Block Design 中集成 PS7、VDMA、`evm_pl_top` 与 `rgb2dvi`，右键 `.bd` -> **Create HDL Wrapper** 并设为顶层 （**补充1，补充2**）| 执行完整 Implementation 生成 Bitstream，并点击 `File -> Export -> Export Hardware (包含 bitstream)` 生成 `.xsa` 供 Vitis 导入。 |

#### 补充1：Block Design (BD) 全系统图形化互联标准流程

1. **新建设计**：点击左侧 `Flow Navigator -> IP INTEGRATOR -> Create Block Design`，命名为 `system`；
2. **添加核心 IP 与模块**：
   * 点击 `+`（Add IP）添加 **`ZYNQ7 Processing System`**；
   * 画板空白处右键选择 **`Add Module...`**，引入本工程的核心算法核 **`evm_pl_top`**；
   * 再次右键选择 **`Add Module...`**，引入本项目自带的纯 Verilog HDMI 发送核 **`rgb2dvi`**（免下载外部第三方 IP 库）；
   * 点击 `+`（Add IP）添加 **`Constant`**（IP 名称为 `xlconstant`，默认输出 `1'b0`，命名为 `xlconstant_0`）；
   * 再次点击 `+`（Add IP）添加第二个 **`Constant`**（双击将其常量值修改为 **`1`**，命名为 `xlconstant_1`）；
3. **配置 ZYNQ PS7 外设、时钟与串口（激活摄像头与通信关键步）**：
   * 双击画板上的 **`processing_system7_0`** 打开配置向导；
   * 进入 **`MIO Configuration`** -> 展开 **`I/O Peripherals`**：
     * **务必勾选 `UART 1`**（保持默认 **`MIO 48-49`**）：确保串口终端打印与 PyQt5 上位机全双工通信；
     * **务必勾选 `I2C 0`**，接口位置选择 **`EMIO`**：用于 ARM PS 端通过 SCCB 总线配置 OV5640 寄存器、激活 720P@60fps 与 2D 降噪 ISP；
   * 进入 **`Clock Configuration`** -> 展开 **`PL Fabric Clocks`**：
     * 确认 **`FCLK_CLK0`** 保持 **`50 MHz`**（系统主时钟）；
     * 勾选使能 **`FCLK_CLK1`**，设置请求频率为 **`24.000000 MHz`**（为 OV5640 提供主驱动时钟 XCLK）；
   * 进入 **`DDR Configuration`** 选择与开发板相匹配的 DDR3 芯片型号（如 MT41J256M16 RE-125 等）；
   * 点击 **OK** 保存退出；
4. **完成总线与端口互联（核对下表全部 15 组连线）**：

| 模块与端口 | 互联目标 | 说明与接线目的 |
| :--- | :--- | :--- |
| `processing_system7_0 / M_AXI_GP0` | `evm_pl_top_0 / s_axi` | 点击顶部 **Run Connection Automation** 自动通过 AXI SmartConnect 互联 |
| `processing_system7_0 / FCLK_CLK0` | `evm_pl_top_0 / clk` 与 `rgb2dvi_0 / PixelClk` | 50MHz 系统主时钟统一驱动算法与显示核 |
| `rst_ps7_0_50M / peripheral_aresetn` | `evm_pl_top_0 / rst_n` 与 `rgb2dvi_0 / rst_n` | 全局同步低有效复位 |
| `evm_pl_top_0 / m_axis_state` | `evm_pl_top_0 / s_axis_state` | **输出接口直连自身输入接口**（启用片上 RAM 闭环运行，免外挂 VDMA） |
| `evm_pl_top_0 / vid_out_rgb[23:0]` | `rgb2dvi_0 / vid_pData[23:0]` | 24-bit RGB 视频数据流 |
| `evm_pl_top_0 / vid_out_valid` | `rgb2dvi_0 / vid_pVDE` | 像素数据有效（Data Enable） |
| `evm_pl_top_0 / vid_out_vs` | `rgb2dvi_0 / vid_pVSync` | 场同步信号 |
| `xlconstant_0 / dout[0:0]` (恒0) | `rgb2dvi_0 / vid_pHSync` 与 `SerialClk` | 提供固定接地电平 0，消除浮空警告 |
| **`processing_system7_0 / IIC_0`** | **右键选择 Make External** | **【摄像头配置】** 引出 `IIC_0_0` (对应 XDC 引脚 SCL: `N17`, SDA: `P18`) |
| **`processing_system7_0 / FCLK_CLK1`** | **右键选择 Make External** 并重命名为 `cmos_xclk_0` | **【摄像头时钟】** 输出 24MHz 驱动时钟 (对应 XDC 引脚 `R16`) |
| **`xlconstant_0 / dout[0:0]` (恒0)** | **引出一条支路 Make External** 并重命名为 `cmos_pwdn_0` | **【摄像头休眠解除】** 强制拉低接地 0，退出低功耗待机模式 (对应 XDC 引脚 `R17`) |
| **`xlconstant_1 / dout[0:0]` (恒1)** | **右键选择 Make External** 并重命名为 `cmos_rst_n_0` | **【摄像头复位释放】** 强制拉高接高电平 1，释放硬件复位 (对应 XDC 引脚 `P15`) |
| `evm_pl_top_0 / cmos_*` (共4组) | **依次右键选择 Make External** | 引出 `cmos_pclk_0`, `cmos_vsync_0`, `cmos_href_0`, `cmos_data_0[7:0]` 物理像素流引脚 |
| `rgb2dvi_0 / tmds_*` 与使能 (共5组) | **依次右键选择 Make External** | 引出 `tmds_clk_p_0`, `tmds_clk_n_0`, `tmds_data_p_0`, `tmds_data_n_0`, `hdmi_out_en_0` 高清输出引脚 |
| `processing_system7_0 / DDR & FIXED_IO` | **依次右键选择 Make External** | 引出 ARM 专用物理硬核引脚 |

#### 补充2：生成顶层 Wrapper、校验与导出 XSA（避坑必读）

> [!CAUTION]
> **绝对不能直接对 `evm_pl_top.v` 运行 Implementation（避坑 Place 30-58 报错）**：
> `evm_pl_top.v` 带有 180 根芯片内部 AXI 总线。若直接将其作为顶层进行实现，Vivado 会误将内部总线映射为物理外部引脚，导致 `Number of unplaced IO Ports (180) is greater than available pins (125)` 报错退出！必须生成 Block Design Wrapper 作为全系统唯一顶层！

1. **设计规则检查**：在 Block Design 画板按快捷键 **`F6`**（或点击顶部对勾图标 **Validate Design**），确认提示 `Validation successful`；
2. **生成顶层文件**：保存画板后，在 Sources 窗口中右键点击 `system.bd` 文件 -> 选择 **`Create HDL Wrapper...`**（勾选 *Let Vivado manage wrapper and auto-update*）；
3. **确认顶层切换**：确认 Sources 树形图中的顶层粗体模块已自动更新为 **`system_wrapper`**；
4. **生成比特流**：点击左侧工具栏最下方 **Generate Bitstream**（一次性自动化完成 Synthesis、Implementation 与 Bitstream 生成）；
5. **导出硬件描述包**：点击顶部菜单 `File -> Export -> Export Hardware...`，选择 **Include bitstream**，生成供 Vitis 导入的 **`system_wrapper.xsa` 文件**。

---

### 3. Vitis 2025.2 (Unified IDE) ARM PS 端裸机工程编译与全系统下板联调

> [!NOTE]
> **Vitis 2025.2 架构变革说明**：
> 从 2023.2 起，AMD 全面转向基于 **VS Code 内核的 Vitis Unified IDE**，彻底革新了旧版 Eclipse 体系。核心升级为 **组件架构 (Component Model)**：必须先基于 `.xsa` 构建 **Platform Component (平台组件)** 并编译生成 BSP，再创建 **Application Component (应用组件)** 导入源码与下板运行。

#### 步骤 3.1：启动 Vitis 2025.2 并基于 `.xsa` 构建 Platform Component (平台组件)
1. **启动 IDE**；
2. **选择工作空间**：设置 Workspace 目录；
3. **新建平台组件向导**：在欢迎页点击 **`Create Platform Component`**：
   * **Component name**：输入平台名称，例如 `evm_platform`，点击 **Next**；
   * **Flow**：在 Browse...中，选中 Vivado 导出的 `system_wrapper.xsa`；
   * **Operating System & Processor**：操作系统选择 **`standalone`**（裸机），目标处理器默认识别为 **`ps7_cortexa9_0`**；
   * 点击 **Next** -> **Finish** 完成创建。
4. **编译平台组件（关键步骤）**：
   * 在左侧 **VITIS COMPONENTS** 列表树中选中 `evm_platform`；
   * 在下方的 **FLOW** 流程导航面板中，点击 **`Build`**；
   * Vitis 将自动调用 `SDTgen` 与 `Lopper` 解析 XSA 并生成系统设备树、Board Support Package (BSP) 与 `.xpfm` 文件，等待提示 *Build Finished Successfully*。

#### 步骤 3.2：创建 Application Component (应用组件) 并导入现成 C 驱动源码
1. **新建应用组件**：点击顶部菜单 **`File -> New Component -> Application`**；
2. **配置应用向导**：
   * **Component name**：输入应用名称，例如 `evm_ps_control`，点击 **Next**；
   * **Select Platform**：选择刚才构建完成的平台组件 **`evm_platform`**，点击 **Next**；
   * **Domain**：选择默认的裸机域 **`standalone_ps7_cortexa9_0`**，点击 **Next**；
   * 点击 **Finish**。
3. **导入本项目现成 C 语言代码**：
   * 在左侧组件树中展开：`evm_ps_control -> Sources -> src`；
   * 将本项目现成的 3 个源文件导入 `src` 文件夹中：
     * [`03_ZYNQ_PS_Software/app/main_ps_control.c`]（全双工 GUI 滑块与串口双模主控）
     * [`03_ZYNQ_PS_Software/drivers/ov5640_sccb.c`]（SCCB 硬件 2D 去噪 ISP 驱动）
     * [`03_ZYNQ_PS_Software/drivers/ov5640_sccb.h`]
4. **编译应用**：
   * 在 **FLOW** 流程面板中选中 `evm_ps_control`，点击 **`Build`**；
   * 编译成功生成可执行文件 `evm_ps_control.elf`。

> [!TIP]
> **常见编译报错与解决办法 (Vitis 编译避坑指南)**：
> 1. **报错 `fatal error: ../drivers/ov5640_sccb.h: No such file or directory`**：
>    * **原因**：当把所有 C 源码直接放进 Vitis 的平铺 `src/` 目录时，原相对路径失效。
>    * **解决**：本仓库 `main_ps_control.c` 现已升级智能探测（使用 `#if __has_include` 自动适配平铺目录与子目录），直接将最新的 [`main_ps_control.c`] 覆盖到 Vitis `src/` 即可。
> 2. **报错 `fatal error: xiicps.h: No such file or directory` 或上位机画面呈现彩条**：
>    * **原因**：Vivado Block Design 中的 `processing_system7_0` 未启用 PS 内部 I2C 外设（`I2C 0` 分配至 `EMIO`）或未引出 `cmos_pwdn`/`cmos_rst_n` 控制信号。这会导致 Vitis 板级支持包 (BSP) 缺少 `xiicps.h`，跳过 SCCB 写入，摄像头未被唤醒，采集卡因无信号而输出内置测试彩条！
>    * **标准解决（推荐）**：在 Vivado Block Design 中双击 `processing_system7_0` -> `MIO Configuration` -> `I/O Peripherals` -> 勾选 `I2C 0`（选 `EMIO`），将导出的 `IIC_0` 端口 Make External，并引出 `cmos_pwdn_0`（接地0）、`cmos_rst_n_0`（接高1）与 `cmos_xclk_0`（FCLK_CLK1 24MHz）。重新生成 Bitstream 与 XSA，在 Vitis 中点击 Platform -> **Update Hardware Specification** 并重新编译，OV5640 即可被成功点亮并输出流畅画面！



#### 步骤 3.3：硬件连接、烧录运行与多端联调
1. **硬件连接与跳线**：
   * 开发板拨码开关配置为 **JTAG 模式**；
   * 连接 JTAG 下载线与 USB-UART 串口线至 PC；
   * OV5640 摄像头插入底板 P2 接口，连接 HDMI 输出线至显示器（或通过 USB-HDMI 采集卡插入 PC）；
   * 开发板上电开机。
2. **目标连接确认 (Target Connections)**：
   * 在 Vitis Unified IDE 界面顶部的 **Vitis**展开 **Target Connections**，确认 `Hardware Server`（默认 `localhost`）连接正常，已检测到 `xc7z020` 核心。
3. **下板运行 (Run on Hardware)**：
   * 在左侧 **FLOW** 流程导航中点击 `evm_ps_control`；
   * 点击 **`Run`**（或点击齿轮图标展开 Launch Configurations，确认勾选 *Program FPGA* 下载比特流并执行 *ps7_init* 初始化）；
   * Vitis 将一键自动化完成：烧录 FPGA Bitstream -> 初始化 ARM PS7 时钟与 DDR3 -> 下载 `evm_ps_control.elf` 到板卡并自动启动运行。
4. **全双工交互体验**：
   * **串口监视器**：可在顶部菜单点击 **`Vitis -> Serial Monitor`**（或打开 PC 端串口助手 / 本工程 PyQt5 智能上位机），波特率设置为 **`115200`**；
   * **画面呈现**：外接显示器或 PC 上位机画面瞬间点亮，呈现 60FPS 超流畅微动/热力图画质；
   * **控制联动**：
     * 可在串口键盘按下 `1`（原画直通）、`2`（微动放大）、`3`（热力图叠加）、`+` / `-`（调倍数）、`c`（切色表）；
     * 或启动 PC 上位机 [`evm_monitor_pc.py`]，点击按钮选模、拖动 Alpha 与抗噪 Gate 滑块动态调参，大字指标卡片与 FFT 频谱实时同步刷新！

---
*本项目包含详尽的理论数学推导、C 仿真模型、Verilog 硬件流水线与交互界面：*
* **实物接线与下板实操**：请参阅 [`05_Project_Documents/ZYNQ_EVM_硬件接线与板级实操全联调手册.md`]；
* **算法移植与理论全景**：请参阅 [`05_Project_Documents/ZYNQ_EVM_运动放大移植开发指南.md`]；
* **开发全景与实战排错记录**：请参阅 [`05_Project_Documents/ZYNQ_EVM_完整对话记录与开发全貌.md`]。

