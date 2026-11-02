# ZYNQ EVM 运动微动放大与热力图系统 —— 硬件接线与板级实操全联调手册

本手册专门面向 **ZYNQ-7000 (XC7Z020)** 软硬件协同 EVM 运动放大系统，详细梳理从**硬件实物接线、跳线拨码、Vivado 硬件确认、Vitis 2025.2 编译烧录**到 **PC 端 PyQt5 上位机全功能联调**的完整操作细节与避坑指南。

---

## 目录
1. [硬件环境与器材准备清单](#一硬件环境与器材准备清单)
2. [硬件物理接口与接线拓扑](#二硬件物理接口与接线拓扑)
3. [两大显示联调链路选择](#三大显示联调链路选择)
4. [Vivado 硬件关键配置核查](#四vivado-硬件关键配置核查)
5. [Vitis 2025.2 下板烧录与启动实操](#五vitis-20252-下板烧录与启动实操)
6. [PC 端 PyQt5 智能上位机操作指南](#六pc-端-pyqt5-智能上位机操作指南)
7. [全链路常见故障排查与诊断 (FAQ)](#七全链路常见故障排查与诊断-faq)

---

## 一、硬件环境与器材准备清单

在开始下板联调前，请确保准备齐备以下硬件设备与线缆配件：

| 序号 | 硬件设备 / 线缆 | 规格要求 / 推荐型号 | 数量 | 用途与作用 |
| :---: | :--- | :--- | :---: | :--- |
| **1** | **ZYNQ-7020 开发板** | 芯片型号 `XC7Z020-CLG400-2`（正点原子领航者 / ALINX AX7020 等） | 1 块 | 核心运算底座，承载 PL 图像处理流水线与 PS 软核交互调度 |
| **2** | **OV5640 摄像头模组** | ATK-OV5640（单目 18-Pin 或双目 40-Pin 均可） | 1 个 | 实时采集 720P@60FPS 原始视频源并输入 PL 端 |
| **3** | **USB-HDMI 视频采集卡** | 免驱 USB 3.0 / USB 2.0 高清 HDMI 视频采集卡（支持 1080P/720P@60FPS） | 1 个 | 采集开发板 HDMI 输出的高清画面并直连 PC 独立 USB 接口回传 |
| **4** | **USB Hub (集线器)** | 4 口 USB 2.0 / 3.0 集线器 | 1 个 | 汇聚板载 UART 串口与 JTAG 下载口，统一接 PC 独立 USB 接口 |
| **5** | **Micro-USB 数据线** | 优质高速 Micro-USB 数据线 | 2 根 | 分别用于连接板载 UART 与 JTAG 接口至 USB Hub |
| **6** | **标准 HDMI 高清线** | 标准 A 型公对公 HDMI 高清线（长度 1m~1.5m） | 1 根 | 连接开发板 HDMI OUT 接口至 USB-HDMI 视频采集卡 |
| **7** | **直流电源适配器** | 12V / 2A~3A 直流稳压电源 | 1 个 | 为 ZYNQ 开发板系统提供稳定电力供应 |
| **8** | **PC 调试主机** | 预装 Windows 10/11，具备 2 个独立 USB 接口 | 1 台 | 运行 Vivado/Vitis 调试环境与 PyQt5 智能上位机仪表盘 |

---

## 二、硬件物理接口与接线拓扑

### 1. 硬件连接拓扑图

```mermaid
graph TD
    classDef fpga fill:#1E293B,stroke:#38BDF8,stroke-width:2px,color:#FFFFFF;
    classDef pc fill:#0F172A,stroke:#10B981,stroke-width:2px,color:#FFFFFF;
    classDef dev fill:#334155,stroke:#F59E0B,stroke-width:1.5px,color:#FFFFFF;
    classDef hub fill:#1E1B4B,stroke:#A855F7,stroke-width:2px,color:#FFFFFF;

    subgraph Power["供电单元"]
        PWR["12V 直流电源适配器"] -->|DC 供电插孔| BOARD_PWR["开发板电源开关 (ON)"]
    end

    subgraph Board["ZYNQ-7020 开发板 (底板与核心板)"]
        BOOT["拨码开关: JTAG 模式"]
        CAM_PORT["P2/J3 排母接口 (DVP 视频输入)"]:::fpga
        HDMI_OUT["HDMI OUT (板载高清输出接口)"]:::fpga
        JTAG_PORT["板载 JTAG 调试口"]:::fpga
        UART_PORT["USB_UART 串口 (PS MIO 48-49)"]:::fpga
    end

    subgraph Peripherals["外设与集线互联"]
        OV5640["ATK-OV5640 摄像头模组"]:::dev
        USB_HUB["USB Hub (集线器/扩展坞)"]:::hub
        CAPTURE["USB-HDMI 视频采集卡"]:::dev
    end

    subgraph PC_Host["PC 调试主机 (Windows)"]
        PC_USB1["PC 独立 USB 接口 1 (视频采集流)"]:::pc
        PC_USB2["PC 独立 USB 接口 2 (总线汇总)"]:::pc
        VITIS["Vitis 2025.2 (程序烧录/ELF运行)"]:::pc
        DASHBOARD["PyQt5 智能上位机 (evm_monitor_pc.py)"]:::pc
    end

    OV5640 -->|直插排母| CAM_PORT
    
    %% UART 与 JTAG 汇聚到 USB Hub
    UART_PORT -->|Micro-USB 数据线| USB_HUB
    JTAG_PORT -->|Micro-USB 数据线| USB_HUB
    USB_HUB -->|USB 上行线| PC_USB2
    PC_USB2 -.->|COM 虚拟串口 (115200bps)| DASHBOARD
    PC_USB2 -.->|JTAG Server (hw_server)| VITIS

    %% 视频采集卡直连 PC
    HDMI_OUT -->|标准 HDMI 线| CAPTURE
    CAPTURE -->|USB 接口直接插入| PC_USB1
    PC_USB1 -.->|UVC 视频流 (720P@60FPS)| DASHBOARD
```


### 2. 板载接口逐一接线指引

#### (1) 启动模式配置 (BOOT 拨码开关)
* **位置**：板载启动模式拨码开关（通常标记为 `BOOT` 或包含 `SW1`/`SW2` 开关）；
* **设置状态**：必须配置为 **JTAG 启动模式**。
  * *例如正点原子领航者*：将拨码开关打到全 `0`（或根据丝印提示选择 JTAG 位置）；
  * *目的*：防止上电时芯片尝试从外部 SD 卡或 QSPI Flash 自动加载，保持空闲状态等待 Vitis JTAG 接管。

#### (2) 摄像头模块插入 (ATK-OV5640-DUAL 双目模组插在 J3 插座)
> [!IMPORTANT]
> **【ATK-OV5640-DUAL 双目模组插在底板 J3 插孔】**
> * **排母规格**：40 针 (2×20 双排弯排母)，直接插入开发板底板左侧的 **`J3` 扩展排母** 上。
> * **单目抓取机制**：双目模组插在 `J3` 时，其板载左目 **CMOS1 (OV1)** 将被用于 720P 运动放大视频流抓取；
> * **时钟源特性**：双目模组内部自带两个 24MHz 有源晶振（Y1、Y2），**无需 FPGA 外部提供 XCLK**。
> * **安全安装规范**：**务必断电插拔**，防呆方向对齐，压紧到位。

#### (3) 物理串口原理剖析 (硬件硬连线直通 MIO 14/15 -> UART 0)
> [!NOTE]
> **【CH340 物理硬连线直通说明】**
> * 查阅开发板底板原理图（`POWER&UART.SchDoc`）与核心板原理图（`04_FPGA Bank500`）：
>   板载 CH340C 芯片通过 PCB 铜箔走线**直接焊死直通**连接至核心板的 **`PS_MIO14` (RX)** 和 **`PS_MIO15` (TX)**，底板上**无跳线排针**。
> * 在 ZYNQ-7000 硬件架构中，`MIO 14/15` 对应的是 **PS 硬核 `UART 0`**（物理基地址 `0xE0000000`）。
> * **工程已自动完成配置**：Vivado Block Design 已使能 **`UART 0 (MIO 14..15)`**，串口输出完全畅通！

#### (4) 供电规格要求 (始终接入 12V 独立直流供电)
> [!IMPORTANT]
> **【必须接入 12V 独立直流电源适配器】**
> * ZYNQ-7020 + DDR3 + PL 全硬件 EVM 加速流水线 + HDMI 串行驱动 + OV5640 摄像头在启动时峰值功耗达到 **5W ~ 8W**。
> * 必须将 12V 电源适配器插入板载 DC 插座，并将电源开关拨到 **ON**（板上电源指示灯常亮）。

#### (5) JTAG 调试下载口连接
* **接口标识**：板载 **`JTAG`**（Micro-USB 口）或标准 14-Pin JTAG 仿真器接口；
* **连接方法**：使用第一根 Micro-USB 数据线连接开发板 JTAG 口至 **USB Hub**。

#### (6) USB-UART 通信串口连接
* **接口标识**：板载 **`USB_UART`**（内置 CH340 芯片）；
* **连接方法**：使用第二根 Micro-USB 数据线连接该接口至 **USB Hub**，随后将 **USB Hub 连接至 PC 电脑的独立 USB 端口**；
* **系统映射**：插入后，打开 PC 电脑的【设备管理器】->【端口 (COM 和 LPT)】，确认已成功识别出串口（如 `USB-SERIAL CH340 (COM7)`）。

> [!TIP]
> **【为什么 JTAG 与 UART 接 USB Hub，而采集卡直连 PC？】**
> * **高低速总线隔离，保障系统稳定性**：
>   * **JTAG 与 UART 汇聚至 USB Hub**：JTAG 仅在下载 Bitstream 与 ELF 时有短暂脉冲数据，平时处于监听待命状态；UART 串口波特率为 115200bps，每秒实际数据量仅约 11KB 左右。两者的低速控制流通过 USB Hub 合并汇聚，仅占用 PC 一个 USB 接口，线缆规整且完全不会发生总线拥塞；
>   * **USB-HDMI 视频采集卡直连 PC 独立接口**：采集卡承载 720P@60FPS 连续视频流传输，瞬时数据吞吐量大且对供电稳定性要求苛刻。直插 PC 原生 USB 端口可以独占系统总线通道与稳定供电，彻底杜绝视频撕裂、丢帧或因 Hub 供电波动导致的掉线。

> [!TIP]
> **【全程无需打开第三方串口助手，上位机独占一站式搞定】**
> * **无需开启第三方助手**：PC 端 PyQt5 上位机（`evm_monitor_pc.py`）内部已自带全双工串口引擎，负责下发按钮/滑块指令并实时解析 12 字节遥测数据包绘制 FFT 频谱；
> * **避免端口冲突**：Windows 系统对 COM 串口具有**独占性**，若事先打开了 XCOM/SSCOM 等助手，会导致上位机提示“串口已被占用 / 无法打开”。因此**全程直接使用上位机即可**！

#### (7) HDMI 视频输出口与采集卡连接
* **接口标识**：板载 **`HDMI OUT`**（标准 A 型母座）；
* **连接方法**：插入标准 HDMI 高清线，线缆一端接开发板 HDMI OUT 接口，另一端连接至 **USB-HDMI 视频采集卡**，**采集卡直接插入 PC 电脑的独立 USB 接口**。

#### (8) 电源接口与开机
* 插入 12V 直流电源适配器插头至开发板电源插孔；
* 检查所有跳线与接线无误后，将电源船型开关打到 **ON**；
* 确认板上电源指示灯（`POWER`/`VCC`）常亮。

---

## 三、两大显示联调链路选择

根据开发环境与设备条件，本系统支持两种无缝联调方案：

| 方案对比项 | 方案 A：USB-HDMI 采集卡一体化方案 (强烈推荐) | 方案 B：外接独立 HDMI 显示器方案 |
| :--- | :--- | :--- |
| **硬件需求** | PC + USB-HDMI 采集卡 (几十元) + HDMI 线 | PC + 独立显示器 (含 HDMI 输入) + HDMI 线 |
| **画面呈现位置** | **PC 屏幕**：内嵌在 PyQt5 上位机界面左侧窗口 | **独立显示器全屏**：显示大尺寸硬件处理视频 |
| **上位机角色** | 集成“视频监视器 + 4项关键参数卡片 + 调参控制台 + 实时FFT频谱”于一体 | 充当“纯调参控制台 + 实时FFT频谱监控屏” |
| **使用体验** | **极致紧凑**，单笔记本即可完成实验演示与科研答辩 | 适合大屏展示、远距离振动体观测 |
| **连接简图** | `板卡 HDMI OUT` -> `采集卡` -> `PC USB` | `板卡 HDMI OUT` -> `显示器 HDMI IN` |

---

## 四、Vivado 硬件关键配置核查

在导出 `.xsa` 前，请务必复查 Block Design 中如下核心配置与接线，确保软硬件握手与摄像头控制链路 100% 畅通：

### 1. PS7 外设与时钟配置核查（激活摄像头与通信的关键）
1. **PS7 串口 UART 1 启用确认 (全双工通信基石)**：
   * 打开 Block Design，双击打开 `processing_system7_0`；
   * 进入 **`MIO Configuration`** $\rightarrow$ 展开 **`I/O Peripherals`**；
   * 确认已勾选 **`UART 1`**（IO 分配为 **`MIO 48-49`**）；
   * *重要意义*：此选项是板载 USB-UART 通信的物理保证，也是 Vitis 自动生成 `uartps` 驱动库与 `xuartps_hw.h` 头文件的必要前提！
2. **PS7 I2C 0 启用确认 (OV5640 摄像头唤醒基石 - 必选)**：
   * 在 **`I/O Peripherals`** 中勾选 **`I2C 0`**，接口位置选择 **`EMIO`**；
   * 点击 OK 后在画板上将生成的 `IIC_0` 端口右键 **Make External**（引出为外部引脚，对应 XDC 中绑定到 SCL: `N17`，SDA: `P18`）；
   * *重要意义*：OV5640 是裸 CMOS 传感器，必须通过 I2C 写入初始化寄存器配置。若未引出此总线，摄像头传感器不会起振，上位机将只呈现无信号彩条！
3. **PS7 摄像头时钟 FCLK_CLK1 (24MHz) 启用确认**：
   * 进入 **`Clock Configuration`** $\rightarrow$ 展开 **`PL Fabric Clocks`**；
   * 勾选使能 **`FCLK_CLK1`**，设置频率为 **`24.000000 MHz`**；
   * 在画板上将 `FCLK_CLK1` 右键 **Make External** 并重命名为 **`cmos_xclk_0`**（绑定至引脚 `R16`），为摄像头提供主驱动时钟。

### 2. Block Design 全系统标准连线对照表 (全部 15 组)

| 模块与端口 | 互联目标 | 关键作用与接线目的 |
| :--- | :--- | :--- |
| `processing_system7_0 / M_AXI_GP0` | `evm_pl_top_0 / s_axi` | 通过 AXI SmartConnect 互联，用于 PS 配置算法参数与读取峰值 |
| `processing_system7_0 / FCLK_CLK0` | `evm_pl_top_0 / clk` 与 `rgb2dvi_0 / PixelClk` | 50MHz 系统主时钟统一驱动算法与显示流水线 |
| `rst_ps7_0_50M / peripheral_aresetn` | `evm_pl_top_0 / rst_n` 与 `rgb2dvi_0 / rst_n` | 全局低有效同步复位 |
| `evm_pl_top_0 / m_axis_state` | `evm_pl_top_0 / s_axis_state` | **输出接口直连自身输入接口**（片上 RAM 闭环历史帧存储，免外挂 VDMA） |
| `evm_pl_top_0 / vid_out_rgb[23:0]` | `rgb2dvi_0 / vid_pData[23:0]` | 24-bit 处理后视频流输入 HDMI 发送核 |
| `evm_pl_top_0 / vid_out_valid` | `rgb2dvi_0 / vid_pVDE` | 视频数据有效指示 |
| `evm_pl_top_0 / vid_out_vs` | `rgb2dvi_0 / vid_pVSync` | 场同步信号 |
| `xlconstant_0 / dout[0:0]` (恒0) | `rgb2dvi_0 / vid_pHSync` 与 `SerialClk` | 固定接地 0，消除浮空警告 |
| **`processing_system7_0 / IIC_0`** | **右键选择 Make External** | **【摄像头 I2C 配置】** 引出 `IIC_0_0` (约束至 SCL: `N17`, SDA: `P18`) |
| **`processing_system7_0 / FCLK_CLK1`** | **右键 Make External** 命名为 `cmos_xclk_0` | **【摄像头主时钟】** 输出 24MHz 时钟源 (约束至 `R16`) |
| **`xlconstant_0 / dout[0:0]` (恒0)** | **引出支路 Make External** 命名为 `cmos_pwdn_0` | **【摄像头休眠解除】** 强制拉低接地 0，退出低功耗 (约束至 `R17`) |
| **`xlconstant_1 / dout[0:0]` (恒1)** | **右键 Make External** 命名为 `cmos_rst_n_0` | **【摄像头复位释放】** 添加 Constant(1)，强制拉高释放复位 (约束至 `P15`) |
| `evm_pl_top_0 / cmos_*` (4组) | **依次右键选择 Make External** | 引出 `cmos_pclk_0`, `cmos_vsync_0`, `cmos_href_0`, `cmos_data_0[7:0]` 数据流引脚 |
| `rgb2dvi_0 / tmds_*` 与使能 (5组) | **依次右键选择 Make External** | 引出 TMDS 差分时钟与数据线及 `hdmi_out_en_0` |
| `processing_system7_0 / DDR & FIXED_IO` | **依次右键选择 Make External** | 引出 ARM 物理硬核引脚 |

### 3. 顶层 Wrapper 确认与 XSA 导出
1. **顶层 Wrapper 确认**：
   * 确认 Sources 窗口中加粗显示的 Top 模块为 **`system_wrapper`**，而非内部模块 `evm_pl_top.v`；
2. **导出包含比特流的 XSA**：
   * 在生成比特流后，点击 `File -> Export -> Export Hardware...`，勾选 **Include bitstream**，导出至工程根目录。

---

## 五、Vitis 2025.2 下板烧录与启动实操

### 步骤 1：确认目标连接 (Target Connections)
1. 启动 **Vitis Unified IDE** 并打开当前工作区；
2. 展开左侧下方面板中的 **Target Connections**；
3. 检查 `Hardware Server`（默认 `localhost:3121`）状态是否已变为绿色打勾（已连接）；
4. 确认在 Devices 列表中已自动识别出 **`xc7z020`** 芯片核心与 ARM Cortex-A9 双核。

### 步骤 2：一键烧录运行 (Run on Hardware)
1. 在左侧 **VITIS COMPONENTS** 列表树中选中 **`evm_ps_control`**；
2. 展开左下方 **FLOW** 导航中的 **HARDWARE** 分组；
3. 首次运行时，点击 **`Run`** 右侧的齿轮设置图标（*Launch Configurations*）：
   * 确认勾选 **`Program FPGA`**（烧录 PL 端生成的比特流）；
   * 确认勾选 **`Run ps7_init`**（执行 PS 端 PLL 时钟、DDR3 及 MIO 引脚初始化）；
   * 确认可执行文件指向 `evm_ps_control.elf`；
4. 点击 **`Run`** 按钮开始自动烧录：
   * 状态栏将依次打印：*Programming FPGA* -> *Initializing target* -> *Downloading ELF* -> *Resume execution*；
   * 板载 DONE 指示灯瞬间点亮，表示 FPGA 比特流烧录成功，ARM 核心已正式开始执行。

---

## 六、PC 端 PyQt5 智能上位机操作指南

当开发板烧录启动后，即可在 PC 电脑上打开上位机开展全功能联调。

### 步骤 1：启动上位机程序
在 PC 打开终端（PowerShell 或 CMD），进入上位机目录运行：
```bash
cd "04_PC_Dashboard_PyQt5"
python evm_monitor_pc.py
```
稍后将弹出简洁现代风格的 EVM 运动放大监测控制台。

### 步骤 2：启动视频画面输入
1. 点击左上方按钮 **【启动摄像头/采集卡】**；
2. 弹出设备选择对话框：
   * 若使用 **USB-HDMI 采集卡**：通常输入索引号 **`1`**（笔记本自带摄像头为 0，外接采集卡为 1）；
   * 若仅连接了采集卡（如台式机）：输入索引号 **`0`**；
3. 点击确定后，主视频窗口瞬间点亮，呈现 60FPS 超流畅的高清画面！

### 步骤 3：建立串口全双工通信
1. 点击上方按钮 **【连接开发板串口】**；
2. 弹出串口选择对话框，选择之前在设备管理器中识别出的端口（如 `COM3`），波特率保持默认 **`115200`**；
3. 点击确定，连接成功后按钮状态变为绿色“已连接”；
4. 视频下方的 4 项实时大字指标（`处理帧率`、`振动主频`、`位移振幅`、`系统状态`）立即开始高频动态刷新！

### 步骤 4：核心交互操作与动态调参体验
* **显示模式切换**：
  * 点击 **【1: 原始视频直通】**：FPGA 硬件旁路直通，观察物理原画；
  * 点击 **【2: 运动微动放大】**：启动 EVM 硬件加速全流水线，肉眼不可见的机械微动（如吉他弦振、微脉搏、悬臂梁颤动）被剧烈放大；
  * 点击 **【3: 热力图智能叠加】**：激活伪彩色热力图引擎，以半透明形式在原图上高亮映射微动能量场。
* **调色板切换**：
  * 点击 **【调色板: Jet 彩虹色】** 或 **【调色板: Viridis 翠绿】** 切换伪彩色查找表。
* **实时动态滑动调参**：
  * **放大倍数 Alpha (0~150)**：按住滑块左右滑动，开发板毫秒级响应，微动幅度随滑块即时增减；
  * **抗噪死区 Gate (0~100)**：滑动调节死区门限，滤除高频暗电流噪点，消除背景杂波。
* **实时频谱分析**：
  * 点击 **【显示实时振动频谱】**，弹出波形窗口，同步绘制实时微动位移时域曲线与 FFT 频域能量峰值谱。

---

## 七、全链路常见故障排查与诊断 (FAQ)

### Q1：点击 Vitis 中的 Run 提示 `Cable not found` 或找不到设备？
* **排查方法**：
  1. 检查 JTAG 数据线是否插紧，确认所用数据线具备数据传输功能；
  2. 检查开发板电源开关是否已打到 ON，板上电源指示灯是否点亮；
  3. 确认板载拨码开关处于 **JTAG 模式**；
  4. 打开设备管理器查看是否有带黄色感叹号的未识别设备，若有则需重装 Xilinx Cable 驱动（运行 Vivado 安装目录下的 `install_drivers.exe`）。

### Q2：上位机点击“启动摄像头/采集卡”后提示无法打开或报错？
* **排查方法**：
  1. 在设备管理器中展开【相机】，确认 USB 采集卡已被 Windows 正常识别为免驱摄像设备（通常名为 `USB Video`）；
  2. 在输入索引号时尝试切换 `0`、`1` 或 `2`；
  3. 检查是否有其他应用（如 Windows 自带相机、微信、OBS）正在占用该采集卡，关闭占用程序后重试。

### Q3：画面全黑或外接显示器提示“无信号输入 (No Signal)”？
* **排查方法**：
  1. 检查 OV5640 摄像头是否插在底板 **P2** 接口上，引脚接触是否良好；
  2. 检查底板 HDMI OUT 接口处是否有 5V 供电短路或松动；
  3. 确认比特流已成功烧录进 FPGA，开发板上绿色的 **DONE** 指示灯必须处于点亮常亮状态。

### Q4：上位机点击连接串口提示“打开串口失败”或参数不刷新？
### Q4：上位机点击连接串口提示“打开串口失败”或参数不刷新？
* **排查方法**：
  1. 确认已关闭电脑上的其他串口调试助手（如 XCOM、SSCOM），避免端口冲突被独占占用；
  2. 确认波特率必须设置为 **`115200`**；
  3. 确认在 Vivado Block Design 中已为 `processing_system7_0` 勾选使能 **UART 0 (MIO 14-15)**，因为硬件原理图上 CH340 芯片直连的是 MIO 14 与 MIO 15。

### Q5：微动放大画面出现细小的散粒雪花噪点？
* **排查方法**：
  1. 适当向右拖动上位机底部的 **【抗噪死区门限 Gate】** 滑块（推荐设为 20~35），通过硬件非线性死区将 CMOS 暗电流噪点切除；
  2. 保持拍摄环境光照充足，避免在暗光环境下测试（暗光会导致感光元件模拟增益过大产生散粒噪点）。

### Q6：启动上位机后选择索引 1，但画面只显示彩条（Color Bars），OV5640 摄像头未开启？
* **故障根源分析**：
  1. **彩条并非算法输出**：上位机中的彩条是 **USB-HDMI 视频采集卡在检测到 HDMI 输入“无信号 (No Signal)”时自动生成的硬件测试图**；
  2. **双目摄像头引脚必须对应 J3 插座**：ATK-OV5640-DUAL 插在底板 J3 插座上，其左目（CMOS1）的 SCL 在 `H18`、SDA 在 `M17`、RESET 在 `T15`、PCLK 在 `P16`。若 XDC 错绑到 P2 单目引脚，感光芯片收不到 I2C 指令，复位未释放，无法产生视频流，采集卡便自动回退显示彩条。
* **排查与解决步骤**：
  1. **检查 XDC 管脚约束**：确认已按 J3 双目真实引脚绑定 SCL(`H18`), SDA(`M17`), PCLK(`P16`), VSYNC(`J20`), HREF(`M18`), RESET(`T15`), PWDN(`T17`) 及数据总线 D[7:0]；
  2. **检查 Block Design**：确认 `processing_system7_0` 已使能 **`UART 0 (MIO 14..15)`** 与 **`I2C 0 (EMIO)`**；
  3. **重新生成比特流与导出 XSA**：在 Vivado 中点击 **Generate Bitstream**，完成后 **Export Hardware (包含 bitstream，导出 `system_wrapper2.xsa`)**；
  4. **在 Vitis 中更新平台并运行**：右键平台点击 **Update Hardware Specification**，重新运行 `evm_ps_control` 即可！

### Q7：在 Vitis 中烧录程序后，Serial Monitor 完全没有任何输出（死寂无声），摄像头无法激活？
* **故障根源深度定位（经查阅硬件高清原理图彻底查清）**：
  1. **UART 控制器编号选错（核心原因）**：底板原理图显示 CH340C 芯片直接焊死连接至核心板的 **`PS_MIO14` (RX)** 和 **`PS_MIO15` (TX)**。在 ZYNQ 架构中，MIO 14/15 对应的是 **`UART 0`**，而 Vivado 之前配置的是 **`UART 1`（映射到 MIO 48/49 eMMC 闪存引脚）**，导致所有串口打印全部打进了 eMMC 时钟线！
  2. **J3 插座双目模组引脚错位**：约束文件中之前绑定的是 P2 单目插座引脚，双目模组插在 J3 时，真正的 I2C 线路（SCL: H18, SDA: M17）完全悬空，复位信号（RESET: T15）未被送达，传感器处于未初始化状态。
* **极速解决步骤**：
  1. **已自动更新 Vivado Block Design**：将 PS7 的串口从 UART 1 切换为 **`UART 0 (MIO 14..15)`**；
  2. **已自动更新 `pin_constraints.xdc`**：将摄像头引脚完全映射至 J3 双目 CMOS1 真实引脚；
  3. **已自动同步 C 语言驱动代码**：将 `main_ps_control.c` 中的默认串口基地址适配至 `0xE0000000` (UART0)；
  4. **用户执行下板步骤**：在 Vivado 中点击 **Generate Bitstream** $\rightarrow$ **Export Hardware (生成 `system_wrapper2.xsa`)** $\rightarrow$ 在 Vitis 中更新平台并点击 Run，Serial Monitor 立即打印，彩条立刻转为清晰视频！
