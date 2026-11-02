/**
 * @file    main_ps_control.c
 * @brief   ZYNQ-7000 (XC7Z020) ARM Cortex-A9 (PS 端) 裸机主控程序 (GUI滑块/按钮与串口全双工双模驱动版)
 * @details 负责 OV5640 摄像头初始化与锁定、配置 EVM 硬件加速与热力图 IP 寄存器；
 *          全面支持 PC 上位机 GUI 按钮/滑块即时指令 (A:<val>, G:<val>, M:<val>, C:<val>) 
 *          与标准串口调试终端单按键即时响应，并周期性上报硬件微动遥测数据包供 PC 绘制波形。
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#if defined(__has_include)
  #if __has_include("ov5640_sccb.h")
    #include "ov5640_sccb.h"
  #elif __has_include("../drivers/ov5640_sccb.h")
    #include "../drivers/ov5640_sccb.h"
  #else
    #include "ov5640_sccb.h"
  #endif
#else
  #include "ov5640_sccb.h"
#endif

#ifdef __arm__
#include "xil_printf.h"
#include "xil_io.h"
#include "xparameters.h"
#define PRINTF xil_printf

/* 自动适配 UART 驱动库 (PS UART / PL UARTLite / 默认 I/O 回退) */
#if defined(__has_include)
  #if __has_include("xuartps_hw.h")
    #define HAVE_XUARTPS 1
    #include "xuartps_hw.h"
  #elif __has_include("xuartps.h")
    #define HAVE_XUARTPS 1
    #include "xuartps.h"
  #elif __has_include("xuartlite_l.h")
    #define HAVE_XUARTLITE 1
    #include "xuartlite_l.h"
  #endif
#endif

#if !defined(STDIN_BASEADDRESS)
  #if defined(XPAR_XUARTPS_0_BASEADDR)
    #define STDIN_BASEADDRESS XPAR_XUARTPS_0_BASEADDR
  #elif defined(XPAR_UART0_BASEADDR)
    #define STDIN_BASEADDRESS XPAR_UART0_BASEADDR
  #elif defined(XPAR_UART1_BASEADDR)
    #define STDIN_BASEADDRESS XPAR_UART1_BASEADDR
  #else
    #define STDIN_BASEADDRESS 0xE0000000
  #endif
#endif

#include <stdarg.h>

static void zynq_hardware_self_init(void) {
#ifdef __arm__
    /* 1. 解锁 SLCR 寄存器 (系统级控制) */
    Xil_Out32(0xF8000008, 0xDF0D);

    /* 2. 强使能所有 AMBA 外设时钟 (APER_CLK_CTRL: 0xF800012C, 开启 I2C0/1、UART0/1、GPIO 等全部外设时钟，彻底杜绝 APB 总线死锁) */
    Xil_Out32(0xF800012C, 0x01FFFFFF);

    /* 3. 强使能 UART 参考时钟 (UART_CLK_CTRL: 0xF8000154, IO PLL / 18 = 100MHz, 使能 CLKACT0) */
    Xil_Out32(0xF8000154, 0x00001201);

    /* 4. 解除 UART 软件复位 (UART_RST_CTRL: 0xF8000228) */
    Xil_Out32(0xF8000228, 0x00000000);

    /* 5. 解除 I2C 软件复位 (I2C_RST_CTRL: 0xF8000224) */
    Xil_Out32(0xF8000224, 0x00000000);

    /* 6. 强制配置 MIO 14 为 UART0 RX (0xF8000738: L2 Mux, 启用弱上拉与三态) */
    Xil_Out32(0xF8000738, 0x000016E1);

    /* 7. 强制配置 MIO 15 为 UART0 TX (0xF800073C: L2 Mux, 8mA 强推挽输出) */
    Xil_Out32(0xF800073C, 0x000016E0);

    /* 8. 使能 PS-PL Level Shifters (允许安全访问 PL 端 AXI-Lite 寄存器 0x40000000) */
    Xil_Out32(0xF8000900, 0x0000000F);

    /* 9. 解除 PL 端 FPGA 全部复位 (FPGA_RST_CTRL: 0xF8000240 = 0, 释放 FCLK_RESET0_N, 启动 PL 视频全流水线) */
    Xil_Out32(0xF8000240, 0x00000000);

    /* 10. 锁定 SLCR 寄存器 */
    Xil_Out32(0xF8000004, 0x767B);

    /* 8. 裸机强初始化 UART0 外设控制器硬件 (0xE0000000, 杜绝因前级未初始化的死锁) */
    // 8.1 软复位 TX 和 RX FIFO
    Xil_Out32(0xE0000000, 0x00000028);
    for (volatile int d = 0; d < 2000; d++);

    // 8.2 强制设置波特率 115200bps (100MHz 时钟基准: CD=124, BDIV=6)
    Xil_Out32(0xE0000018, 124);         // BAUDGEN = 124
    Xil_Out32(0xE0000034, 6);           // BAUDDIV = 6

    // 8.3 配置帧格式: 8 位数据, 无校验 (None), 1 位停止位, 正常模式 (8-N-1)
    Xil_Out32(0xE0000004, 0x00000020);

    // 8.4 强制开启发送器 TXEN (Bit 4) 与接收器 RXEN (Bit 2)
    Xil_Out32(0xE0000000, 0x00000014);
    for (volatile int d = 0; d < 2000; d++);
#endif
}

extern void outbyte(char c);

static inline void uart_send_char(char c) {
#ifdef __arm__
    /* 等待 TX FIFO 非满 (Bit 4 = 0) */
    while ((Xil_In32(0xE000002C) & 0x00000010) != 0);
    /* 写入 UART0 TX FIFO (0xE0000030) */
    Xil_Out32(0xE0000030, (uint32_t)c);
#else
    putchar(c);
#endif
}

static inline void serial_send_byte(uint8_t b) {
    uart_send_char((char)b);
}

static inline int serial_has_rx_data(void) {
#ifdef __arm__
    /* 检查 UART 0 (0xE0000000, 领航者底板 CH340 COM7 物理直通通道) REMPTY 位 (Bit 1) */
    uint32_t sr = Xil_In32(0xE000002C);
    return ((sr & 0x02) == 0); // 0 表示 FIFO 非空，有数据到达
#else
    return 0;
#endif
}

static inline char serial_read_byte(void) {
#ifdef __arm__
    return (char)(Xil_In32(0xE0000030) & 0xFF);
#else
    return 0;
#endif
}

#define PRINTF xil_printf

#else
#define PRINTF printf
#define Xil_Out32(addr, val) printf("[模拟 AXI-Lite 写] Addr: 0x%08X, Val: %u\n", (unsigned int)(addr), (unsigned int)(val))
#define Xil_In32(addr)       (0)
static inline void serial_send_byte(uint8_t b) { (void)b; }
static inline int serial_has_rx_data(void) { return 0; }
static inline char serial_read_byte(void) { return 0; }
#endif

/* ============================ AXI-Lite 寄存器映射基地址 ============================ */
#if defined(XPAR_EVM_PL_TOP_0_BASEADDR)
  #define EVM_CTRL_BASEADDR       XPAR_EVM_PL_TOP_0_BASEADDR      /**< 由 Vivado 导出的基地址 (0x40000000) */
#elif defined(XPAR_EVM_PL_TOP_BASEADDR)
  #define EVM_CTRL_BASEADDR       XPAR_EVM_PL_TOP_BASEADDR
#else
  #define EVM_CTRL_BASEADDR       0x40000000                      /**< 默认回退基地址 0x40000000 */
#endif

/* 控制与配置寄存器偏移量 (可读写) */
#define REG_EVM_AP_CTRL         (EVM_CTRL_BASEADDR + 0x00)  /**< 控制与状态寄存器 (Bit0: AP_START, Bit7: Auto-Restart) */
#define REG_EVM_WIDTH           (EVM_CTRL_BASEADDR + 0x10)  /**< 图像宽度配置寄存器 */
#define REG_EVM_HEIGHT          (EVM_CTRL_BASEADDR + 0x18)  /**< 图像高度配置寄存器 */
#define REG_EVM_ALPHA           (EVM_CTRL_BASEADDR + 0x20)  /**< 放大倍数 Alpha 配置寄存器 (0~150) */
#define REG_EVM_R1              (EVM_CTRL_BASEADDR + 0x28)  /**< 高通截止因子 r1 (Q4.12 定点格式) */
#define REG_EVM_R2              (EVM_CTRL_BASEADDR + 0x30)  /**< 低通截止因子 r2 (Q4.12 定点格式) */
#define REG_EVM_DISP_MODE       (EVM_CTRL_BASEADDR + 0x38)  /**< 显示模式 (1:直通, 2:放大, 3:热力图叠加) */
#define REG_EVM_COLORMAP        (EVM_CTRL_BASEADDR + 0x40)  /**< 调色板选择 (0:Jet, 1:Viridis) */
#define REG_EVM_NOISE_GATE      (EVM_CTRL_BASEADDR + 0x48)  /**< 硬件抗噪死区门限寄存器 (Q12) */

/* 状态与遥测寄存器偏移量 (只读) */
#define REG_EVM_PEAK_MAG        (EVM_CTRL_BASEADDR + 0x50)  /**< [只读] 硬件统计全帧最大微动峰值幅度 (VIBRATION ALARM) */
#define REG_EVM_FRAME_CNT       (EVM_CTRL_BASEADDR + 0x58)  /**< [只读] 硬件流水线实时帧计数器 */

/* 默认配置参数 */
#define DEFAULT_VIDEO_WIDTH     1280
#define DEFAULT_VIDEO_HEIGHT    720
#define DEFAULT_ALPHA           30              /**< 默认微动放大倍数: 30 倍 */
#define DEFAULT_R1_FIXED        1638            /**< r1 = 0.4  * 4096 = 1638 */
#define DEFAULT_R2_FIXED        205             /**< r2 = 0.05 * 4096 = 205  */
#define DEFAULT_NOISE_GATE      35              /**< 默认抗噪死区门限 (Q12) */
#define ALARM_PEAK_THRESHOLD    25              /**< 触发振动报警的硬件峰值门限 */

/* ============================ 核心功能函数 ============================ */

void evm_configure_hardware(uint32_t width, uint32_t height, uint32_t alpha,
                            uint32_t r1, uint32_t r2, uint32_t noise_gate, uint32_t disp_mode, uint32_t colormap) {
    PRINTF("[PS 端配置] 正在向 PL 端 EVM 加速 IP 写入初始寄存器...\r\n");

    Xil_Out32(REG_EVM_WIDTH, width);
    Xil_Out32(REG_EVM_HEIGHT, height);
    Xil_Out32(REG_EVM_ALPHA, alpha);
    Xil_Out32(REG_EVM_R1, r1);
    Xil_Out32(REG_EVM_R2, r2);
    Xil_Out32(REG_EVM_NOISE_GATE, noise_gate);
    Xil_Out32(REG_EVM_DISP_MODE, disp_mode);
    Xil_Out32(REG_EVM_COLORMAP, colormap);

    /* 启动 IP 自动流水线运行模式 (Auto-Restart 0x81) */
    Xil_Out32(REG_EVM_AP_CTRL, 0x81);

    PRINTF("[PS 端配置] EVM IP 配置完成! 分辨率=%ux%u, Alpha=%u, 门限=%u, 模式=%u, 色表=%s\r\n", 
           (unsigned int)width, (unsigned int)height, (unsigned int)alpha, (unsigned int)noise_gate,
           (unsigned int)disp_mode, colormap == 0 ? "Jet" : "Viridis");
}

void print_control_menu(void) {
    PRINTF("\r\n=======================================================\r\n");
    PRINTF("     ZYNQ EVM 增强版运动放大与热力图系统 (PS 主控终端)   \r\n");
    PRINTF("=======================================================\r\n");
    PRINTF("  【单字符快捷键】:\r\n");
    PRINTF("    '0'        : 模式 0 -> 硬件标准彩条发生器 (Color Bar Test)\r\n");
    PRINTF("    '1'        : 模式 1 -> 原始视频直通 (Bypass)\r\n");
    PRINTF("    '2'        : 模式 2 -> 运动微动放大 (Motion Mag)\r\n");
    PRINTF("    '3'        : 模式 3 -> 热力图 Alpha 半透明叠加 (Heatmap Overlay)\r\n");
    PRINTF("    'c' / 'C'  : 切换热力图调色板 (Jet 彩虹色 <-> Viridis 翠绿色)\r\n");
    PRINTF("    '+' / '='  : 增大放大倍数 Alpha (+5)\r\n");
    PRINTF("    '-' / '_'  : 减小放大倍数 Alpha (-5)\r\n");
    PRINTF("    'n' / 'N'  : 步进微调抗噪死区门限 Gate (+15)\r\n");
    PRINTF("    'd' / 'D'  : 打印 PL 硬件流水线与时钟全链路实时诊断状态\r\n");
    PRINTF("    't' / 'T'  : 开启/关闭 OV5640 传感器片上 8 色彩条测试图\r\n");
    PRINTF("    'e' / 'E'  : 开启自动曝光与自动增益 (AEC/AGC ON, 自适应环境光)\r\n");
    PRINTF("    'l' / 'L'  : 锁定固定曝光与最低增益 (AEC/AGC OFF, 抗微动频闪)\r\n");
    PRINTF("    'b' / 'B'  : 载入室内标定防频闪曝光 (Exp=176 lines, Gain=1.0x, 绝无白屏与行跳动)\r\n");
    PRINTF("    'v' / 'V'  : 载入高灵敏度防频闪曝光 (Exp=352 lines, Gain=1.0x)\r\n");
    PRINTF("    '?' / 'h'  : 查询当前系统状态与打印本菜单\r\n");
    PRINTF("  【GUI 滑块与按钮协议 (支持行指令)】:\r\n");
    PRINTF("    A:<val>    : 设置放大倍数 (例如 A:45 或 ALPHA:45)\r\n");
    PRINTF("    G:<val>    : 设置抗噪门限 (例如 G:20 或 GATE:20)\r\n");
    PRINTF("    M:<val>    : 设置显示模式 (例如 M:3  或 MODE:3)\r\n");
    PRINTF("    C:<val>    : 设置调色板   (例如 C:0 为 Jet, C:1 为 Viridis)\r\n");
    PRINTF("    EXP:<val>  : 手动精准设置曝光行数 (例如 EXP:176 或 EXP:352)\r\n");
    PRINTF("    GAIN:<val> : 手动精准设置模拟增益 (例如 GAIN:16 为 1.0x, GAIN:24 为 1.5x)\r\n");
    PRINTF("    R1:<val>   : 设置高通因子 (例如 R1:1638)\r\n");
    PRINTF("    R2:<val>   : 设置低通因子 (例如 R2:205)\r\n");
    PRINTF("=======================================================\r\n\r\n");
}

/**
 * @brief 向 PC 端上位机发送 12 字节二进制遥测数据包 (供绘制微动波形与 FFT 频谱)
 */
void pc_telemetry_send_frame(int16_t disp_amp, uint16_t energy, uint16_t freq_x10, 
                             uint8_t alpha, uint8_t alarm_state) {
    static uint8_t frame_seq = 0;
    uint8_t packet[12];

    packet[0] = 0xAA;                                     /* 帧头 1 */
    packet[1] = 0x55;                                     /* 帧头 2 */
    packet[2] = frame_seq++;                              /* 帧序号递增 */
    packet[3] = (uint8_t)((disp_amp >> 8) & 0xFF);        /* 振动位移高字节 */
    packet[4] = (uint8_t)(disp_amp & 0xFF);               /* 振动位移低字节 */
    packet[5] = (uint8_t)((energy >> 8) & 0xFF);          /* 振动能量高字节 */
    packet[6] = (uint8_t)(energy & 0xFF);                 /* 振动能量低字节 */
    packet[7] = (uint8_t)((freq_x10 >> 8) & 0xFF);        /* 频率高字节 */
    packet[8] = (uint8_t)(freq_x10 & 0xFF);               /* 频率低字节 */
    packet[9] = alpha;                                    /* 当前 Alpha */
    packet[10] = alarm_state;                             /* 报警状态标志 */

    uint8_t checksum = 0;
    for (int i = 0; i < 11; i++) {
        checksum += packet[i];
    }
    packet[11] = checksum;

#ifdef __arm__
    for (int i = 0; i < 12; i++) {
        serial_send_byte(packet[i]);
    }
#else
    (void)packet;
#endif
}

/**
 * @brief 解析来自上位机 GUI 按钮/滑块的指令或标准串口输入
 */
void process_serial_cmd(const char *cmd, uint32_t *p_alpha, uint32_t *p_noise_gate, 
                        uint32_t *p_disp_mode, uint32_t *p_colormap, 
                        uint32_t *p_r1, uint32_t *p_r2) {
    if (!cmd || cmd[0] == '\0') return;

    /* 跳过前导空白字符 */
    while (*cmd && isspace((unsigned char)*cmd)) cmd++;
    if (*cmd == '\0') return;

    /* 1. 单字符极速模式 (按键兼容) */
    if (cmd[1] == '\0' || cmd[1] == '\r' || cmd[1] == '\n') {
        char ch = cmd[0];
        switch (ch) {
            case '0':
            case '1':
            case '2':
            case '3':
                *p_disp_mode = ch - '0';
                Xil_Out32(REG_EVM_DISP_MODE, *p_disp_mode);
                PRINTF("[ACK] MODE=%u\r\n", (unsigned int)*p_disp_mode);
                return;

            case 'd':
            case 'D': {
                uint32_t pclk_cnt  = Xil_In32(EVM_CTRL_BASEADDR + 0x60);
                uint32_t cap_cnt   = Xil_In32(EVM_CTRL_BASEADDR + 0x64);
                uint32_t mux_cnt   = Xil_In32(EVM_CTRL_BASEADDR + 0x68);
                uint32_t status    = Xil_In32(EVM_CTRL_BASEADDR + 0x6C);
                PRINTF("[DIAG] PCLK_CNT=%u, CAP_PIXELS=%u, OUT_PIXELS=%u\r\n", pclk_cnt, cap_cnt, mux_cnt);
                PRINTF("[DIAG] STATUS=0x%08X (VSYNC=%d, HREF=%d, CAP_VAL=%d, DS_VAL=%d, LB_VAL=%d, IIR_VAL=%d, MUX_VAL=%d, BANK=%d, COL=%d)\r\n",
                       status,
                       (status >> 31) & 1,
                       (status >> 30) & 1,
                       (status >> 29) & 1,
                       (status >> 28) & 1,
                       (status >> 27) & 1,
                       (status >> 26) & 1,
                       (status >> 25) & 1,
                       (status >> 15) & 1,
                       status & 0x3FF);
                return;
            }

            case 'c':
            case 'C':
                *p_colormap = !(*p_colormap);
                Xil_Out32(REG_EVM_COLORMAP, *p_colormap);
                PRINTF("[ACK] COLORMAP=%u (%s)\r\n", (unsigned int)*p_colormap, *p_colormap == 0 ? "Jet" : "Viridis");
                return;

            case '+':
            case '=':
                if (*p_alpha <= 145) *p_alpha += 5;
                else *p_alpha = 150;
                Xil_Out32(REG_EVM_ALPHA, *p_alpha);
                PRINTF("[ACK] ALPHA=%u\r\n", (unsigned int)*p_alpha);
                return;

            case '-':
            case '_':
                if (*p_alpha >= 5) *p_alpha -= 5;
                else *p_alpha = 0;
                Xil_Out32(REG_EVM_ALPHA, *p_alpha);
                PRINTF("[ACK] ALPHA=%u\r\n", (unsigned int)*p_alpha);
                return;

            case 'n':
            case 'N':
                *p_noise_gate = (*p_noise_gate >= 80) ? 10 : *p_noise_gate + 15;
                Xil_Out32(REG_EVM_NOISE_GATE, *p_noise_gate);
                PRINTF("[ACK] GATE=%u\r\n", (unsigned int)*p_noise_gate);
                return;

            case 'e':
            case 'E':
                ov5640_enable_aec_agc();
                PRINTF("[ACK] AEC/AGC ENABLED (自动曝光/增益已开启)\r\n");
                return;

            case 'l':
            case 'L':
                ov5640_lock_aec_awb();
                PRINTF("[ACK] AEC/AGC LOCKED (曝光与增益已锁定)\r\n");
                return;

            case 'p':
            case 'P': {
                static uint8_t cur_pol = 0x20;
                cur_pol = (cur_pol == 0x20) ? 0x21 : 0x20;
                sccb_write_reg(0x4740, cur_pol);
                PRINTF("[ACK] 0x4740 POLARITY toggled to 0x%02X\r\n", cur_pol);
                return;
            }

            case 't':
            case 'T': {
                static uint8_t test_pattern_on = 0;
                test_pattern_on = !test_pattern_on;
                sccb_write_reg(0x503D, test_pattern_on ? 0x80 : 0x00);
                PRINTF("[ACK] TEST PATTERN %s (0x503D = 0x%02X)\r\n",
                       test_pattern_on ? "ENABLED (8色标准彩条测试图)" : "DISABLED (正常感光模式)",
                       test_pattern_on ? 0x80 : 0x00);
                return;
            }

            case 'h':
            case 'H':
                print_control_menu();
                return;

            case 'k':
            case 'K':
                sccb_write_reg(0x3503, 0x03);
                sccb_write_reg(0x3500, 0x00);
                sccb_write_reg(0x3501, 0x00);
                sccb_write_reg(0x3502, 0xA0);
                sccb_write_reg(0x350A, 0x00);
                sccb_write_reg(0x350B, 0x10);
                PRINTF("[ACK] DARK EXPOSURE SET (Exp=10 lines, Gain=1.0x)\r\n");
                return;

            case 'b':
            case 'B':
                sccb_write_reg(0x3503, 0x03);
                sccb_write_reg(0x3500, 0x00);
                sccb_write_reg(0x3501, 0x10);
                sccb_write_reg(0x3502, 0xE0);
                sccb_write_reg(0x350A, 0x00);
                sccb_write_reg(0x350B, 0x10);
                sccb_write_reg(0x3A00, 0x20);
                sccb_write_reg(0x3C01, 0xB4);
                sccb_write_reg(0x3A08, 0x01);
                sccb_write_reg(0x3A09, 0x0E);
                PRINTF("[ACK] CALIBRATED INDOOR EXPOSURE SET (Exp=270 lines, Gain=1.0x, 100Hz Anti-Flicker LOCKED)\r\n");
                return;

            case 'v':
            case 'V':
                sccb_write_reg(0x3503, 0x03);
                sccb_write_reg(0x3500, 0x00);
                sccb_write_reg(0x3501, 0x21);
                sccb_write_reg(0x3502, 0xC0);
                sccb_write_reg(0x350A, 0x00);
                sccb_write_reg(0x350B, 0x10);
                sccb_write_reg(0x3A00, 0x20);
                sccb_write_reg(0x3C01, 0xB4);
                sccb_write_reg(0x3A08, 0x01);
                sccb_write_reg(0x3A09, 0x0E);
                PRINTF("[ACK] HIGH-SENSITIVITY INDOOR EXPOSURE SET (Exp=540 lines, Gain=1.0x, 50Hz Anti-Flicker LOCKED)\r\n");
                return;

            case '?':
                PRINTF("[STATUS] MODE=%u, ALPHA=%u, GATE=%u, COLORMAP=%u, R1=%u, R2=%u\r\n",
                       (unsigned int)*p_disp_mode, (unsigned int)*p_alpha, (unsigned int)*p_noise_gate,
                       (unsigned int)*p_colormap, (unsigned int)*p_r1, (unsigned int)*p_r2);
                return;

            default:
                break;
        }
    }

    /* 2. GUI 滑块与按钮完整协议解析 (例如 "A:35", "ALPHA:35", "G:20", "M:3", "C:1") */
    char temp[64];
    strncpy(temp, cmd, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    /* 去除末尾换行 */
    char *end = temp + strlen(temp) - 1;
    while (end >= temp && (*end == '\r' || *end == '\n' || isspace((unsigned char)*end))) {
        *end = '\0';
        end--;
    }

    /* 寻找键值分隔符 ':' 或 '=' */
    char *sep = strchr(temp, ':');
    if (!sep) sep = strchr(temp, '=');

    if (sep) {
        *sep = '\0';
        char *key = temp;
        char *val_str = sep + 1;
        while (*val_str && isspace((unsigned char)*val_str)) val_str++;
        int val = atoi(val_str);

        /* 键名转大写 */
        for (char *p = key; *p; p++) *p = (char)toupper((unsigned char)*p);

        if (strcmp(key, "A") == 0 || strcmp(key, "ALPHA") == 0) {
            if (val >= 0 && val <= 150) {
                *p_alpha = (uint32_t)val;
                Xil_Out32(REG_EVM_ALPHA, *p_alpha);
                PRINTF("[ACK] ALPHA=%u\r\n", (unsigned int)*p_alpha);
            }
        } else if (strcmp(key, "G") == 0 || strcmp(key, "GATE") == 0 || strcmp(key, "NOISE_GATE") == 0) {
            if (val >= 0 && val <= 500) {
                *p_noise_gate = (uint32_t)val;
                Xil_Out32(REG_EVM_NOISE_GATE, *p_noise_gate);
                PRINTF("[ACK] GATE=%u\r\n", (unsigned int)*p_noise_gate);
            }
        } else if (strcmp(key, "M") == 0 || strcmp(key, "MODE") == 0) {
            if (val >= 1 && val <= 3) {
                *p_disp_mode = (uint32_t)val;
                Xil_Out32(REG_EVM_DISP_MODE, *p_disp_mode);
                PRINTF("[ACK] MODE=%u\r\n", (unsigned int)*p_disp_mode);
            }
        } else if (strcmp(key, "C") == 0 || strcmp(key, "CMAP") == 0 || strcmp(key, "COLORMAP") == 0) {
            *p_colormap = (val != 0) ? 1 : 0;
            Xil_Out32(REG_EVM_COLORMAP, *p_colormap);
            PRINTF("[ACK] COLORMAP=%u\r\n", (unsigned int)*p_colormap);
        } else if (strcmp(key, "R1") == 0) {
            if (val > 0 && val < 4096) {
                *p_r1 = (uint32_t)val;
                Xil_Out32(REG_EVM_R1, *p_r1);
                PRINTF("[ACK] R1=%u\r\n", (unsigned int)*p_r1);
            }
        } else if (strcmp(key, "R2") == 0) {
            if (val > 0 && val < 4096) {
                *p_r2 = (uint32_t)val;
                Xil_Out32(REG_EVM_R2, *p_r2);
                PRINTF("[ACK] R2=%u\r\n", (unsigned int)*p_r2);
            }
        } else if (strcmp(key, "AEC") == 0) {
            if (val != 0) {
                ov5640_enable_aec_agc();
                PRINTF("[ACK] AEC/AGC ENABLED\r\n");
            } else {
                ov5640_lock_aec_awb();
                PRINTF("[ACK] AEC/AGC LOCKED\r\n");
            }
        } else if (strcmp(key, "EXP") == 0 || strcmp(key, "EXPOSURE") == 0) {
            if (val > 0 && val <= 2000) {
                uint32_t sub = (uint32_t)(val * 16);
                sccb_write_reg(0x3503, 0x03);
                sccb_write_reg(0x3500, (uint8_t)((sub >> 16) & 0x0F));
                sccb_write_reg(0x3501, (uint8_t)((sub >> 8) & 0xFF));
                sccb_write_reg(0x3502, (uint8_t)(sub & 0xFF));
                PRINTF("[ACK] EXPOSURE SET: %u lines\r\n", (unsigned int)val);
            }
        } else if (strcmp(key, "GAIN") == 0) {
            if (val >= 0 && val <= 0x01FF) {
                sccb_write_reg(0x3503, 0x03);
                sccb_write_reg(0x350A, (uint8_t)((val >> 8) & 0x03));
                sccb_write_reg(0x350B, (uint8_t)(val & 0xFF));
                PRINTF("[ACK] GAIN SET: 0x%04X (%.2fx)\r\n", (unsigned int)val, val / 16.0f);
            }
        } else if (strcmp(key, "R") == 0 || strcmp(key, "READ") == 0) {
            uint16_t r_addr = (uint16_t)strtoul(val_str, NULL, 16);
            uint8_t r_val = 0;
            int ret = sccb_read_reg(r_addr, &r_val);
            PRINTF("[SCCB READ] Reg=0x%04X, Val=0x%02X (ret=%d)\r\n", r_addr, r_val, ret);
        } else if (strcmp(key, "W") == 0 || strcmp(key, "WRITE") == 0) {
            char *sep2 = strchr(val_str, ':');
            if (!sep2) sep2 = strchr(val_str, '=');
            if (sep2) {
                *sep2 = '\0';
                uint16_t w_addr = (uint16_t)strtoul(val_str, NULL, 16);
                uint8_t w_val = (uint8_t)strtoul(sep2 + 1, NULL, 16);
                int ret = sccb_write_reg(w_addr, w_val);
                PRINTF("[SCCB WRITE] Reg=0x%04X, Val=0x%02X (ret=%d)\r\n", w_addr, w_val, ret);
            }
        } else {
            PRINTF("[ERR] Unknown command key: %s\r\n", key);
        }
    } else {
        /* 无分隔符命令, 例如 "STATUS", "INFO", "HELP" */
        for (char *p = temp; *p; p++) *p = (char)toupper((unsigned char)*p);
        if (strcmp(temp, "STATUS") == 0 || strcmp(temp, "INFO") == 0 || strcmp(temp, "QUERY") == 0) {
            PRINTF("[STATUS] MODE=%u, ALPHA=%u, GATE=%u, COLORMAP=%u, R1=%u, R2=%u\r\n",
                   (unsigned int)*p_disp_mode, (unsigned int)*p_alpha, (unsigned int)*p_noise_gate,
                   (unsigned int)*p_colormap, (unsigned int)*p_r1, (unsigned int)*p_r2);
        } else if (strcmp(temp, "HELP") == 0) {
            print_control_menu();
        } else {
            PRINTF("[ERR] Unrecognized string: %s\r\n", temp);
        }
    }
}

/* ============================ 主程序入口 ============================ */

int main(void) {
    uint32_t current_alpha      = DEFAULT_ALPHA;
    uint32_t current_noise_gate = DEFAULT_NOISE_GATE;
    uint32_t current_disp_mode  = 1; // 默认开启原画直通，直接呈现真实场景
    uint32_t current_colormap   = 0; // 默认 Jet 彩虹色
    uint32_t current_r1         = DEFAULT_R1_FIXED;
    uint32_t current_r2         = DEFAULT_R2_FIXED;

    /* 硬件强自检与外设自激活 (强制使能 UART0、配置 MIO 14/15 为 115200bps、打开 PL Level Shifters) */
    zynq_hardware_self_init();

    /* 开机第一行强心跳打印，验证 ARM 正在执行且串口硬件通路畅通 */
    PRINTF("\r\n\r\n=======================================================\r\n");
    PRINTF("  [ZYNQ 启动] ARM Cortex-A9 已进入 main() 函数执行!\r\n");
    PRINTF("  [串口状态] UART 0 (115200bps, MIO 14/15) 物理通信完全畅通!\r\n");
    PRINTF("=======================================================\r\n");

    print_control_menu();

    /* 1. 初始化 OV5640 摄像头 (支持双目模块进行单目抓取) */
    PRINTF("\r\n[步骤 1/3] 正在检测并初始化 OV5640 摄像头 (单目抓取 720P@60fps)...\r\n");
    int cam_ret = ov5640_init_720p();
    if (cam_ret != 0) {
        PRINTF("[警告] 摄像头初始化异常(错误码: %d)，请检查摄像头排插与供电!\r\n", cam_ret);
    } else {
        PRINTF("[成功] OV5640 摄像头初始化顺利完成!\r\n");
    }

    /* 2. 配置并激活 PL 端 EVM 硬件加速全流水线 */
    PRINTF("[步骤 2/3] 正在启动 PL 端硬件加速流水线...\r\n");
    evm_configure_hardware(DEFAULT_VIDEO_WIDTH, DEFAULT_VIDEO_HEIGHT, current_alpha,
                           current_r1, current_r2, current_noise_gate, current_disp_mode, current_colormap);

    /* 3. 进入用户交互控制循环 */
    PRINTF("[步骤 3/3] 进入全双工监听状态: 支持 PC 上位机 GUI 滑块/按钮协议与串口终端按键...\r\n");

#ifdef __arm__
    char rx_buf[64];
    uint8_t rx_idx = 0;
    uint32_t loop_tick = 0;
    uint32_t last_frame_cnt = 0;

    while (1) {
        /* A. 串口指令接收与解析 */
        while (serial_has_rx_data()) {
            char ch = serial_read_byte();

            if (ch == '\r' || ch == '\n') {
                if (rx_idx > 0) {
                    rx_buf[rx_idx] = '\0';
                    process_serial_cmd(rx_buf, &current_alpha, &current_noise_gate, 
                                       &current_disp_mode, &current_colormap, 
                                       &current_r1, &current_r2);
                    rx_idx = 0;
                }
            } else if (rx_idx == 0 && !serial_has_rx_data() && 
                       (ch == '0' || ch == '1' || ch == '2' || ch == '3' || 
                        ch == '+' || ch == '=' || ch == '-' || ch == '_' || 
                        ch == '?' || ch == 'h' || ch == 'H')) {
                /* 单字符即时按键响应 (仅在 FIFO 中无后续字符时零延时触发) */
                rx_buf[0] = ch;
                rx_buf[1] = '\0';
                process_serial_cmd(rx_buf, &current_alpha, &current_noise_gate, 
                                   &current_disp_mode, &current_colormap, 
                                   &current_r1, &current_r2);
            } else {
                /* 累加进入命令缓冲区供滑块协议解析 (如 A:35, G:20) */
                if (rx_idx < sizeof(rx_buf) - 1) {
                    rx_buf[rx_idx++] = ch;
                } else {
                    rx_idx = 0; // 防止溢出
                }
            }
        }

        /* B. 定期读取硬件微动峰值并向上位机上报遥测数据包 (~20Hz) */
        loop_tick++;
        if (loop_tick >= 100000) {
            loop_tick = 0;

            uint32_t curr_frame = Xil_In32(REG_EVM_FRAME_CNT);
            if (curr_frame != last_frame_cnt) {
                last_frame_cnt = curr_frame;

                /* 读取 PL 端统计的实时全帧微动峰值强度 */
                uint32_t peak_raw = Xil_In32(REG_EVM_PEAK_MAG) & 0xFF;
                int16_t disp_amp = (int16_t)(peak_raw * 10); // 放大 10 倍以定点表示
                uint16_t energy  = (uint16_t)(peak_raw * peak_raw);
                uint16_t freq_x10 = 25; // 示例主频 2.5Hz
                uint8_t alarm_state = (peak_raw > ALARM_PEAK_THRESHOLD) ? 1 : 0;

                /* 发送 12 字节二进制帧包给 PC 上位机绘制波形 */
                pc_telemetry_send_frame(disp_amp, energy, freq_x10, (uint8_t)current_alpha, alarm_state);
            }
        }
    }
#else
    PRINTF("[仿真环境] 测试指令解析...\n");
    process_serial_cmd("A:45", &current_alpha, &current_noise_gate, &current_disp_mode, &current_colormap, &current_r1, &current_r2);
    process_serial_cmd("G:20", &current_alpha, &current_noise_gate, &current_disp_mode, &current_colormap, &current_r1, &current_r2);
    process_serial_cmd("M:2", &current_alpha, &current_noise_gate, &current_disp_mode, &current_colormap, &current_r1, &current_r2);
    process_serial_cmd("C:1", &current_alpha, &current_noise_gate, &current_disp_mode, &current_colormap, &current_r1, &current_r2);
    process_serial_cmd("?", &current_alpha, &current_noise_gate, &current_disp_mode, &current_colormap, &current_r1, &current_r2);
    PRINTF("[仿真环境] 主控初始化顺利通过.\n");
#endif

    return 0;
}
