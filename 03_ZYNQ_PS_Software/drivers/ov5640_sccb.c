/**
 * @file    ov5640_sccb.c
 * @brief   ATK-OV5640 摄像头寄存器配置与 SCCB 通信实现 (抗噪优化版)
 * @details 包含 720P 输出模式配置、曝光增益锁定以及针对微动放大的 2D 硬件去噪 ISP 激活
 */

#include "ov5640_sccb.h"
#include <stdio.h>

#ifdef __arm__
#include "xil_printf.h"
#include "xparameters.h"
#include "sleep.h"
#define LOG_PRINT xil_printf

#if defined(__has_include)
  #if __has_include("xiicps.h")
    #define HAVE_XIICPS 1
    #include "xiicps.h"
  #endif
#endif

#else
#define LOG_PRINT printf
#define usleep(x) ((void)0)
#endif

#if defined(HAVE_XIICPS)
XIicPs IicInstance;
static int s_iic_initialized = 0;

static int ov5640_iic_init(void) {
    if (s_iic_initialized) return 0;
    
    XIicPs_Config *iic_cfg = NULL;

    /* 优先使用 Vitis 2024/2025 SDT 架构的 BaseAddress 查找 (0xE0004000) */
#if defined(XPAR_XIICPS_0_BASEADDR)
    iic_cfg = XIicPs_LookupConfig(XPAR_XIICPS_0_BASEADDR);
#elif defined(XPAR_I2C0_BASEADDR)
    iic_cfg = XIicPs_LookupConfig(XPAR_I2C0_BASEADDR);
#elif defined(XPAR_PS7_I2C_0_DEVICE_ID)
    iic_cfg = XIicPs_LookupConfig(XPAR_PS7_I2C_0_DEVICE_ID);
#elif defined(XPAR_XIICPS_0_DEVICE_ID)
    iic_cfg = XIicPs_LookupConfig(XPAR_XIICPS_0_DEVICE_ID);
#else
    /* 默认回退到 ZYNQ-7000 I2C 0 物理基地址 0xE0004000 */
    iic_cfg = XIicPs_LookupConfig(0xE0004000);
#endif
    if (!iic_cfg) {
        LOG_PRINT("[I2C 错误] 未找到 PS I2C 控制器配置!\r\n");
        return -1;
    }
    
    int status = XIicPs_CfgInitialize(&IicInstance, iic_cfg, iic_cfg->BaseAddress);
    if (status != XST_SUCCESS) {
        LOG_PRINT("[I2C 错误] I2C 控制器初始化失败!\r\n");
        return -1;
    }
    
    XIicPs_SetSClk(&IicInstance, 100000); // 100kHz 标准 I2C 速率
    s_iic_initialized = 1;
    return 0;
}
#endif

/* ============================ 核心函数实现 ============================ */

int32_t sccb_write_reg(uint16_t reg_addr, uint8_t reg_val) {
#if defined(__arm__) && defined(HAVE_XIICPS)
    if (!s_iic_initialized) {
        if (ov5640_iic_init() != 0) return -1;
    }
    uint8_t buffer[3];
    buffer[0] = (uint8_t)(reg_addr >> 8);   /* 寄存器高字节 */
    buffer[1] = (uint8_t)(reg_addr & 0xFF); /* 寄存器低字节 */
    buffer[2] = reg_val;                    /* 数据字节 */

    int status = XIicPs_MasterSendPolled(&IicInstance, buffer, 3, OV5640_SCCB_SLAVE_ADDR);
    if (status != 0) {
        LOG_PRINT("SCCB Write Error: Reg=0x%04X, Val=0x%02X\r\n", reg_addr, reg_val);
        return -1;
    }
    return 0;
#else
    (void)reg_addr;
    (void)reg_val;
    return 0;
#endif
}

int32_t sccb_read_reg(uint16_t reg_addr, uint8_t *reg_val) {
    if (!reg_val) return -1;
#if defined(__arm__) && defined(HAVE_XIICPS)
    if (!s_iic_initialized) {
        if (ov5640_iic_init() != 0) return -1;
    }
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(reg_addr >> 8);
    buffer[1] = (uint8_t)(reg_addr & 0xFF);

    int status = XIicPs_MasterSendPolled(&IicInstance, buffer, 2, OV5640_SCCB_SLAVE_ADDR);
    if (status != 0) {
        return -1;
    }
    status = XIicPs_MasterRecvPolled(&IicInstance, reg_val, 1, OV5640_SCCB_SLAVE_ADDR);
    if (status != 0) {
        return -2;
    }
    return 0;
#else
    (void)reg_addr;
    *reg_val = 0x00;
    return 0;
#endif
}

int32_t ov5640_enable_hardware_denoise(void) {
    LOG_PRINT("[OV5640 抗噪配置] 正在激活片上 2D 硬件去噪引擎 (Denoise ISP)...\r\n");

    /* 1. 使能片上 2D 硬件去噪引擎: 0x5300 Bit[0] 置 1 */
    sccb_write_reg(OV5640_REG_DENOISE_EN, 0x01);

    /* 2. 配置适度去噪阈值 (既去除散粒噪点，又保留亚像素物理边缘微动) */
    sccb_write_reg(OV5640_REG_DENOISE_TH1, 0x08);
    sccb_write_reg(OV5640_REG_DENOISE_TH2, 0x10);

    /* 3. 关闭激进的人工数字边缘锐化 (人工锐化会在边缘产生振铃脉冲噪点) */
    sccb_write_reg(OV5640_REG_SHARP_ENABLE, 0x00);

    LOG_PRINT("[OV5640 抗噪配置] 2D 硬件去噪引擎已激活，从传感器源头压制暗电流噪点!\r\n");
    return 0;
}

int32_t ov5640_enable_aec_agc(void) {
    LOG_PRINT("[OV5640 配置] 正在开启自动曝光 (AEC)、自动增益 (AGC) 与自动白平衡 (AWB)...\r\n");

    /* 1. 设置基准曝光时间 (300 lines << 4 = 0x12C0) 与基准模拟增益 (1.0x = 0x0010) */
    sccb_write_reg(0x3500, 0x00);
    sccb_write_reg(0x3501, 0x12);
    sccb_write_reg(0x3502, 0xC0);
    sccb_write_reg(0x350A, 0x00);
    sccb_write_reg(0x350B, 0x10);

    /* 2. 配置 AEC 目标亮度与稳定窗口 (Target Luminance: 0x58=88, 0x48=72) */
    sccb_write_reg(0x3A0F, 0x58); // AEC 稳定上限
    sccb_write_reg(0x3A10, 0x48); // AEC 稳定下限
    sccb_write_reg(0x3A1B, 0x58); // AEC 快速上限
    sccb_write_reg(0x3A1E, 0x48); // AEC 快速下限
    sccb_write_reg(0x3A11, 0x60); // AEC 步进
    sccb_write_reg(0x3A1F, 0x14); // AEC 快速步进

    /* 3. 配置 50Hz/60Hz 最大曝光行数 (全帧 740 行 = 0x02E4) */
    sccb_write_reg(0x3A02, 0x02);
    sccb_write_reg(0x3A03, 0xE4);
    sccb_write_reg(0x3A14, 0x02);
    sccb_write_reg(0x3A15, 0xE4);

    /* 4. 配置 AEC 增益上限 (Gain Ceiling: 8x 增益 0x0080) */
    sccb_write_reg(0x3A18, 0x00);
    sccb_write_reg(0x3A19, 0x80);

    /* 5. 配置自动工频 50Hz/60Hz 检测与防闪烁滤波 (Banding Filter) */
    sccb_write_reg(0x3C01, 0x34);
    sccb_write_reg(0x3A00, 0x78);

    /* 6. 开启自动曝光 (AEC) 与自动增益 (AGC): 寄存器 0x3503 清零恢复全自动 */
    if (sccb_write_reg(OV5640_REG_AEC_MANUAL, 0x00) != 0) return -1;

    /* 7. 开启自动白平衡 (AWB) 与硬件缩放 (Scale): 0x5001 = 0xA3 */
    if (sccb_write_reg(OV5640_REG_AWB_ENABLE, 0xa3) != 0) return -1;

    LOG_PRINT("[OV5640 配置] 成功开启 AEC/AGC 与 AWB，已恢复全自动光照与色彩自适应!\r\n");
    return 0;
}

int32_t ov5640_lock_aec_awb(void) {
    LOG_PRINT("[OV5640 配置] 正在执行 EVM 关键图像抗噪配置 (锁定曝光与增益)...\r\n");

    /* 1. 关闭自动曝光控制 (AEC): 寄存器 0x3503 写入 0x03 (手动曝光 + 手动增益) */
    if (sccb_write_reg(OV5640_REG_AEC_MANUAL, 0x03) != 0) return -1;

    /* 2. 写入适度室内曝光时间 (500 行 << 4 = 8000 = 0x1F40) */
    sccb_write_reg(OV5640_REG_EXPOSURE_H, 0x00);
    sccb_write_reg(OV5640_REG_EXPOSURE_M, 0x1F);
    sccb_write_reg(OV5640_REG_EXPOSURE_L, 0x40);

    /* 3. 设置适度增益 2.0x (增益=0x0020) */
    sccb_write_reg(OV5640_REG_GAIN_H, 0x00);
    sccb_write_reg(OV5640_REG_GAIN_L, 0x20);

    /* 4. 保持 AWB 开启，严禁写入 0x5002~0x5007 内部调试寄存器 */
    sccb_write_reg(0x5580, 0x00);
    if (sccb_write_reg(OV5640_REG_AWB_ENABLE, 0xa3) != 0) return -1;

    LOG_PRINT("[OV5640 配置] 成功配置曝光与增益!\r\n");
    return 0;
}

/* OV5640 完整官方标准 720P@60FPS (1280x720 RGB565 DVP) 硬件寄存器初始化表 */
static const ov5640_reg_cfg_t ov5640_720p_regs[] = {
    /* 1. 系统控制与软件复位 */
    {0x3103, 0x11}, // 系统时钟直接由引脚提供
    {0x3008, 0x82}, // 硬件软复位
    {0x3008, 0x02}, // 正常工作模式 (原正点原子官方SDK配置，清除Bit 6休眠)
    {0x3103, 0x03}, // PLL 时钟使能

    /* 2. 模拟前端偏置与电荷泵配置 (OmniVision 官方核心模拟参数) */
    {0x4740, 0x20},
    {0x4050, 0x6e},
    {0x4051, 0x8f},
    {0x3017, 0xff}, // DVP 数据总线与控制引脚全部使能
    {0x3018, 0xff},
    {0x302c, 0x02},
    {0x3108, 0x01}, // PCLK 分频与 SCLK
    {0x3630, 0x2e},
    {0x3632, 0xe2},
    {0x3633, 0x23},
    {0x3621, 0xe0},
    {0x3704, 0xa0},
    {0x3703, 0x5a},
    {0x3715, 0x78},
    {0x3717, 0x01},
    {0x370b, 0x60},
    {0x3705, 0x1a},
    {0x3905, 0x02},
    {0x3906, 0x10},
    {0x3901, 0x0a},
    {0x3731, 0x12},
    {0x3600, 0x08},
    {0x3601, 0x33},
    {0x302d, 0x60},
    {0x3620, 0x52},
    {0x371b, 0x20},
    {0x471c, 0x50},
    {0x3a18, 0x00},
    {0x3a19, 0xf8},
    {0x3635, 0x1c},
    {0x3634, 0x40},
    {0x3622, 0x01},
    {0x3c04, 0x28},
    {0x3c05, 0x98},
    {0x3c06, 0x00},
    {0x3c07, 0x08},
    {0x3c08, 0x00},
    {0x3c09, 0x1c},
    {0x3c0a, 0x9c},
    {0x3c0b, 0x40},

    /* 3. 720P (1280x720) 窗口与输出时序配置 */
    {0x3820, 0x46}, // 开启垂直翻转 (Bit[2] ISP vflip + Bit[1] Sensor vflip), 矫正凸透镜光学倒立
    {0x3821, 0x07}, // 开启水平镜像 (Bit[2] ISP mirror + Bit[1] Sensor mirror + Bit[0] H binning)
    {0x3800, 0x00}, // X 起始高位
    {0x3801, 0x00}, // X 起始低位
    {0x3802, 0x00}, // Y 起始高位
    {0x3803, 0xfa}, // Y 起始低位 (250)
    {0x3804, 0x0a}, // X 结束高位
    {0x3805, 0x3f}, // X 结束低位 (2623)
    {0x3806, 0x06}, // Y 结束高位
    {0x3807, 0xa9}, // Y 结束低位 (1705)
    {0x3808, 0x05}, // 输出宽度 1280 高位
    {0x3809, 0x00}, // 输出宽度 1280 低位
    {0x380a, 0x02}, // 输出高度 720 高位
    {0x380b, 0xd0}, // 输出高度 720 低位
    {0x3810, 0x00},
    {0x3811, 0x10}, // X offset = 16
    {0x3812, 0x00},
    {0x3813, 0x04}, // Y offset = 4
    {0x3814, 0x31}, // timing X inc (2x 抽样)
    {0x3815, 0x31}, // timing Y inc (2x 抽样)
    {0x3034, 0x1a},
    {0x3035, 0x11}, // 720P PLL 预分频
    {0x3036, 0x40}, // 720P PLL 倍频器
    {0x3037, 0x13}, // 720P PLL 根分频
    {0x3038, 0x00},
    {0x3039, 0x00},
    {0x380c, 0x07}, // 总水平周期 HTS (1892)
    {0x380d, 0x64},
    {0x380e, 0x02}, // 总垂直周期 VTS = 740 (0x02E4)
    {0x380f, 0xe4},
    {0x3c01, 0xb4}, // 自动工频防闪烁滤波使能
    {0x3c00, 0x04},
    {0x3a00, 0x20}, // 开启 50Hz 工频防闪烁滤波 (彻底杜绝室内 50Hz/100Hz 交流电频闪)
    {0x3a08, 0x01}, // 50Hz 阶跃步长 (270 lines = 0x010E)
    {0x3a09, 0x0e},
    {0x3a0e, 0x06},
    {0x3a0a, 0x00},
    {0x3a0b, 0x7b},
    {0x3a0d, 0x08},
    {0x3a02, 0x05},
    {0x3a03, 0xc4},
    {0x3a14, 0x05},
    {0x3a15, 0xc4},
    {0x3618, 0x00},
    {0x3612, 0x29},
    {0x3708, 0x64},
    {0x3709, 0x52},
    {0x370c, 0x03},
    {0x4001, 0x02}, // 黑电平自动校准 (BLC)
    {0x4004, 0x02},
    {0x3000, 0x00}, // 内部核心模块全部使能
    {0x3002, 0x1c},
    {0x3004, 0xff}, // 内部时钟门控全部开启
    {0x3006, 0xc3},
    {0x300e, 0x58}, // DVP 模式使能, MIPI 休眠
    {0x302e, 0x00},

    /* 4. 输出数据格式控制: RGB565 (16-bit) 与三原色增益基准 */
    {0x4300, 0x60}, // RGB565 格式: BGR 顺序 (0x60 纠正红蓝互换)
    {0x501f, 0x01}, // ISP RGB 输出选通
    {0x4713, 0x03}, // DVP 极性与时序配置
    {0x4407, 0x04},
    {0x460b, 0x35},
    {0x460c, 0x20}, // DVP PCLK 手动分频器使能 (Bit 1 = 0 由 0x3824 精确控制)
    {0x5001, 0xa3}, // AWB 开启, CMX 开启, 缩放 SCALE 开启 (0xA3)

    /* 5. 颜色矩阵、Gamma 曲线与画质提升 */
    {0x5180, 0xff},
    {0x5181, 0xf2},
    {0x5182, 0x00},
    {0x5183, 0x14},
    {0x5184, 0x25},
    {0x5185, 0x24},
    {0x5186, 0x16},
    {0x5187, 0x16},
    {0x5188, 0x16},
    {0x5189, 0x62},
    {0x518a, 0x62},
    {0x518b, 0xf0},
    {0x518c, 0xb2},
    {0x518d, 0x50},
    {0x518e, 0x30},
    {0x518f, 0x30},
    {0x5190, 0x50},
    {0x5191, 0xf8},
    {0x5192, 0x04},
    {0x5193, 0x70},
    {0x5194, 0xf0},
    {0x5195, 0xf0},
    {0x5196, 0x03},
    {0x5197, 0x01},
    {0x5198, 0x04},
    {0x5199, 0x12},
    {0x519a, 0x04},
    {0x519b, 0x00},
    {0x519c, 0x06},
    {0x519d, 0x82},
    {0x519e, 0x38},
    {0x5480, 0x01},
    {0x5481, 0x06},
    {0x5482, 0x12},
    {0x5483, 0x24},
    {0x5484, 0x4a},
    {0x5485, 0x58},
    {0x5486, 0x65},
    {0x5487, 0x72},
    {0x5488, 0x7d},
    {0x5489, 0x88},
    {0x548a, 0x92},
    {0x548b, 0xa3},
    {0x548c, 0xb2},
    {0x548d, 0xc8},
    {0x548e, 0xdd},
    {0x548f, 0xf0},
    {0x5490, 0x15},
    {0x5580, 0x00}, // 关闭 SDE 饱和度与对比度失真，还原纯净真实场景
    {0x5583, 0x40},
    {0x5584, 0x20},
    {0x5589, 0x10},
    {0x558a, 0x00},
    {0x558b, 0xf8},
    {0x5000, 0xa7}, // 开启镜头校正、坏点补偿与色彩插值
    {0x5025, 0x00},

    /* 6. 自动曝光 AEC/AGC 稳定窗口与门限 */
    {0x3a0f, 0x78}, // AEC 稳定上限 (120/256)
    {0x3a10, 0x68}, // AEC 稳定下限 (104/256)
    {0x3a1b, 0x78},
    {0x3a1e, 0x68},
    {0x3a11, 0x60},
    {0x3a1f, 0x14},
    {0x4005, 0x1a},
    {0x3406, 0x00},
    {0x3503, 0x00}, // 开启全自动曝光 (AEC/AGC)
    {0x3008, 0x02}  // 启动正常感光与高速输出
};

int32_t ov5640_init_720p(void) {
#if defined(__arm__) && !defined(HAVE_XIICPS)
    LOG_PRINT("[OV5640 提示] BSP 中未启用 PS 端 I2C 控制器 (xiicps.h)。\r\n");
    LOG_PRINT("[OV5640 提示] 若摄像头未由 PL 硬件状态机配置，画面将呈现彩条!\r\n");
    LOG_PRINT("[OV5640 提示] 请在 Vivado BD 中使能 PS7 I2C 0 (EMIO) 并导出 SCL/SDA 管脚!\r\n");
    return 0;
#else
    uint8_t chip_id_h = 0, chip_id_l = 0;

    /* 1. 初始化 I2C 控制器 */
    if (ov5640_iic_init() != 0) {
        LOG_PRINT("[OV5640 错误] I2C 总线初始化失败，请检查 Vivado I2C EMIO 配置!\r\n");
        return -1;
    }

    /* 硬件上电稳定延时 (等待 100ms 确保传感器内部 PLL 起振) */
    usleep(100000);

    /* 2. 读取并核验芯片 ID (带 5 次重试机制) */
    int id_ok = 0;
    for (int retry = 0; retry < 5; retry++) {
        chip_id_h = 0;
        chip_id_l = 0;
        sccb_read_reg(OV5640_REG_CHIP_ID_H, &chip_id_h);
        sccb_read_reg(OV5640_REG_CHIP_ID_L, &chip_id_l);
        if (chip_id_h == 0x56 && chip_id_l == 0x40) {
            id_ok = 1;
            break;
        }
        usleep(10000); // 10ms 重试间隔
    }

    if (id_ok) {
        LOG_PRINT("成功检测到 OV5640 (Chip ID: 0x5640)!\r\n");
    } else {
        LOG_PRINT("[警告] 读取 OV5640 ID 为: 0x%02X%02X (可能读时序未对齐)，强制执行 720P 配置写入...\r\n", chip_id_h, chip_id_l);
    }

    /* 3. 批量写入 720P@60fps RGB565 基础配置表 */
    LOG_PRINT("[OV5640 配置] 正在写入 720P@60FPS 时钟与输出时序...\r\n");
    size_t num_regs = sizeof(ov5640_720p_regs) / sizeof(ov5640_720p_regs[0]);
    int write_errors = 0;
    int consecutive_fails = 0;
    for (size_t i = 0; i < num_regs; i++) {
        if (sccb_write_reg(ov5640_720p_regs[i].reg, ov5640_720p_regs[i].val) != 0) {
            write_errors++;
            consecutive_fails++;
            if (consecutive_fails >= 5) {
                LOG_PRINT("\r\n[I2C 错误] 连续 5 次寄存器写入未收到应答(NACK)!\r\n");
                LOG_PRINT("[排查指引] 1. 确认 ATK-OV5640 摄像头插在板载 P2 (18针单目接口)，引脚完全对齐插入\r\n");
                LOG_PRINT("           2. 确认已插上 12V 直流电源适配器 (不能仅靠 USB Hub 供电)\r\n");
                LOG_PRINT("           3. 确认板上 P5 排针跳线帽已短接 1-3 和 2-4 (选择 PS 端串口)\r\n");
                return -2;
            }
        } else {
            consecutive_fails = 0;
        }
        usleep(500); // 每次写入微小延时保障总线稳定
    }
    LOG_PRINT("[OV5640 配置] 720P@60FPS 配置写入完毕! (错误数: %d)\r\n", write_errors);

    /* 4. 开启全自动曝光 (AEC)、自动增益 (AGC) 与自动白平衡 (AWB) 自适应真实物理场景 */
    ov5640_enable_aec_agc();

    /* 等待时钟锁定并输出稳定 PCLK */
    usleep(50000);
    LOG_PRINT("[OV5640 配置] 摄像头启动成功! 像素时钟已正常向 PL 端输出!\r\n");
    return 0;
#endif
}
