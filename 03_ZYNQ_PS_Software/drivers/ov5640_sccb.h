/**
 * @file    ov5640_sccb.h
 * @brief   ATK-OV5640 摄像头配置与 SCCB 通信驱动头文件 (抗噪优化版)
 * @details 提供标准 SCCB 寄存器读写接口、片上 2D 硬件去噪 ISP 与曝光/增益锁定配置
 */

#ifndef OV5640_SCCB_H_
#define OV5640_SCCB_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 关键寄存器与地址定义 ============================ */
#define OV5640_SCCB_SLAVE_ADDR      0x3C        /**< OV5640 7位 I2C/SCCB 设备从机地址 (8位写地址为 0x78) */

/* 核心图像控制寄存器 */
#define OV5640_REG_CHIP_ID_H        0x300A      /**< 芯片 ID 高字节 (固定读取为 0x56) */
#define OV5640_REG_CHIP_ID_L        0x300B      /**< 芯片 ID 低字节 (固定读取为 0x40) */
#define OV5640_REG_AEC_MANUAL       0x3503      /**< 自动曝光 (AEC) 手动控制开关寄存器 */
#define OV5640_REG_EXPOSURE_H       0x3500      /**< 手动曝光时间高字节 */
#define OV5640_REG_EXPOSURE_M       0x3501      /**< 手动曝光时间中字节 */
#define OV5640_REG_EXPOSURE_L       0x3502      /**< 手动曝光时间低字节 */
#define OV5640_REG_GAIN_H           0x350A      /**< 手动增益高字节 */
#define OV5640_REG_GAIN_L           0x350B      /**< 手动增益低字节 */
#define OV5640_REG_AWB_ENABLE       0x5001      /**< 自动白平衡 (AWB) 开关控制寄存器 */

/* 【专项抗噪硬件寄存器】OV5640 片上 2D 去噪 ISP 引擎 */
#define OV5640_REG_DENOISE_EN       0x5300      /**< 2D 硬件去噪使能控制寄存器 (Bit[0]=1 使能) */
#define OV5640_REG_DENOISE_TH1      0x5302      /**< 去噪阈值高阶控制 */
#define OV5640_REG_DENOISE_TH2      0x5303      /**< 去噪平滑阈值控制 */
#define OV5640_REG_SHARP_ENABLE     0x5308      /**< 锐度边缘控制 (微动放大需抑制人工过冲锐化) */

/* 寄存器配置项键值对结构体 */
typedef struct {
    uint16_t reg;                               /**< 16位寄存器地址 */
    uint8_t  val;                               /**< 8位写入参数值 */
} ov5640_reg_cfg_t;

/* ============================ 核心对外接口 ============================ */

int32_t sccb_write_reg(uint16_t reg_addr, uint8_t reg_val);
int32_t sccb_read_reg(uint16_t reg_addr, uint8_t *reg_val);
int32_t ov5640_init_720p(void);
int32_t ov5640_enable_aec_agc(void);
int32_t ov5640_lock_aec_awb(void);

/**
 * @brief      启用 OV5640 内部硬件 2D 降噪引擎 (针对 EVM 微动放大专项优化)
 * @details    配置片上 2D Denoise ISP，滤除 CMOS 感光阵列产生的暗电流与高频散粒噪声，
 *             从传感器源头消灭被放大数十倍后引发的雪花杂波。
 * @return     int32_t 0 表示配置成功，负数表示失败
 */
int32_t ov5640_enable_hardware_denoise(void);

#ifdef __cplusplus
}
#endif

#endif /* OV5640_SCCB_H_ */
