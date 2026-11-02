/**
 * @file    evm_motion_model.h
 * @brief   欧拉视频放大 (EVM) 运动模式与变化热力图定点化算法 C 语言头文件
 * @details 融合多尺度降采样思路、变化幅度场提取、Jet/Viridis 伪彩热力图与 5 种显示模式
 */

#ifndef EVM_MOTION_MODEL_H_
#define EVM_MOTION_MODEL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================ 定点化常量与宏定义 ============================ */
#define EVM_FIXED_SHIFT     12                          /**< Q4.12 定点数小数位宽: 12 位 */
#define EVM_FIXED_ONE       (1 << EVM_FIXED_SHIFT)      /**< 定点数数值 1.0 (4096) */
#define EVM_MAX_WIDTH       1280                        /**< 最大支持图像宽度 */
#define EVM_MAX_HEIGHT      720                         /**< 最大支持图像高度 */

/**
 * @brief 5 种核心显示模式定义
 */
typedef enum {
    EVM_MODE_BYPASS          = 1,   /**< 1: 原始视频直通 */
    EVM_MODE_AMPLIFY         = 2,   /**< 2: 经典 EVM 运动放大画面 */
    EVM_MODE_HEATMAP_ONLY    = 3,   /**< 3: 纯微动变化幅度热力图 */
    EVM_MODE_HEATMAP_OVERLAY = 4,   /**< 4: 热力图与原图半透明 Alpha 叠加 (推荐默认) */
    EVM_MODE_SPLIT_SCREEN    = 5    /**< 5: 左右半屏对比显示 (左原图，右放大/热力图) */
} evm_display_mode_t;

/**
 * @brief 热力图调色板类型
 */
typedef enum {
    EVM_COLORMAP_JET     = 0,       /**< Jet 彩虹色谱 (蓝->青->黄->红) */
    EVM_COLORMAP_VIRIDIS = 1        /**< Viridis 色谱 (紫->蓝->绿->黄) */
} evm_colormap_t;

/**
 * @brief 图像数据结构体 (RGB888 格式)
 */
typedef struct {
    uint16_t width;             /**< 图像宽度 (像素) */
    uint16_t height;            /**< 图像高度 (像素) */
    uint8_t *data;              /**< 图像数据指针，排列为 R, G, B, R, G, B... */
} evm_image_t;

/**
 * @brief EVM 运动与热力图状态句柄与历史帧缓存结构体
 */
typedef struct {
    uint16_t width;             /**< 图像宽度 */
    uint16_t height;            /**< 图像高度 */
    uint8_t  alpha;             /**< 放大倍数 (推荐 10 ~ 80) */
    uint8_t  mode;              /**< 当前显示模式 (evm_display_mode_t) */
    uint8_t  colormap;          /**< 当前热力图调色板 (evm_colormap_t) */
    int16_t  r1_fixed;          /**< 高通截止系数 r1 (Q4.12 定点格式) */
    int16_t  r2_fixed;          /**< 低通截止系数 r2 (Q4.12 定点格式) */
    int16_t  smooth_r_fixed;    /**< 热力图时间平滑系数 (Q4.12 定点格式) */
    int16_t  noise_gate_fixed;  /**< 时域微动噪声门限 (Q4.12 定点格式，滤除传感器散粒噪声) */
    int32_t *lowpass1;          /**< 历史状态缓存 1: 上一帧的一阶低通结果 */
    int32_t *lowpass2;          /**< 历史状态缓存 2: 上一帧的一阶低通结果 */
    int32_t *mag_smoothed;      /**< 历史状态缓存 3: 微振动能量幅度场时间平滑缓存 (防闪烁) */
} evm_motion_instance_t;

/* ============================ 核心函数接口声明 ============================ */

/**
 * @brief      初始化 EVM 运动与热力图实例
 * @param[out] inst          待初始化的 EVM 实例指针
 * @param[in]  width         视频流宽度 (像素)
 * @param[in]  height        视频流高度 (像素)
 * @param[in]  alpha         初始放大倍数
 * @param[in]  r1_float      高通截止频率滤波因子 (浮点，如 0.4f)
 * @param[in]  r2_float      低通截止频率滤波因子 (浮点，如 0.05f)
 * @return     int32_t       0 表示成功，负数表示内存分配失败
 */
int32_t evm_motion_init(evm_motion_instance_t *inst, uint16_t width, uint16_t height, 
                        uint8_t alpha, float r1_float, float r2_float);

/**
 * @brief      释放 EVM 实例占用的堆内存
 * @param[in]  inst          待释放的 EVM 实例指针
 */
void evm_motion_free(evm_motion_instance_t *inst);

/**
 * @brief      执行单帧 EVM 运动模式与热力图处理流水线
 * @param[in]  inst          EVM 算法实例指针
 * @param[in]  src_frame     输入原始图像 (RGB888)
 * @param[out] dst_frame     输出处理图像 (RGB888)
 */
void evm_motion_process_frame(evm_motion_instance_t *inst, const evm_image_t *src_frame, 
                              evm_image_t *dst_frame);

/**
 * @brief      动态修改放大倍数 Alpha
 * @param[in]  inst          EVM 算法实例指针
 * @param[in]  new_alpha     新的放大倍数
 */
void evm_motion_set_alpha(evm_motion_instance_t *inst, uint8_t new_alpha);

/**
 * @brief      动态切换显示模式
 * @param[in]  inst          EVM 算法实例指针
 * @param[in]  mode          新显示模式 (1: 直通, 2: 放大, 3: 纯热力图, 4: 叠加, 5: 分屏)
 */
void evm_motion_set_mode(evm_motion_instance_t *inst, uint8_t mode);

/**
 * @brief      动态切换热力图调色板
 * @param[in]  inst          EVM 算法实例指针
 * @param[in]  colormap      调色板 (0: Jet, 1: Viridis)
 */
void evm_motion_set_colormap(evm_motion_instance_t *inst, uint8_t colormap);

/**
 * @brief      设置时域微动噪声死区门限 (Noise Coring)
 * @param[in]  inst          EVM 算法实例指针
 * @param[in]  gate          噪声门限数值 (对应 Q4.12 格式)
 */
void evm_motion_set_noise_gate(evm_motion_instance_t *inst, uint16_t gate);

#ifdef __cplusplus
}
#endif

#endif /* EVM_MOTION_MODEL_H_ */
