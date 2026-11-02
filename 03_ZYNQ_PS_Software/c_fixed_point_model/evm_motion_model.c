/**
 * @file    evm_motion_model.c
 * @brief   欧拉视频放大 (EVM) 运动模式与变化热力图定点化算法 C 语言实现
 * @details 采用 Q4.12 定点格式，实现空间 5x5 高斯差分、时域双低通 IIR、振动能量场平滑与 Jet/Viridis 热力图叠加
 */

#include "evm_motion_model.h"
#include <stdlib.h>
#include <string.h>

/* ============================ 内部辅助常量与 LUT ============================ */

static const int16_t GAUSS_1D_KERNEL[5] = {1, 4, 6, 4, 1};

static inline uint8_t clamp_u8(int32_t val) {
    if (val > 255) return 255;
    if (val < 0)   return 0;
    return (uint8_t)val;
}

/**
 * 256 阶伪彩色查找表: Jet (RGB 格式)
 * 对应硬件 PL 端的 ROM 查表
 */
static void get_jet_color(uint8_t val, uint8_t *r, uint8_t *g, uint8_t *b) {
    float v = (float)val / 255.0f;
    float red = 0.0f, green = 0.0f, blue = 0.0f;

    if (v < 0.125f) {
        red = 0.0f; green = 0.0f; blue = 0.5f + (v / 0.125f) * 0.5f;
    } else if (v < 0.375f) {
        red = 0.0f; green = (v - 0.125f) / 0.25f; blue = 1.0f;
    } else if (v < 0.625f) {
        red = (v - 0.375f) / 0.25f; green = 1.0f; blue = 1.0f - (v - 0.375f) / 0.25f;
    } else if (v < 0.875f) {
        red = 1.0f; green = 1.0f - (v - 0.625f) / 0.25f; blue = 0.0f;
    } else {
        red = 1.0f - (v - 0.875f) / 0.25f * 0.5f; green = 0.0f; blue = 0.0f;
    }

    *r = (uint8_t)(red * 255.0f);
    *g = (uint8_t)(green * 255.0f);
    *b = (uint8_t)(blue * 255.0f);
}

/**
 * 256 阶伪彩色查找表: Viridis (RGB 格式)
 */
static void get_viridis_color(uint8_t val, uint8_t *r, uint8_t *g, uint8_t *b) {
    /* 采用分段线性近似 Viridis 特征色彩 */
    uint32_t v = val;
    if (v < 64) {
        *r = (uint8_t)(68 - (v * 30 / 64));
        *g = (uint8_t)(1 + (v * 83 / 64));
        *b = (uint8_t)(84 + (v * 50 / 64));
    } else if (v < 128) {
        uint32_t sub = v - 64;
        *r = (uint8_t)(38 - (sub * 7 / 64));
        *g = (uint8_t)(84 + (sub * 50 / 64));
        *b = (uint8_t)(134 + (sub * 9 / 64));
    } else if (v < 192) {
        uint32_t sub = v - 128;
        *r = (uint8_t)(31 + (sub * 80 / 64));
        *g = (uint8_t)(134 + (sub * 44 / 64));
        *b = (uint8_t)(143 - (sub * 65 / 64));
    } else {
        uint32_t sub = v - 192;
        *r = (uint8_t)(111 + (sub * 142 / 63));
        *g = (uint8_t)(178 + (sub * 43 / 63));
        *b = (uint8_t)(78 - (sub * 40 / 63));
    }
}

static void spatial_gaussian_blur_5x5(const uint8_t *src_y, uint8_t *dst_y, 
                                      uint16_t width, uint16_t height) {
    int16_t *temp_h = (int16_t *)malloc(width * height * sizeof(int16_t));
    if (!temp_h) return;

    for (int y = 0; y < height; y++) {
        int row_offset = y * width;
        for (int x = 0; x < width; x++) {
            int32_t sum = 0;
            for (int k = -2; k <= 2; k++) {
                int px = x + k;
                if (px < 0) px = 0;
                if (px >= width) px = width - 1;
                sum += src_y[row_offset + px] * GAUSS_1D_KERNEL[k + 2];
            }
            temp_h[row_offset + x] = (int16_t)(sum >> 4);
        }
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int32_t sum = 0;
            for (int k = -2; k <= 2; k++) {
                int py = y + k;
                if (py < 0) py = 0;
                if (py >= height) py = height - 1;
                sum += temp_h[py * width + x] * GAUSS_1D_KERNEL[k + 2];
            }
            dst_y[y * width + x] = (uint8_t)(sum >> 4);
        }
    }

    free(temp_h);
}

/* ============================ 核心对外 API 实现 ============================ */

int32_t evm_motion_init(evm_motion_instance_t *inst, uint16_t width, uint16_t height, 
                        uint8_t alpha, float r1_float, float r2_float) {
    if (!inst || width == 0 || height == 0) return -1;

    inst->width = width;
    inst->height = height;
    inst->alpha = alpha;
    inst->mode = EVM_MODE_HEATMAP_OVERLAY; /* 默认开启视觉效果最佳的热力图叠加模式 */
    inst->colormap = EVM_COLORMAP_JET;

    inst->r1_fixed = (int16_t)(r1_float * EVM_FIXED_ONE + 0.5f);
    inst->r2_fixed = (int16_t)(r2_float * EVM_FIXED_ONE + 0.5f);
    inst->smooth_r_fixed = (int16_t)(0.25f * EVM_FIXED_ONE + 0.5f); // 时间平滑系数 0.25
    inst->noise_gate_fixed = 35; // 默认抗噪死区门限 (~0.008 灰度级)

    size_t pixel_count = (size_t)width * height;

    inst->lowpass1 = (int32_t *)malloc(pixel_count * sizeof(int32_t));
    inst->lowpass2 = (int32_t *)malloc(pixel_count * sizeof(int32_t));
    inst->mag_smoothed = (int32_t *)malloc(pixel_count * sizeof(int32_t));

    if (!inst->lowpass1 || !inst->lowpass2 || !inst->mag_smoothed) {
        evm_motion_free(inst);
        return -1;
    }

    memset(inst->lowpass1, 0, pixel_count * sizeof(int32_t));
    memset(inst->lowpass2, 0, pixel_count * sizeof(int32_t));
    memset(inst->mag_smoothed, 0, pixel_count * sizeof(int32_t));

    return 0;
}

void evm_motion_free(evm_motion_instance_t *inst) {
    if (!inst) return;
    if (inst->lowpass1) { free(inst->lowpass1); inst->lowpass1 = NULL; }
    if (inst->lowpass2) { free(inst->lowpass2); inst->lowpass2 = NULL; }
    if (inst->mag_smoothed) { free(inst->mag_smoothed); inst->mag_smoothed = NULL; }
}

void evm_motion_set_alpha(evm_motion_instance_t *inst, uint8_t new_alpha) {
    if (inst) inst->alpha = new_alpha;
}

void evm_motion_set_mode(evm_motion_instance_t *inst, uint8_t mode) {
    if (inst && mode >= 1 && mode <= 5) inst->mode = mode;
}

void evm_motion_set_colormap(evm_motion_instance_t *inst, uint8_t colormap) {
    if (inst) inst->colormap = colormap;
}

void evm_motion_set_noise_gate(evm_motion_instance_t *inst, uint16_t gate) {
    if (inst) inst->noise_gate_fixed = (int16_t)gate;
}

void evm_motion_process_frame(evm_motion_instance_t *inst, const evm_image_t *src_frame, 
                              evm_image_t *dst_frame) {
    if (!inst || !src_frame || !dst_frame) return;

    uint16_t w = inst->width;
    uint16_t h = inst->height;
    size_t total_pixels = (size_t)w * h;

    uint8_t *Y_orig = (uint8_t *)malloc(total_pixels);
    uint8_t *Y_blur = (uint8_t *)malloc(total_pixels);
    if (!Y_orig || !Y_blur) {
        if (Y_orig) free(Y_orig);
        if (Y_blur) free(Y_blur);
        return;
    }

    /* 1. RGB888 转灰度亮度 Y */
    for (size_t i = 0; i < total_pixels; i++) {
        uint8_t r = src_frame->data[i * 3 + 0];
        uint8_t g = src_frame->data[i * 3 + 1];
        uint8_t b = src_frame->data[i * 3 + 2];
        Y_orig[i] = (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
    }

    /* 2. 空间滤波: 5x5 可分离高斯模糊获得低频基准 */
    spatial_gaussian_blur_5x5(Y_orig, Y_blur, w, h);

    int16_t one_minus_r1 = EVM_FIXED_ONE - inst->r1_fixed;
    int16_t one_minus_r2 = EVM_FIXED_ONE - inst->r2_fixed;
    int16_t one_minus_smooth = EVM_FIXED_ONE - inst->smooth_r_fixed;
    uint16_t mid_x = w / 2;

    /* 3. 逐像素流水线处理 */
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            size_t idx = y * w + x;

            /* 空间残差 L0 (Q4.12) */
            int32_t L0 = ((int32_t)Y_orig[idx] - (int32_t)Y_blur[idx]) << EVM_FIXED_SHIFT;

            /* 时域双 IIR 更新 */
            inst->lowpass1[idx] = ((one_minus_r1 * inst->lowpass1[idx]) + (inst->r1_fixed * L0)) >> EVM_FIXED_SHIFT;
            inst->lowpass2[idx] = ((one_minus_r2 * inst->lowpass2[idx]) + (inst->r2_fixed * L0)) >> EVM_FIXED_SHIFT;
            int32_t delta_y = inst->lowpass1[idx] - inst->lowpass2[idx];

            /* 【时域噪声门限软阈值 (Temporal Noise Coring)】: 彻底滤除微小传感器噪点 */
            int32_t abs_d = (delta_y >= 0) ? delta_y : -delta_y;
            int32_t delta_clean = 0;
            if (abs_d > inst->noise_gate_fixed) {
                delta_clean = (delta_y > 0) ? (abs_d - inst->noise_gate_fixed) : -(abs_d - inst->noise_gate_fixed);
            }

            /* 计算画面放大像素 (仅针对去噪后的有效信号) */
            int32_t amplified_val = (inst->alpha * delta_clean) >> EVM_FIXED_SHIFT;
            int32_t Y_amplified = (int32_t)Y_orig[idx] + amplified_val;
            uint8_t Y_out = clamp_u8(Y_amplified);
            int16_t delta_bright = (int16_t)Y_out - (int16_t)Y_orig[idx];

            uint8_t r_orig = src_frame->data[idx * 3 + 0];
            uint8_t g_orig = src_frame->data[idx * 3 + 1];
            uint8_t b_orig = src_frame->data[idx * 3 + 2];

            uint8_t r_amp = clamp_u8((int32_t)r_orig + delta_bright);
            uint8_t g_amp = clamp_u8((int32_t)g_orig + delta_bright);
            uint8_t b_amp = clamp_u8((int32_t)b_orig + delta_bright);

            /* 计算变化幅度并做时间平滑 (提取纯净微动能量场) */
            int32_t mag_raw = (delta_clean >= 0) ? delta_clean : -delta_clean;
            inst->mag_smoothed[idx] = ((one_minus_smooth * inst->mag_smoothed[idx]) + 
                                       (inst->smooth_r_fixed * mag_raw)) >> EVM_FIXED_SHIFT;

            /* 能量场归一化至 0~255 热力图强度 */
            int32_t mag_intensity = (inst->mag_smoothed[idx] * inst->alpha) >> (EVM_FIXED_SHIFT - 3);
            uint8_t mag_u8 = clamp_u8(mag_intensity);

            /* 查表获得伪彩色热力图 RGB */
            uint8_t r_heat = 0, g_heat = 0, b_heat = 0;
            if (inst->colormap == EVM_COLORMAP_JET) {
                get_jet_color(mag_u8, &r_heat, &g_heat, &b_heat);
            } else {
                get_viridis_color(mag_u8, &r_heat, &g_heat, &b_heat);
            }

            /* Alpha 混合半透明叠加: 当微动越大，热力图权重越大 */
            int32_t blend_alpha = (mag_u8 * 180) / 255; /* 最大权重约 70% */
            uint8_t r_overlay = clamp_u8(((256 - blend_alpha) * r_orig + blend_alpha * r_heat) >> 8);
            uint8_t g_overlay = clamp_u8(((256 - blend_alpha) * g_orig + blend_alpha * g_heat) >> 8);
            uint8_t b_overlay = clamp_u8(((256 - blend_alpha) * b_orig + blend_alpha * b_heat) >> 8);

            /* 4. 根据显示模式输出对应通道 */
            uint8_t final_r = 0, final_g = 0, final_b = 0;

            switch (inst->mode) {
                case EVM_MODE_BYPASS:
                    final_r = r_orig; final_g = g_orig; final_b = b_orig;
                    break;
                case EVM_MODE_AMPLIFY:
                    final_r = r_amp; final_g = g_amp; final_b = b_amp;
                    break;
                case EVM_MODE_HEATMAP_ONLY:
                    final_r = r_heat; final_g = g_heat; final_b = b_heat;
                    break;
                case EVM_MODE_HEATMAP_OVERLAY:
                    final_r = r_overlay; final_g = g_overlay; final_b = b_overlay;
                    break;
                case EVM_MODE_SPLIT_SCREEN:
                    if (x < mid_x) {
                        final_r = r_orig; final_g = g_orig; final_b = b_orig;
                    } else if (x == mid_x) {
                        final_r = 255; final_g = 0; final_b = 0; /* 红色分割线 */
                    } else {
                        final_r = r_overlay; final_g = g_overlay; final_b = b_overlay;
                    }
                    break;
                default:
                    final_r = r_overlay; final_g = g_overlay; final_b = b_overlay;
                    break;
            }

            dst_frame->data[idx * 3 + 0] = final_r;
            dst_frame->data[idx * 3 + 1] = final_g;
            dst_frame->data[idx * 3 + 2] = final_b;
        }
    }

    free(Y_orig);
    free(Y_blur);
}
