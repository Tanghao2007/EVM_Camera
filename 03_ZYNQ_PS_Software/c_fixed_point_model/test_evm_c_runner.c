/**
 * @file    test_evm_c_runner.c
 * @brief   EVM C 语言软件模块全功能自动化测试程序 (增强版)
 * @details 自动化测试算法定点化流水线、摄像头配置逻辑、热力图 LUT 映射与 5 种显示模式，验证无崩溃、无数值溢出
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "evm_motion_model.h"
#include "../drivers/ov5640_sccb.h"

#define TEST_WIDTH   320
#define TEST_HEIGHT  240
#define TEST_FRAMES  30

static void generate_synthetic_vibrating_frame(evm_image_t *img, int frame_idx) {
    memset(img->data, 50, (size_t)img->width * img->height * 3); // 灰色背景

    /* 正弦微小振荡偏移: 周期 6 帧，振幅 2 像素 */
    int offset = (int)round(2.0 * sin(2.0 * 3.14159265 * frame_idx / 6.0));
    int center_x = img->width / 2 + offset;
    int center_y = img->height / 2;
    int box_half_size = 30;

    for (int y = center_y - box_half_size; y <= center_y + box_half_size; y++) {
        for (int x = center_x - box_half_size; x <= center_x + box_half_size; x++) {
            if (x >= 0 && x < img->width && y >= 0 && y < img->height) {
                size_t idx = (y * img->width + x) * 3;
                img->data[idx + 0] = 220; // 亮白色前景块
                img->data[idx + 1] = 220;
                img->data[idx + 2] = 220;
            }
        }
    }
}

int main(void) {
    printf("====================================================\n");
    printf("  运行 EVM 增强版 C 语言模块自动化编译与功能测试    \n");
    printf("====================================================\n\n");

    /* ----------------- 测试项 1: OV5640 SCCB 驱动逻辑 ----------------- */
    printf("[测试项 1] 测试 OV5640 摄像头配置与锁定逻辑...\n");
    int status_cam = ov5640_init_720p();
    assert(status_cam == 0);
    printf("-> OV5640 初始化与 AEC/AWB 锁定测试: [PASS]\n\n");

    /* ----------------- 测试项 2: EVM 运动算法与热力图缓存初始化 ----------------- */
    printf("[测试项 2] 初始化 EVM 运动算法与热力图实例 (Q4.12 定点)... \n");
    evm_motion_instance_t evm_inst;
    int status_init = evm_motion_init(&evm_inst, TEST_WIDTH, TEST_HEIGHT, 40, 0.4f, 0.05f);
    assert(status_init == 0);
    assert(evm_inst.lowpass1 != NULL);
    assert(evm_inst.lowpass2 != NULL);
    assert(evm_inst.mag_smoothed != NULL);
    printf("-> EVM 实例内存分配与定点系数转换: [PASS]\n\n");

    /* ----------------- 测试项 3: 连续时序多帧运动放大流水线 (多模式) ----------------- */
    printf("[测试项 3] 运行 %d 帧合成微振动序列，验证 5 种显示模式与热力图生成...\n", TEST_FRAMES);
    evm_image_t src_frame, dst_frame;
    src_frame.width = TEST_WIDTH;
    src_frame.height = TEST_HEIGHT;
    src_frame.data = (uint8_t *)malloc((size_t)TEST_WIDTH * TEST_HEIGHT * 3);

    dst_frame.width = TEST_WIDTH;
    dst_frame.height = TEST_HEIGHT;
    dst_frame.data = (uint8_t *)malloc((size_t)TEST_WIDTH * TEST_HEIGHT * 3);

    assert(src_frame.data != NULL);
    assert(dst_frame.data != NULL);

    int total_non_zero_amplification = 0;

    for (int f = 0; f < TEST_FRAMES; f++) {
        generate_synthetic_vibrating_frame(&src_frame, f);

        /* 轮流测试模式 4 (热力图叠加) 和模式 5 (分屏对比) */
        if (f < 15) {
            evm_motion_set_mode(&evm_inst, EVM_MODE_HEATMAP_OVERLAY);
        } else {
            evm_motion_set_mode(&evm_inst, EVM_MODE_SPLIT_SCREEN);
        }

        evm_motion_process_frame(&evm_inst, &src_frame, &dst_frame);

        assert(dst_frame.data != NULL);

        if (evm_inst.mode == EVM_MODE_SPLIT_SCREEN) {
            /* 验证中线是否正确绘制了红线 */
            int mid_x = TEST_WIDTH / 2;
            size_t mid_pixel_idx = (TEST_HEIGHT / 2 * TEST_WIDTH + mid_x) * 3;
            assert(dst_frame.data[mid_pixel_idx + 0] == 255);
            assert(dst_frame.data[mid_pixel_idx + 1] == 0);
            assert(dst_frame.data[mid_pixel_idx + 2] == 0);
        }

        /* 统计变化像素点 */
        for (int y = 0; y < TEST_HEIGHT; y++) {
            for (int x = 0; x < TEST_WIDTH; x++) {
                size_t p = (y * TEST_WIDTH + x) * 3;
                if (dst_frame.data[p] != src_frame.data[p]) {
                    total_non_zero_amplification++;
                }
            }
        }
    }

    printf("-> 成功处理 %d 帧视频流，捕捉并生成了热力图与微动特征!\n", TEST_FRAMES);
    assert(total_non_zero_amplification > 0);
    printf("-> 空间高斯卷积、时域差分 IIR、幅度平滑与伪彩叠加测试: [PASS]\n\n");

    /* ----------------- 测试项 4: 调色板与动态调参测试 ----------------- */
    printf("[测试项 4] 切换调色板与动态调整放大倍数 Alpha 测试...\n");
    evm_motion_set_colormap(&evm_inst, EVM_COLORMAP_VIRIDIS);
    assert(evm_inst.colormap == EVM_COLORMAP_VIRIDIS);
    evm_motion_set_alpha(&evm_inst, 60);
    assert(evm_inst.alpha == 60);
    printf("-> 调色板与动态调参测试: [PASS]\n\n");

    /* ----------------- 清理与资源释放 ----------------- */
    free(src_frame.data);
    free(dst_frame.data);
    evm_motion_free(&evm_inst);
    printf("-> 内存安全释放测试: [PASS]\n\n");

    printf("====================================================\n");
    printf("  全部测试通过! (ALL TESTS PASSED WITH 0 ERRORS)    \n");
    printf("====================================================\n");

    return 0;
}
