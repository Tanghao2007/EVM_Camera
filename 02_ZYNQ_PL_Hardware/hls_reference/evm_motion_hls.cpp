#include "evm_motion_hls.h"

// 5x5 二项式高斯滤波近似核 (归一化分母为 256)
static const ap_int<8> GAUSS_5X5[5][5] = {
    {1,  4,  6,  4, 1},
    {4, 16, 24, 16, 4},
    {6, 24, 36, 24, 6},
    {4, 16, 24, 16, 4},
    {1,  4,  6,  4, 1}
};

// 伪彩色 Jet 映射 (硬件分段流水线逼近)
static void get_jet_hardware(ap_uint<8> val, ap_uint<8>& r, ap_uint<8>& g, ap_uint<8>& b) {
    #pragma HLS INLINE
    if (val < 32) {
        r = 0; g = 0; b = 128 + (val << 2);
    } else if (val < 96) {
        r = 0; g = (val - 32) << 2; b = 255;
    } else if (val < 160) {
        r = (val - 96) << 2; g = 255; b = 255 - ((val - 96) << 2);
    } else if (val < 224) {
        r = 255; g = 255 - ((val - 160) << 2); b = 0;
    } else {
        r = 255 - ((val - 224) << 2); g = 0; b = 0;
    }
}

// 伪彩色 Viridis 映射
static void get_viridis_hardware(ap_uint<8> val, ap_uint<8>& r, ap_uint<8>& g, ap_uint<8>& b) {
    #pragma HLS INLINE
    if (val < 64) {
        r = 68 - ((val * 30) >> 6);
        g = 1 + ((val * 83) >> 6);
        b = 84 + ((val * 50) >> 6);
    } else if (val < 128) {
        ap_uint<8> sub = val - 64;
        r = 38 - ((sub * 7) >> 6);
        g = 84 + ((sub * 50) >> 6);
        b = 134 + ((sub * 9) >> 6);
    } else if (val < 192) {
        ap_uint<8> sub = val - 128;
        r = 31 + ((sub * 80) >> 6);
        g = 134 + ((sub * 44) >> 6);
        b = 143 - ((sub * 65) >> 6);
    } else {
        ap_uint<8> sub = val - 192;
        r = 111 + ((sub * 142) >> 6);
        g = 178 + ((sub * 43) >> 6);
        b = 78 - ((sub * 40) >> 6);
    }
}

void evm_motion_accel(
    video_stream_t&  src_stream,
    state_stream_t&  state_in_stream,
    state_stream_t&  state_out_stream,
    video_stream_t&  dst_stream,
    ap_uint<16>      width,
    ap_uint<16>      height,
    ap_uint<8>       alpha,
    ap_uint<12>      r1_fixed,
    ap_uint<12>      r2_fixed,
    ap_uint<12>      noise_gate,
    ap_uint<3>       disp_mode,
    ap_uint<1>       colormap_sel
) {
    #pragma HLS INTERFACE axis port=src_stream
    #pragma HLS INTERFACE axis port=state_in_stream
    #pragma HLS INTERFACE axis port=state_out_stream
    #pragma HLS INTERFACE axis port=dst_stream

    #pragma HLS INTERFACE s_axilite port=width        bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=height       bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=alpha        bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=r1_fixed     bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=r2_fixed     bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=noise_gate   bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=disp_mode    bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=colormap_sel bundle=CTRL_BUS
    #pragma HLS INTERFACE s_axilite port=return       bundle=CTRL_BUS

    // 片上行缓存 (BRAM): 缓存 4 行用于 5x5 滑动窗口
    ap_uint<8> line_buffer[4][MAX_WIDTH];
    #pragma HLS ARRAY_PARTITION variable=line_buffer complete dim=1

    // 5x5 滑动窗口寄存器
    ap_uint<8> win[5][5];
    #pragma HLS ARRAY_PARTITION variable=win complete dim=0

    ap_int<16> one_minus_r1 = FIXED_ONE - r1_fixed;
    ap_int<16> one_minus_r2 = FIXED_ONE - r2_fixed;

    ROW_LOOP: for (int r = 0; r < height; r++) {
        COL_LOOP: for (int c = 0; c < width; c++) {
            #pragma HLS PIPELINE II=1

            video_axis_t in_pixel = src_stream.read();
            state_axis_t state_in = state_in_stream.read();

            ap_uint<8> red   = in_pixel.data.range(23, 16);
            ap_uint<8> green = in_pixel.data.range(15, 8);
            ap_uint<8> blue  = in_pixel.data.range(7, 0);

            // 1. RGB 转灰度亮度 Y
            ap_uint<8> Y = (77 * red + 150 * green + 29 * blue) >> 8;

            // 2. 5x5 行缓存与滑动窗口更新
            for (int i = 0; i < 4; i++) {
                win[i][4] = line_buffer[i][c];
            }
            win[4][4] = Y;

            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 4; j++) {
                    win[i][j] = win[i][j + 1];
                }
            }

            for (int i = 0; i < 3; i++) {
                line_buffer[i][c] = line_buffer[i + 1][c];
            }
            line_buffer[3][c] = Y;

            // 3. 空间高斯模糊卷积
            ap_int<20> conv_sum = 0;
            for (int i = 0; i < 5; i++) {
                for (int j = 0; j < 5; j++) {
                    conv_sum += win[i][j] * GAUSS_5X5[i][j];
                }
            }
            ap_uint<8> Y_blur = conv_sum >> 8;

            // 4. 空间带通残差 L0 (Q4.12)
            ap_int<20> L0 = ((ap_int<16>)Y - (ap_int<16>)Y_blur) << FIXED_SHIFT;

            // 5. 提取上一帧时域状态
            ap_int<16> prev_lp1 = (ap_int<16>)state_in.data.range(31, 16);
            ap_int<16> prev_lp2 = (ap_int<16>)state_in.data.range(15, 0);

            // 6. 时域一阶差分 IIR 滤波
            ap_int<24> new_lp1 = ((one_minus_r1 * prev_lp1) + (r1_fixed * L0)) >> FIXED_SHIFT;
            ap_int<24> new_lp2 = ((one_minus_r2 * prev_lp2) + (r2_fixed * L0)) >> FIXED_SHIFT;

            state_axis_t state_out;
            state_out.data.range(31, 16) = (ap_int<16>)new_lp1;
            state_out.data.range(15, 0)  = (ap_int<16>)new_lp2;
            state_out.user = state_in.user;
            state_out.last = state_in.last;
            state_out_stream.write(state_out);

            // 7. 提取高频时域微动量并执行硬件时域死区门限 (Coring)
            ap_int<16> delta_y = new_lp1 - new_lp2;
            ap_uint<16> abs_delta = (delta_y < 0) ? (ap_uint<16>)(-delta_y) : (ap_uint<16>)delta_y;
            ap_int<16> delta_clean = 0;
            if (abs_delta > noise_gate) {
                delta_clean = (delta_y > 0) ? (ap_int<16>)(abs_delta - noise_gate) : (ap_int<16>)(-(ap_int<16>)(abs_delta - noise_gate));
            }

            // 8. 计算运动放大像素 (使用剔除噪点后的 clean 信号)
            ap_int<24> amplified_val = (alpha * delta_clean) >> FIXED_SHIFT;
            ap_int<16> Y_amplified = (ap_int<16>)Y + amplified_val;
            ap_uint<8> Y_out = (Y_amplified > 255) ? 255 : ((Y_amplified < 0) ? 0 : (ap_uint<8>)Y_amplified);

            ap_int<16> delta_bright = (ap_int<16>)Y_out - (ap_int<16>)Y;
            ap_int<16> new_r = (ap_int<16>)red + delta_bright;
            ap_int<16> new_g = (ap_int<16>)green + delta_bright;
            ap_int<16> new_b = (ap_int<16>)blue + delta_bright;

            ap_uint<8> r_amp = (new_r > 255) ? 255 : ((new_r < 0) ? 0 : (ap_uint<8>)new_r);
            ap_uint<8> g_amp = (new_g > 255) ? 255 : ((new_g < 0) ? 0 : (ap_uint<8>)new_g);
            ap_uint<8> b_amp = (new_b > 255) ? 255 : ((new_b < 0) ? 0 : (ap_uint<8>)new_b);

            // 9. 【新特性】纯净微动能量幅度场提取与伪彩色热力图映射
            ap_uint<16> abs_clean = (delta_clean < 0) ? (ap_uint<16>)(-delta_clean) : (ap_uint<16>)delta_clean;
            ap_uint<24> mag_scaled = (abs_clean * alpha) >> (FIXED_SHIFT - 3);
            ap_uint<8> mag_u8 = (mag_scaled > 255) ? 255 : (ap_uint<8>)mag_scaled;

            ap_uint<8> r_heat, g_heat, b_heat;
            if (colormap_sel == 0) {
                get_jet_hardware(mag_u8, r_heat, g_heat, b_heat);
            } else {
                get_viridis_hardware(mag_u8, r_heat, g_heat, b_heat);
            }

            // Alpha 混合叠加: (256-a)*orig + a*heat
            ap_uint<8> blend_w = (mag_u8 * 180) >> 8;
            ap_uint<8> r_overlay = (((256 - blend_w) * red) + (blend_w * r_heat)) >> 8;
            ap_uint<8> g_overlay = (((256 - blend_w) * green) + (blend_w * g_heat)) >> 8;
            ap_uint<8> b_overlay = (((256 - blend_w) * blue) + (blend_w * b_heat)) >> 8;

            // 10. 多路显示选择 Mux
            video_axis_t out_pixel;
            out_pixel.user = in_pixel.user;
            out_pixel.last = in_pixel.last;

            if (disp_mode == 1) { // 1: 原画直通
                out_pixel.data = in_pixel.data;
            } else if (disp_mode == 2) { // 2: 运动放大
                out_pixel.data.range(23, 16) = r_amp;
                out_pixel.data.range(15, 8)  = g_amp;
                out_pixel.data.range(7, 0)   = b_amp;
            } else if (disp_mode == 3) { // 3: 纯热力图
                out_pixel.data.range(23, 16) = r_heat;
                out_pixel.data.range(15, 8)  = g_heat;
                out_pixel.data.range(7, 0)   = b_heat;
            } else if (disp_mode == 4) { // 4: 热力图半透明叠加 (推荐)
                out_pixel.data.range(23, 16) = r_overlay;
                out_pixel.data.range(15, 8)  = g_overlay;
                out_pixel.data.range(7, 0)   = b_overlay;
            } else if (disp_mode == 5) { // 5: 左右分屏对比
                if (c < (width >> 1)) {
                    out_pixel.data = in_pixel.data; // 左边原画
                } else if (c == (width >> 1)) {
                    out_pixel.data = 0x0000FF;      // 中间红线
                } else {
                    out_pixel.data.range(23, 16) = r_overlay; // 右边热力叠加
                    out_pixel.data.range(15, 8)  = g_overlay;
                    out_pixel.data.range(7, 0)   = b_overlay;
                }
            } else {
                out_pixel.data.range(23, 16) = r_overlay;
                out_pixel.data.range(15, 8)  = g_overlay;
                out_pixel.data.range(7, 0)   = b_overlay;
            }

            dst_stream.write(out_pixel);
        }
    }
}
