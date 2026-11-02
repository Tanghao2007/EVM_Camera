#ifndef EVM_MOTION_HLS_H_
#define EVM_MOTION_HLS_H_

#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

// 视频分辨率宏定义 (支持 720P / 降采样 360P)
#define MAX_WIDTH   1280
#define MAX_HEIGHT  720

// 定点格式配置 (Q4.12 格式)
#define FIXED_SHIFT 12
#define FIXED_ONE   (1 << FIXED_SHIFT) // 4096

// AXI-Stream 像素数据包定义 (24-bit RGB888 或 YUV444)
typedef ap_axiu<24, 1, 1, 1> video_axis_t;
typedef hls::stream<video_axis_t> video_stream_t;

// 时域状态包 (每个像素 32-bit: lowpass1 16-bit + lowpass2 16-bit)
typedef ap_uint<32> state_pack_t;
typedef ap_axiu<32, 1, 1, 1> state_axis_t;
typedef hls::stream<state_axis_t> state_stream_t;

// 顶层加速 IP 声明
void evm_motion_accel(
    video_stream_t&  src_stream,          // 输入视频流 (RGB888)
    state_stream_t&  state_in_stream,     // 从 DDR3/VDMA 读入上一帧历史状态
    state_stream_t&  state_out_stream,    // 写入 DDR3/VDMA 的当前帧状态
    video_stream_t&  dst_stream,          // 输出视频流 (RGB888 HDMI)
    ap_uint<16>      width,               // 图像宽度
    ap_uint<16>      height,              // 图像高度
    ap_uint<8>       alpha,               // 放大倍数 (0~150)
    ap_uint<12>      r1_fixed,            // 滤波参数 r1 (Q12)
    ap_uint<12>      r2_fixed,            // 滤波参数 r2 (Q12)
    ap_uint<12>      noise_gate,          // 时域噪声门限 (Q12, 硬件软阈值降噪)
    ap_uint<3>       disp_mode,           // 显示模式 (1:直通, 2:放大, 3:热力图叠加)
    ap_uint<1>       colormap_sel         // 调色板选择 (0: Jet, 1: Viridis)
);

#endif // EVM_MOTION_HLS_H_
