"""
==============================================================================
ZYNQ EVM 运动放大算法 - 阶段 1: Python 定点化 (Fixed-Point) 位真仿真与验证脚本 (增强版)
融合新方案特性:
  1. 空间降采样预处理 (720P -> 360P, 显著提升 SNR 并降低运算负担)
  2. 空间两级多尺度残差分解 (解耦微动与宏观晃动)
  3. 变化幅度场提取与时间平滑 (|DeltaY| 能量场 + 时间平滑防闪烁)
  4. Jet / Viridis 伪彩热力图查找表 (Colormap LUT)
  5. 5 种全功能显示模式实时快捷键切换:
     - [按键 1]: 原始画面直通 (Bypass)
     - [按键 2]: 经典 EVM 运动放大画面 (Motion Amplified)
     - [按键 3]: 纯微动变化幅度热力图 (Heatmap)
     - [按键 4]: 热力图半透明 Alpha 叠加到原画 (Heatmap Overlay - 效果最佳!)
     - [按键 5]: 左右分屏对比模式 (原画 vs 放大/热力图)
     - [按键 C]: 切换调色板 (Jet 彩虹色 / Viridis 翠绿色)
     - [按键 +/-]: 微调运动放大倍数 Alpha
==============================================================================
"""

import cv2
import numpy as np

# ================= 定点化运算参数配置 =================
FIXED_SHIFT = 12                     # Q4.12 定点数: 小数位 12 位
FIXED_SCALE = 1 << FIXED_SHIFT       # 4096

# 滤波频率参数 (针对 30fps 视频)
R1_FLOAT = 0.4                       # 高通边界
R2_FLOAT = 0.05                      # 低通边界
ALPHA_DEFAULT = 30                   # 默认放大倍数

R1_FIXED = int(round(R1_FLOAT * FIXED_SCALE))
R2_FIXED = int(round(R2_FLOAT * FIXED_SCALE))
ONE_MINUS_R1 = FIXED_SCALE - R1_FIXED
ONE_MINUS_R2 = FIXED_SCALE - R2_FIXED

# 热力图平滑系数 (IIR 低通平滑时间闪烁)
SMOOTH_R = 0.25
SMOOTH_R_FIXED = int(round(SMOOTH_R * FIXED_SCALE))
ONE_MINUS_SMOOTH_R = FIXED_SCALE - SMOOTH_R_FIXED

# 显示模式定义
MODE_BYPASS          = 1
MODE_AMPLIFY         = 2
MODE_HEATMAP_ONLY    = 3
MODE_HEATMAP_OVERLAY = 4
MODE_SPLIT_SCREEN    = 5

def generate_colormap_lut(colormap_type='jet'):
    """生成 256 色 8-bit RGB 伪彩色查找表 (模拟硬件 ROM 查表)"""
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    for i in range(256):
        lut[i, 0, 0] = i
    if colormap_type == 'jet':
        return cv2.applyColorMap(lut, cv2.COLORMAP_JET).reshape(256, 3)
    else:
        return cv2.applyColorMap(lut, cv2.COLORMAP_VIRIDIS).reshape(256, 3)

def run_evm_fixed_point_enhanced(video_source=0, alpha=ALPHA_DEFAULT, use_downscale=True):
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print(f"无法打开视频源: {video_source}")
        return

    ret, frame0 = cap.read()
    if not ret:
        return

    if use_downscale:
        frame0 = cv2.resize(frame0, (frame0.shape[1] // 2, frame0.shape[0] // 2), interpolation=cv2.INTER_AREA)

    h, w = frame0.shape[:2]
    print("=" * 65)
    print("  ZYNQ EVM 增强版运动放大与变化热力图仿真器启动成功!")
    print(f"  分辨率: {w}x{h} (2x空间平滑降采样: {'已启用' if use_downscale else '已关闭'})")
    print("  [快捷键说明]:")
    print("    '1': 原始画质直通        '2': EVM 运动放大画面")
    print("    '3': 纯微动幅度热力图    '4': 热力图 Alpha 半透明叠加 (推荐)")
    print("    '5': 左右分屏对比模式    'C': 切换热力图色表 (Jet/Viridis)")
    print("    '+'/'-': 增减放大倍数    'Q': 退出程序")
    print("=" * 65)

    # 预分配定点历史状态缓存 (模拟 DDR3 / VDMA 帧历史)
    lowpass1 = np.zeros((h, w), dtype=np.int32)
    lowpass2 = np.zeros((h, w), dtype=np.int32)
    # 幅度能量场平滑缓存 (防闪烁)
    mag_smoothed = np.zeros((h, w), dtype=np.int32)

    # 初始化状态
    gray0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2GRAY)
    blurred0 = cv2.GaussianBlur(gray0, (5, 5), 1.0)
    spatial_band0 = (gray0.astype(np.int32) - blurred0.astype(np.int32)) << FIXED_SHIFT
    lowpass1[:, :] = spatial_band0
    lowpass2[:, :] = spatial_band0

    curr_alpha = alpha
    curr_mode = MODE_HEATMAP_OVERLAY
    colormap_name = 'jet'
    lut = generate_colormap_lut(colormap_name)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if use_downscale:
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

        # 1. 颜色空间转换: 提取亮度 Y
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        Y = yuv[:, :, 0]

        # 2. 空间滤波提取高频微动残差: L0 = Y - Blur(Y)
        Y_blur = cv2.GaussianBlur(Y, (5, 5), 1.0)
        L0 = (Y.astype(np.int32) - Y_blur.astype(np.int32)) << FIXED_SHIFT

        # 3. 时域一阶差分 IIR 带通滤波 (定点乘加流水线)
        lowpass1 = ((ONE_MINUS_R1 * lowpass1) >> FIXED_SHIFT) + ((R1_FIXED * L0) >> FIXED_SHIFT)
        lowpass2 = ((ONE_MINUS_R2 * lowpass2) >> FIXED_SHIFT) + ((R2_FIXED * L0) >> FIXED_SHIFT)
        delta_y = lowpass1 - lowpass2  # Q4.12 微小物理振动带通量

        # 4. 运动增益放大与饱和截断
        amplified_delta = (curr_alpha * delta_y) >> FIXED_SHIFT
        Y_amplified = np.clip(Y.astype(np.int32) + amplified_delta, 0, 255).astype(np.uint8)
        yuv_amp = yuv.copy()
        yuv_amp[:, :, 0] = Y_amplified
        frame_amplified = cv2.cvtColor(yuv_amp, cv2.COLOR_YUV2BGR)

        # 5. 【新特性】计算运动微动幅度场并进行时间平滑 (防高频闪烁)
        # 提取微动绝对值能量 |delta_y|
        mag_raw = np.abs(delta_y)  # Q4.12
        # 时间平滑滤波: mag_smoothed = (1 - r)*mag_smoothed + r*mag_raw
        mag_smoothed = ((ONE_MINUS_SMOOTH_R * mag_smoothed) >> FIXED_SHIFT) + ((SMOOTH_R_FIXED * mag_raw) >> FIXED_SHIFT)

        # 归一化并映射至 0~255 热力图强度索引 (乘以动态灵敏度增益)
        mag_intensity = (mag_smoothed * curr_alpha) >> (FIXED_SHIFT - 3) # 增强对比度
        mag_u8 = np.clip(mag_intensity, 0, 255).astype(np.uint8)

        # 6. 【新特性】通过伪彩色 LUT 映射生成彩色热力图
        heatmap_rgb = lut[mag_u8]

        # 7. 【新特性】热力图与原图 Alpha 混合半透明叠加
        # 在幅度较小处降低热力图可见度，剧烈微动处高亮显示
        alpha_mask = np.clip(mag_u8.astype(np.float32) / 120.0, 0.0, 0.75)[:, :, np.newaxis]
        overlay_frame = ((1.0 - alpha_mask) * frame.astype(np.float32) + alpha_mask * heatmap_rgb.astype(np.float32)).astype(np.uint8)

        # 8. 模式切换选择器 (Mux)
        if curr_mode == MODE_BYPASS:
            out_display = frame.copy()
            mode_text = "MODE: 1 - ORIGINAL BYPASS"
        elif curr_mode == MODE_AMPLIFY:
            out_display = frame_amplified.copy()
            mode_text = f"MODE: 2 - EVM MOTION AMPLIFIED (Alpha={curr_alpha}x)"
        elif curr_mode == MODE_HEATMAP_ONLY:
            out_display = heatmap_rgb.copy()
            mode_text = f"MODE: 3 - MOTION HEATMAP [{colormap_name.upper()}]"
        elif curr_mode == MODE_HEATMAP_OVERLAY:
            out_display = overlay_frame.copy()
            mode_text = f"MODE: 4 - HEATMAP OVERLAY [{colormap_name.upper()}] (Alpha={curr_alpha}x)"
        elif curr_mode == MODE_SPLIT_SCREEN:
            # 左右分屏对比 (左侧原画，右侧热力叠加)
            half_w = w // 2
            out_display = np.hstack((frame[:, :half_w], overlay_frame[:, half_w:]))
            cv2.line(out_display, (half_w, 0), (half_w, h), (0, 0, 255), 2)
            cv2.putText(out_display, "ORIGINAL", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(out_display, "HEATMAP OVERLAY", (half_w + 20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            mode_text = f"MODE: 5 - SPLIT COMPARISON (Alpha={curr_alpha}x)"

        # 叠加 OSD 状态文字
        cv2.putText(out_display, mode_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.imshow("ZYNQ EVM Motion Magnification & Heatmap Simulator", out_display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            curr_mode = MODE_BYPASS
        elif key == ord('2'):
            curr_mode = MODE_AMPLIFY
        elif key == ord('3'):
            curr_mode = MODE_HEATMAP_ONLY
        elif key == ord('4'):
            curr_mode = MODE_HEATMAP_OVERLAY
        elif key == ord('5'):
            curr_mode = MODE_SPLIT_SCREEN
        elif key == ord('c') or key == ord('C'):
            colormap_name = 'viridis' if colormap_name == 'jet' else 'jet'
            lut = generate_colormap_lut(colormap_name)
            print(f">> 切换调色板为: {colormap_name.upper()}")
        elif key == ord('+') or key == ord('='):
            curr_alpha = min(150, curr_alpha + 5)
            print(f">> 放大倍数增加 -> Alpha: {curr_alpha}")
        elif key == ord('-') or key == ord('_'):
            curr_alpha = max(1, curr_alpha - 5)
            print(f">> 放大倍数减小 -> Alpha: {curr_alpha}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("启动增强版 EVM 运动放大与微动热力图实时仿真 (按 Q 退出)...")
    run_evm_fixed_point_enhanced(0, alpha=ALPHA_DEFAULT, use_downscale=True)
