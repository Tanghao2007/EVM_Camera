"""
==============================================================================
  ZYNQ-7020 EVM 视讯微动放大与智能分析系统 (自适应缩放工业仪表盘)
  UI 特色:
    1. 全界面由统一的 UI 缩放系数驱动: 字号/间距/控件高度随窗口大小等比缩放,
       窗口最大化或全屏 (F11) 后各部分占比保持协调, 不再出现"视频长不大、
       文字却很小"的失衡;
    2. 双路视频窗口左右严格等宽, 按原始宽高比居中显示, 占据主体垂直空间;
    3. 处理帧率、振动主频、微位移幅值、监测状态 4 张等宽指标卡片位于视频正下方;
    4. 底部控制区分为左右两栏: 左侧模式/调色板/图表开关, 右侧 Alpha 与
       抗噪门限滑块, 标签-滑块-数值三列对齐;
    5. 展开振动频谱图时自动从视频区分得垂直空间, 收起后恢复;
    6. 全双工串口联动: 滑块与按钮即时下发指令至 ZYNQ ARM PS 端 AXI 寄存器,
       支持遥测绘图 (通信协议与视频处理引擎与原版完全一致)。
==============================================================================
"""

import sys
import time
import cv2
import numpy as np
import serial
import serial.tools.list_ports
from collections import deque

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, QSlider,
                             QFileDialog, QFrame, QSizePolicy, QInputDialog,
                             QMessageBox, QShortcut)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF
from PyQt5.QtGui import QFont, QImage, QColor, QPainter, QPainterPath, QKeySequence
import pyqtgraph as pg

# ==================== 全局定点数运算参数配置 ====================
FIXED_SHIFT = 12                     # Q4.12 定点数
FIXED_SCALE = 1 << FIXED_SHIFT       # 4096

MODE_BYPASS          = 1  # 原始视频直通
MODE_AMPLIFY         = 2  # 运动微位移放大
MODE_HEATMAP_OVERLAY = 3  # 热力图半透明叠加 (核心推荐)

COLORMAP_JET     = 0  # 经典彩虹色 (冷蓝 -> 暖红)
COLORMAP_VIRIDIS = 1  # 翠绿色谱 (高对比度感知)

ALARM_DISP_THRESHOLD = 2.5


# ==================== 后台视频采集与 EVM 全局硬件仿真加速引擎 ====================
class EVMVideoEngine(QThread):
    frame_processed = pyqtSignal(QImage, QImage, float, float, int, float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.paused = False
        self.video_source = 1  # 默认使用 MS2109 USB-HDMI 视频采集卡 (设备号 1; 0 为笔记本前置摄像头)
        self.cap = None

        # 算法控制参数
        self.alpha = 35
        self.mode = MODE_HEATMAP_OVERLAY
        self.colormap = COLORMAP_JET
        self.use_downscale = True

        # 时域噪声死区门限 (Q4.12 定点数, 默认 35 * 16 = 560)
        self.noise_gate_fixed = 560

        # 滤波系数配置 (Q12 定点)
        self.r1_fixed = int(round(0.4 * FIXED_SCALE))
        self.r2_fixed = int(round(0.05 * FIXED_SCALE))
        self.smooth_r_fixed = int(round(0.25 * FIXED_SCALE))
        self.one_minus_r1 = FIXED_SCALE - self.r1_fixed
        self.one_minus_r2 = FIXED_SCALE - self.r2_fixed
        self.one_minus_smooth = FIXED_SCALE - self.smooth_r_fixed

        self.jet_lut = self._build_lut(cv2.COLORMAP_JET)
        self.viridis_lut = self._build_lut(cv2.COLORMAP_VIRIDIS)

        self.lowpass1 = None
        self.lowpass2 = None
        self.mag_smoothed = None
        self.prev_w = 0
        self.prev_h = 0
        self.disp_smooth = 0.0

    def _build_lut(self, colormap_type):
        lut = np.zeros((256, 1, 3), dtype=np.uint8)
        for i in range(256):
            lut[i, 0, 0] = i
        return cv2.applyColorMap(lut, colormap_type).reshape(256, 3)

    def set_source(self, source):
        self.video_source = source
        if self.cap:
            self.cap.release()
            self.cap = None
        self.disp_smooth = 0.0
        self.lowpass1 = None
        self.lowpass2 = None
        self.mag_smoothed = None

    def set_noise_gate(self, val):
        # 宽动态范围映射 (Q12 定点，val=500 时对应约 2.0 个灰阶门限，彻底压制 CMOS 散粒暗电流热噪)
        self.noise_gate_fixed = val * 16

    def run(self):
        fps_counter = 0
        fps_timer = time.time()
        curr_fps = 30.0

        while self.running:
            if self.paused:
                time.sleep(0.03)
                continue

            if self.cap is None or not self.cap.isOpened():
                if isinstance(self.video_source, int):
                    self.cap = cv2.VideoCapture(self.video_source, cv2.CAP_DSHOW)
                    if not self.cap.isOpened():
                        self.cap = cv2.VideoCapture(self.video_source)
                    if self.cap.isOpened():
                        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                else:
                    self.cap = cv2.VideoCapture(self.video_source)
                if not self.cap.isOpened():
                    time.sleep(0.1)
                    continue

            ret, frame = self.cap.read()
            if not ret:
                if isinstance(self.video_source, str):
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                else:
                    time.sleep(0.03)
                    continue

            # 1. 空间平滑降采样 (640x360 提升 SNR)
            if self.use_downscale and frame.shape[1] > 640:
                frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)

            h, w = frame.shape[:2]
            orig_frame = frame.copy()

            if self.lowpass1 is None or self.prev_w != w or self.prev_h != h:
                self.lowpass1 = np.zeros((h, w), dtype=np.int32)
                self.lowpass2 = np.zeros((h, w), dtype=np.int32)
                self.mag_smoothed = np.zeros((h, w), dtype=np.int32)
                self.prev_w = w
                self.prev_h = h

            # 2. 提取灰度亮度分量 Y
            yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
            Y = yuv[:, :, 0]

            # 3. 空间 5x5 高斯差分获得微动残差 L0
            Y_blur = cv2.GaussianBlur(Y, (5, 5), 1.0)
            L0 = (Y.astype(np.int32) - Y_blur.astype(np.int32)) << FIXED_SHIFT

            # 4. 时域双 IIR 递归带通滤波
            self.lowpass1 = ((self.one_minus_r1 * self.lowpass1) >> FIXED_SHIFT) + ((self.r1_fixed * L0) >> FIXED_SHIFT)
            self.lowpass2 = ((self.one_minus_r2 * self.lowpass2) >> FIXED_SHIFT) + ((self.r2_fixed * L0) >> FIXED_SHIFT)
            delta_y = self.lowpass1 - self.lowpass2

            # 5. 时域硬件死区软阈值门限 (Temporal Noise Coring)
            gate = self.noise_gate_fixed
            abs_delta = np.abs(delta_y)
            delta_y_denoised = np.where(abs_delta > gate, np.sign(delta_y) * (abs_delta - gate), 0)

            # 剔除边缘 10 像素黑边与卷积边界阶跃效应 (彻底排除黑边带来的几十像素虚假尖峰)
            margin = 10
            if h > 2 * margin and w > 2 * margin:
                inner_delta = delta_y_denoised[margin:-margin, margin:-margin]
            else:
                inner_delta = delta_y_denoised
            inner_abs = np.abs(inner_delta)

            # 采用抗噪鲁棒分位数 (99.5 百分位)，过滤单点孤立热噪点与偶发离群值，仅捕获连片物理结构微动
            robust_raw_disp = float(np.percentile(inner_abs, 99.5)) / (1 << FIXED_SHIFT)
            # 动态非对称 EMA：运动发生时即时敏捷跟进响应 (0.40 历史 + 0.60 新值)，静止时平滑回落 (0.70 历史 + 0.30 新值)
            if robust_raw_disp > self.disp_smooth:
                self.disp_smooth = 0.40 * self.disp_smooth + 0.60 * robust_raw_disp
            else:
                self.disp_smooth = 0.70 * self.disp_smooth + 0.30 * robust_raw_disp

            peak_disp = self.disp_smooth if self.disp_smooth > 0.05 else 0.0
            total_energy = int(np.mean(inner_abs))

            # 提取最大运动区域的即时瞬态符号化位移 (供波形图正负振荡与 FFT 真实谱分析)
            if peak_disp > 0.05:
                thresh_top = np.percentile(inner_abs, 99.5)
                top_active = inner_delta[inner_abs >= thresh_top]
                if len(top_active) > 0:
                    signed_val = float(np.median(top_active)) / (1 << FIXED_SHIFT)
                    inst_disp = float(np.clip(signed_val, -peak_disp, peak_disp))
                else:
                    inst_disp = 0.0
            else:
                inst_disp = 0.0

            # 6. 微动放大运算
            amplified_delta = (self.alpha * delta_y_denoised) >> FIXED_SHIFT
            Y_amp = np.clip(Y.astype(np.int32) + amplified_delta, 0, 255).astype(np.uint8)
            yuv_amp = yuv.copy()
            yuv_amp[:, :, 0] = Y_amp
            amp_bgr = cv2.cvtColor(yuv_amp, cv2.COLOR_YUV2BGR)

            # 7. 变化热力图生成与半透明叠加
            mag_clean = np.abs(delta_y_denoised)
            self.mag_smoothed = ((self.one_minus_smooth * self.mag_smoothed) >> FIXED_SHIFT) + ((self.smooth_r_fixed * mag_clean) >> FIXED_SHIFT)
            mag_scaled = (self.mag_smoothed * self.alpha) >> (FIXED_SHIFT - 3)
            mag_u8 = np.clip(mag_scaled, 0, 255).astype(np.uint8)

            lut = self.jet_lut if self.colormap == COLORMAP_JET else self.viridis_lut
            heatmap_bgr = lut[mag_u8]

            alpha_mask = np.clip(mag_u8.astype(np.float32) / 100.0, 0.0, 0.75)[:, :, np.newaxis]
            overlay_bgr = ((1.0 - alpha_mask) * frame.astype(np.float32) + alpha_mask * heatmap_bgr.astype(np.float32)).astype(np.uint8)

            # 8. 模式选通输出
            if self.mode == MODE_BYPASS:
                final_processed = orig_frame
            elif self.mode == MODE_AMPLIFY:
                final_processed = amp_bgr
            else:
                final_processed = overlay_bgr

            # 9. 实际帧率统计
            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                curr_fps = fps_counter / (time.time() - fps_timer)
                fps_counter = 0
                fps_timer = time.time()

            qimg_orig = self._mat_to_qimage(orig_frame)
            qimg_proc = self._mat_to_qimage(final_processed)

            self.frame_processed.emit(qimg_orig, qimg_proc, curr_fps, peak_disp, total_energy, inst_disp)
            time.sleep(0.015)

        if self.cap:
            self.cap.release()

    def _mat_to_qimage(self, mat):
        rgb = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        return QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()

    def stop(self):
        self.running = False
        self.wait()


# ==================== 自适应视频显示控件 ====================
class VideoView(QWidget):
    """
    自绘视频控件:
      - 始终按原始宽高比居中显示 (留黑边), 随窗口任意缩放;
      - 直接 drawImage, 不使用 QLabel.setPixmap, 因此不会反过来撑大/锁死布局;
      - 无信号时显示占位提示。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._image = None
        self.radius = 8
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(240, 135)

    def set_image(self, qimg):
        self._image = qimg
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)

        rect = QRectF(self.rect())
        clip = QPainterPath()
        clip.addRoundedRect(rect, self.radius, self.radius)
        p.setClipPath(clip)
        p.fillRect(rect, QColor("#0B1220"))

        if self._image is None or self._image.isNull():
            f = self.font()
            f.setPixelSize(max(12, int(self.height() * 0.05)))
            p.setFont(f)
            p.setPen(QColor("#64748B"))
            p.drawText(self.rect(), Qt.AlignCenter, "等待视频信号 …")
            return

        iw, ih = self._image.width(), self._image.height()
        scale = min(rect.width() / iw, rect.height() / ih)
        tw, th = iw * scale, ih * scale
        target = QRectF((rect.width() - tw) / 2.0, (rect.height() - th) / 2.0, tw, th)
        p.drawImage(target, self._image)


# ==================== 主窗口 UI 界面设计 (自适应缩放工业仪表盘) ====================
class EVMAdvancedDashboard(QMainWindow):
    # 设计基准窗口尺寸: 在此尺寸下 UI 缩放系数为 1.0, 窗口越大各部分按比例放大
    BASE_W = 1280
    BASE_H = 800
    SCALE_MIN = 0.85
    SCALE_MAX = 2.4

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZYNQ-7020 视讯微动放大与智能分析系统")
        self.setMinimumSize(1100, 680)

        # 初始窗口按屏幕可用区域的比例确定并居中, 而不是写死像素
        screen = QApplication.primaryScreen().availableGeometry()
        self.resize(int(screen.width() * 0.78), int(screen.height() * 0.84))
        frame = self.frameGeometry()
        frame.moveCenter(screen.center())
        self.move(frame.topLeft())

        # 串口通信管理
        self.ser = None
        self.serial_rx_timer = QTimer(self)
        self.serial_rx_timer.timeout.connect(self.read_serial_data)

        self.ui_scale = None          # 当前 UI 缩放系数 (None 表示尚未应用)
        self._alarm_state = None      # 监测状态缓存, 仅在变化时刷新样式

        self.engine = EVMVideoEngine()
        self.engine.frame_processed.connect(self.on_frame_ready)

        self.disp_history = deque([0.0] * 128, maxlen=128)

        self.init_ui()

        # 全屏快捷键: F11 切换全屏, Esc 退出全屏
        QShortcut(QKeySequence("F11"), self, activated=self.toggle_fullscreen)
        QShortcut(QKeySequence("Esc"), self, activated=self.exit_fullscreen)

        self.engine.start()

    # ------------------------------------------------------------------
    #  缩放系统: 一切尺寸 / 字号 / 间距都由 ui_scale 驱动
    # ------------------------------------------------------------------
    def _s(self, v):
        return max(1, int(round(v * self.ui_scale)))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_scale()

    def _refresh_scale(self):
        if not hasattr(self, "main_layout"):
            return
        raw = min(self.width() / self.BASE_W, self.height() / self.BASE_H)
        scale = round(max(self.SCALE_MIN, min(raw, self.SCALE_MAX)) * 20) / 20.0  # 0.05 步进, 避免频繁重排
        if scale != self.ui_scale:
            self.ui_scale = scale
            self._apply_scale()

    def _build_stylesheet(self):
        s = self._s
        return f"""
            QMainWindow {{ background-color: #EEF2F7; }}
            QWidget {{
                color: #0F172A;
                font-family: 'Microsoft YaHei', 'Segoe UI', 'PingFang SC', 'Noto Sans CJK SC', Arial, sans-serif;
                font-size: {s(14)}px;
            }}
            QLabel {{ background: transparent; border: none; }}

            /* ---------- 顶部横栏 ---------- */
            QFrame#HeaderBar {{ background-color: #1E3A8A; border: none; border-radius: {s(10)}px; }}
            QLabel#HeaderTitle {{ color: #FFFFFF; font-size: {s(22)}px; font-weight: bold; }}
            QLabel#HeaderMode {{ color: #FDE047; font-size: {s(16)}px; font-weight: bold; }}
            QLabel#HeaderSerial {{ color: #93C5FD; font-size: {s(15)}px; font-weight: bold; }}
            QLabel#HeaderSerial[connected="true"] {{ color: #4ADE80; }}

            QLabel#SectionLabel {{ color: #1E293B; font-size: {s(16)}px; font-weight: bold; }}

            /* ---------- 面板 / 卡片 ---------- */
            QFrame#Panel {{
                background-color: #FFFFFF;
                border: 1px solid #D5DEE9;
                border-radius: {s(10)}px;
            }}
            QLabel#PanelTitle {{
                color: #1E3A8A;
                font-size: {s(15)}px;
                font-weight: bold;
                border-left: {s(4)}px solid #2563EB;
                padding-left: {s(8)}px;
            }}
            QFrame#MetricCard {{
                background-color: #FFFFFF;
                border: 1px solid #D5DEE9;
                border-radius: {s(10)}px;
            }}
            QFrame#MetricCard[alarm="true"] {{
                background-color: #FEF2F2;
                border: 1px solid #FCA5A5;
            }}
            QLabel#MetricTitle {{ color: #64748B; font-size: {s(14)}px; font-weight: bold; }}
            QLabel#MetricValue {{ font-size: {s(28)}px; font-weight: bold; }}
            QLabel#MetricValue[tone="blue"]  {{ color: #2563EB; }}
            QLabel#MetricValue[tone="teal"]  {{ color: #0D9488; }}
            QLabel#MetricValue[tone="amber"] {{ color: #D97706; }}
            QLabel#MetricValue[tone="ok"]    {{ color: #16A34A; }}
            QLabel#MetricValue[tone="alarm"] {{ color: #DC2626; }}

            QFrame#Divider {{ background-color: #E2E8F0; border: none; }}

            /* ---------- 按钮 ---------- */
            QPushButton {{
                background-color: #FFFFFF;
                color: #1E293B;
                border: 1px solid #CBD5E1;
                border-radius: {s(8)}px;
                padding: {s(5)}px {s(14)}px;
                font-size: {s(14)}px;
                font-weight: bold;
            }}
            QPushButton:hover {{ background-color: #F1F5F9; border-color: #94A3B8; }}
            QPushButton:pressed {{ background-color: #E2E8F0; }}
            QPushButton[active="true"] {{
                background-color: #2563EB; color: #FFFFFF; border: 1px solid #1D4ED8;
            }}
            QPushButton[active="true"]:hover {{ background-color: #1D4ED8; }}
            QPushButton#SerialBtn {{
                background-color: #0284C7; color: #FFFFFF; border: none;
            }}
            QPushButton#SerialBtn:hover {{ background-color: #0369A1; }}
            QPushButton#SerialBtn[connected="true"] {{ background-color: #059669; }}
            QPushButton#SerialBtn[connected="true"]:hover {{ background-color: #047857; }}

            /* ---------- 滑块 ---------- */
            QLabel#FieldLabel {{ color: #1E293B; font-size: {s(14)}px; font-weight: bold; }}
            QLabel#ValueBlue  {{ color: #2563EB; font-family: Consolas, 'Courier New', monospace; font-size: {s(16)}px; font-weight: bold; }}
            QLabel#ValueGreen {{ color: #059669; font-family: Consolas, 'Courier New', monospace; font-size: {s(16)}px; font-weight: bold; }}
            QSlider::groove:horizontal {{
                border: none; height: {s(8)}px; background: #E2E8F0; border-radius: {s(4)}px;
            }}
            QSlider::sub-page:horizontal {{ background: #2563EB; border-radius: {s(4)}px; }}
            QSlider::handle:horizontal {{
                background: #1D4ED8; width: {s(18)}px;
                margin: -{s(5)}px 0; border-radius: {s(9)}px;
            }}
            QSlider::handle:horizontal:hover {{ background: #1E40AF; }}
        """

    def _apply_scale(self):
        s = self._s
        self.setStyleSheet(self._build_stylesheet())

        self.main_layout.setContentsMargins(s(18), s(12), s(18), s(16))
        self.main_layout.setSpacing(s(10))

        # 顶部横栏
        self.header_bar.setFixedHeight(s(58))
        self.header_layout.setContentsMargins(s(24), s(6), s(24), s(6))
        self.header_layout.setSpacing(s(26))

        # 快捷控制条
        self.action_bar.setContentsMargins(s(2), 0, s(2), 0)
        self.action_bar.setSpacing(s(10))
        for btn in self._action_buttons:
            btn.setMinimumHeight(s(40))

        # 双路视频
        self.video_row.setSpacing(s(14))
        for lay in self._panel_layouts:
            lay.setContentsMargins(s(16), s(14), s(16), s(16))
            lay.setSpacing(s(10))
        for view in (self.video_left, self.video_right):
            view.radius = s(8)
            view.update()

        # 指标卡片
        self.metric_bar.setSpacing(s(14))
        for lay in self._card_layouts:
            lay.setContentsMargins(s(12), s(10), s(12), s(12))
            lay.setSpacing(s(2))

        # 底部控制区: 左右两栏、两行严格对齐
        self.ctrl_row.setSpacing(s(24))
        for grid in (self.left_grid, self.right_grid):
            grid.setHorizontalSpacing(s(12))
            grid.setVerticalSpacing(s(10))
            grid.setRowMinimumHeight(0, s(42))
            grid.setRowMinimumHeight(1, s(42))
        for btn in self._grid_buttons:
            btn.setMinimumHeight(s(38))
        for sl in (self.slider_alpha, self.slider_gate):
            sl.setMinimumHeight(s(28))
        self.lbl_alpha_val.setMinimumWidth(s(58))
        self.lbl_gate_val.setMinimumWidth(s(58))

        # 频谱图区域
        self.chart_container.setMinimumHeight(s(125))
        self.chart_layout.setSpacing(s(12))
        self._style_plot(self.plot_wave, "<b>实时微振动位移波形 (px)</b>", self.curve_wave, "#2563EB")
        self._style_plot(self.plot_fft, "<b>振动能量频谱分析 FFT (Hz)</b>", self.curve_fft, "#E11D48")

    def _style_plot(self, plot, title_html, curve, color):
        s = self._s
        plot.setTitle(title_html, color="#1E293B", size=f"{s(13)}px")
        tick_font = QFont()
        tick_font.setPixelSize(s(12))
        for axis_name in ("bottom", "left"):
            axis = plot.getAxis(axis_name)
            axis.setStyle(tickFont=tick_font)
            axis.setTextPen(pg.mkPen("#475569"))
            axis.setPen(pg.mkPen("#94A3B8"))
        curve.setPen(pg.mkPen(color=color, width=max(2, s(2))))

    @staticmethod
    def _set_prop(widget, name, value):
        """修改动态属性并刷新样式 (用于属性选择器 [active="true"] 等)"""
        widget.setProperty(name, value)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    # ------------------------------------------------------------------
    #  界面搭建
    # ------------------------------------------------------------------
    def init_ui(self):
        self._panel_layouts = []
        self._card_layouts = []
        self._action_buttons = []
        self._grid_buttons = []

        main_widget = QWidget()
        self.main_layout = QVBoxLayout(main_widget)

        # ==================== 1. 顶部深蓝科技横栏 ====================
        self.header_bar = QFrame()
        self.header_bar.setObjectName("HeaderBar")
        self.header_layout = QHBoxLayout(self.header_bar)

        self.lbl_main_title = QLabel("ZYNQ-7020 视讯微动放大与智能分析系统")
        self.lbl_main_title.setObjectName("HeaderTitle")
        self.header_layout.addWidget(self.lbl_main_title)
        self.header_layout.addStretch()

        self.lbl_hdr_mode = QLabel("模式: 热力图叠加")
        self.lbl_hdr_mode.setObjectName("HeaderMode")
        self.header_layout.addWidget(self.lbl_hdr_mode)

        self.lbl_hdr_serial = QLabel("● 开发板: 未连接")
        self.lbl_hdr_serial.setObjectName("HeaderSerial")
        self.lbl_hdr_serial.setProperty("connected", "false")
        self.header_layout.addWidget(self.lbl_hdr_serial)

        self.main_layout.addWidget(self.header_bar, 0)

        # ==================== 2. 次级快捷控制条 ====================
        self.action_bar = QHBoxLayout()

        lbl_section = QLabel("📊 实时运行控制")
        lbl_section.setObjectName("SectionLabel")
        self.action_bar.addWidget(lbl_section)
        self.action_bar.addStretch()

        self.btn_open_file = QPushButton("📁 导入视频")
        self.btn_open_file.clicked.connect(self.select_video_file)
        self.btn_camera = QPushButton("📹 已激活设备 [1]")
        self.btn_camera.clicked.connect(self.start_camera)
        self.btn_pause = QPushButton("⏸ 暂停/播放")
        self.btn_pause.clicked.connect(self.toggle_pause)
        self.btn_serial = QPushButton("🔌 连接开发板串口")
        self.btn_serial.setObjectName("SerialBtn")
        self.btn_serial.setProperty("connected", "false")
        self.btn_serial.clicked.connect(self.connect_serial_dialog)

        for btn in (self.btn_open_file, self.btn_camera, self.btn_pause, self.btn_serial):
            self._action_buttons.append(btn)
            self.action_bar.addWidget(btn)

        self.main_layout.addLayout(self.action_bar, 0)

        # ==================== 3. 核心双屏视频对比区域 (占据全部剩余空间) ====================
        self.video_row = QHBoxLayout()

        left_panel, left_layout = self._make_panel("原始视频监控")
        self.video_left = VideoView()
        left_layout.addWidget(self.video_left, 1)

        right_panel, right_layout = self._make_panel("微动放大 / 热力图叠加")
        self.video_right = VideoView()
        right_layout.addWidget(self.video_right, 1)

        self.video_row.addWidget(left_panel, 1)    # 左右严格等宽
        self.video_row.addWidget(right_panel, 1)
        self.main_layout.addLayout(self.video_row, 10)

        # ==================== 4. 状态指标卡片栏 (4 张等宽) ====================
        self.metric_bar = QHBoxLayout()

        self.card_fps, self.val_fps = self._create_card("处理帧率", "30.0 fps", "blue")
        self.card_freq, self.val_freq = self._create_card("振动主频", "0.0 Hz", "teal")
        self.card_disp, self.val_disp = self._create_card("微位移幅值", "0.00 px", "amber")
        self.card_alarm, self.val_alarm = self._create_card("监测状态", "正常", "ok")

        for card in (self.card_fps, self.card_freq, self.card_disp, self.card_alarm):
            self.metric_bar.addWidget(card, 1)
        self.main_layout.addLayout(self.metric_bar, 0)

        # ==================== 5. 底部系统控制与参数滑块栏 ====================
        self.bottom_panel, bottom_layout = self._make_panel("系统模式与参数动态调节")

        self.ctrl_row = QHBoxLayout()

        # ---- 左栏: 模式按钮 (第 1 行) + 调色板 / 图表开关 (第 2 行) ----
        self.left_grid = QGridLayout()

        self.btn_mode1 = QPushButton("1: 原始视频直通")
        self.btn_mode2 = QPushButton("2: 运动微动放大")
        self.btn_mode3 = QPushButton("3: 热力图叠加 (推荐)")
        self.btn_palette = QPushButton("🎨 切换调色板: Jet 彩虹")
        self.btn_toggle_chart = QPushButton("📈 显示实时振动频谱")

        self.mode_buttons = [self.btn_mode1, self.btn_mode2, self.btn_mode3]
        for idx, btn in enumerate(self.mode_buttons, start=1):
            btn.clicked.connect(lambda checked, m=idx: self.switch_mode(m))
            self.left_grid.addWidget(btn, 0, (idx - 1) * 2, 1, 2)
        self.left_grid.addWidget(self.btn_palette, 1, 0, 1, 3)
        self.left_grid.addWidget(self.btn_toggle_chart, 1, 3, 1, 3)
        for c in range(6):
            self.left_grid.setColumnStretch(c, 1)

        self.btn_palette.clicked.connect(self.toggle_palette)
        self.btn_toggle_chart.clicked.connect(self.toggle_chart_visibility)

        for btn in self.mode_buttons + [self.btn_palette, self.btn_toggle_chart]:
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self._grid_buttons.append(btn)
        self._highlight_mode_button(3)

        # ---- 右栏: 两条参数滑块 (标签 / 滑块 / 数值 三列对齐) ----
        self.right_grid = QGridLayout()

        lbl_alpha_title = QLabel("放大倍数 Alpha:")
        lbl_alpha_title.setObjectName("FieldLabel")
        self.slider_alpha = QSlider(Qt.Horizontal)
        self.slider_alpha.setRange(1, 150)
        self.slider_alpha.setValue(35)
        self.slider_alpha.valueChanged.connect(self.on_alpha_changed)
        self.lbl_alpha_val = QLabel("35x")
        self.lbl_alpha_val.setObjectName("ValueBlue")
        self.lbl_alpha_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        lbl_gate_title = QLabel("抗噪门限 Gate:")
        lbl_gate_title.setObjectName("FieldLabel")
        self.slider_gate = QSlider(Qt.Horizontal)
        self.slider_gate.setRange(0, 500)
        self.slider_gate.setValue(35)
        self.slider_gate.valueChanged.connect(self.on_noise_gate_changed)
        self.lbl_gate_val = QLabel("35")
        self.lbl_gate_val.setObjectName("ValueGreen")
        self.lbl_gate_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        self.right_grid.addWidget(lbl_alpha_title, 0, 0)
        self.right_grid.addWidget(self.slider_alpha, 0, 1)
        self.right_grid.addWidget(self.lbl_alpha_val, 0, 2)
        self.right_grid.addWidget(lbl_gate_title, 1, 0)
        self.right_grid.addWidget(self.slider_gate, 1, 1)
        self.right_grid.addWidget(self.lbl_gate_val, 1, 2)
        self.right_grid.setColumnStretch(1, 1)

        divider = QFrame()
        divider.setObjectName("Divider")
        divider.setFixedWidth(1)

        self.ctrl_row.addLayout(self.left_grid, 5)
        self.ctrl_row.addWidget(divider)
        self.ctrl_row.addLayout(self.right_grid, 5)
        bottom_layout.addLayout(self.ctrl_row)

        # 实时微振动波形与 FFT 谱图 (默认折叠, 展开后自动分得更多垂直空间)
        self.chart_container = QWidget()
        self.chart_layout = QHBoxLayout(self.chart_container)
        self.chart_layout.setContentsMargins(0, 0, 0, 0)

        self.plot_wave = pg.PlotWidget()
        self.plot_wave.setBackground("#FFFFFF")
        self.plot_wave.showGrid(x=True, y=True, alpha=0.15)
        self.curve_wave = self.plot_wave.plot()
        self.chart_layout.addWidget(self.plot_wave, 1)

        self.plot_fft = pg.PlotWidget()
        self.plot_fft.setBackground("#FFFFFF")
        self.plot_fft.showGrid(x=True, y=True, alpha=0.15)
        self.curve_fft = self.plot_fft.plot()
        self.chart_layout.addWidget(self.plot_fft, 1)

        self.chart_container.setVisible(False)
        bottom_layout.addWidget(self.chart_container, 1)

        self.main_layout.addWidget(self.bottom_panel, 0)
        self.setCentralWidget(main_widget)

        self._refresh_scale()   # 首次应用缩放与样式

    def _make_panel(self, title):
        """白色圆角面板: 顶部标题 + 内容区。返回 (面板, 其垂直布局)"""
        panel = QFrame()
        panel.setObjectName("Panel")
        lay = QVBoxLayout(panel)
        lbl = QLabel(title)
        lbl.setObjectName("PanelTitle")
        lay.addWidget(lbl, 0)
        self._panel_layouts.append(lay)
        return panel, lay

    def _create_card(self, title, init_val, tone):
        card = QFrame()
        card.setObjectName("MetricCard")
        card.setProperty("alarm", "false")
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        lay = QVBoxLayout(card)
        self._card_layouts.append(lay)

        lbl_title = QLabel(title)
        lbl_title.setObjectName("MetricTitle")
        lbl_title.setAlignment(Qt.AlignCenter)

        lbl_val = QLabel(init_val)
        lbl_val.setObjectName("MetricValue")
        lbl_val.setProperty("tone", tone)
        lbl_val.setAlignment(Qt.AlignCenter)

        lay.addWidget(lbl_title)
        lay.addWidget(lbl_val)
        return card, lbl_val

    # ==================== 全屏控制 ====================
    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def exit_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()

    # ==================== 槽函数与控制逻辑 ====================
    def _update_alarm(self, is_alarm):
        if is_alarm != self._alarm_state:     # 状态翻转时才刷新样式, 避免每帧重新 polish
            self._alarm_state = is_alarm
            if is_alarm:
                self.val_alarm.setText("⚠ 振动超限预警")
                self._set_prop(self.val_alarm, "tone", "alarm")
                self._set_prop(self.card_alarm, "alarm", "true")
            else:
                self.val_alarm.setText("正常")
                self._set_prop(self.val_alarm, "tone", "ok")
                self._set_prop(self.card_alarm, "alarm", "false")

    def on_frame_ready(self, qimg_orig, qimg_proc, fps, disp, energy, inst_disp):
        self.video_left.set_image(qimg_orig)
        self.video_right.set_image(qimg_proc)

        self.val_fps.setText(f"{fps:.1f} fps")
        self.val_disp.setText(f"{disp:.2f} px")
        self._update_alarm(disp > ALARM_DISP_THRESHOLD)

        # 始终将瞬态位移压入历史波形队列 (确保波形连贯无间断)
        self.disp_history.append(inst_disp)
        arr = np.array(self.disp_history)

        # 实时高精度 FFT 振动谱分析与主频估计 (以实际实时运行帧率为准)
        if len(arr) >= 32 and disp > 0.05:
            detrended = arr - np.mean(arr)
            fft_vals = np.abs(np.fft.rfft(detrended))
            dt = 1.0 / max(1.0, fps)
            freqs = np.fft.rfftfreq(len(arr), d=dt)
            if len(fft_vals) > 1 and np.max(fft_vals[1:]) > 0.05:
                peak_idx = 1 + np.argmax(fft_vals[1:])
                dom_freq = float(freqs[peak_idx])
            else:
                dom_freq = 0.0

            if self.chart_container.isVisible():
                self.curve_wave.setData(arr)
                self.curve_fft.setData(freqs[1:], fft_vals[1:])
        else:
            dom_freq = 0.0
            if self.chart_container.isVisible():
                self.curve_wave.setData(arr)
                # 静止状态下谱线归零贴底，避免虚假噪点尖峰
                self.curve_fft.setData(np.linspace(0, 15, len(arr) // 2), np.zeros(len(arr) // 2))

        self.val_freq.setText(f"{dom_freq:.1f} Hz")

    def switch_mode(self, mode_idx):
        self.engine.mode = mode_idx
        self._highlight_mode_button(mode_idx)

        mode_names = {
            1: "模式 1: 原画直通",
            2: "模式 2: 运动放大",
            3: "模式 3: 热力图叠加"
        }
        self.lbl_hdr_mode.setText(mode_names.get(mode_idx, "模式: 未知"))

        # 始终锁定开发板 HDMI 物理输出通道为模式 1 (原画直通), 确保采集卡抓取的视频源始终为纯净原始帧,
        # 从而保证左侧【原始视频监控】恒定输出真实一致的原图, 同时避免右侧算法对已被硬件放大的画面进行二次重复叠加
        self.send_serial_cmd("M:1\n")

    def _highlight_mode_button(self, active_mode):
        for idx, btn in enumerate(self.mode_buttons, start=1):
            self._set_prop(btn, "active", "true" if idx == active_mode else "false")

    def toggle_palette(self):
        if self.engine.colormap == COLORMAP_JET:
            self.engine.colormap = COLORMAP_VIRIDIS
            self.btn_palette.setText("🎨 切换调色板: Viridis 翠绿")
            self.send_serial_cmd("C:1\n")
        else:
            self.engine.colormap = COLORMAP_JET
            self.btn_palette.setText("🎨 切换调色板: Jet 彩虹")
            self.send_serial_cmd("C:0\n")

    def on_alpha_changed(self, val):
        self.engine.alpha = val
        self.lbl_alpha_val.setText(f"{val}x")
        self.send_serial_cmd(f"A:{val}\n")

    def on_noise_gate_changed(self, val):
        self.engine.set_noise_gate(val)
        self.lbl_gate_val.setText(str(val))
        self.send_serial_cmd(f"G:{val}\n")

    def select_video_file(self):
        filePath, _ = QFileDialog.getOpenFileName(self, "选择待分析视频文件", "", "视频文件 (*.mp4 *.avi *.mkv *.mov);;所有文件 (*)")
        if filePath:
            self.engine.set_source(filePath)
            name = filePath.split('/')[-1]
            # 文件名过长会把按钮撑宽、挤压其他控件, 这里截断显示, 完整路径放入悬停提示
            self.btn_open_file.setText(name if len(name) <= 20 else name[:17] + "…")
            self.btn_open_file.setToolTip(filePath)

    def start_camera(self):
        # 弹出设备号选择 (0: 本地摄像头, 1: USB-HDMI 采集卡)
        dev_id, ok = QInputDialog.getInt(self, "选择视频捕获设备", "输入设备编号 (0=自带摄像头, 1=HDMI采集卡):", 1, 0, 10, 1)
        if ok:
            self.engine.set_source(dev_id)
            self.btn_camera.setText(f"已激活设备 [{dev_id}]")

    def toggle_pause(self):
        self.engine.paused = not self.engine.paused
        self.btn_pause.setText("▶ 继续播放" if self.engine.paused else "⏸ 暂停")

    # ==================== 串口与硬件全双工通信 ====================
    def connect_serial_dialog(self):
        ports = serial.tools.list_ports.comports()
        if not ports:
            QMessageBox.warning(self, "串口提示", "未找到可用的开发板串口设备，请检查 USB 线连接!")
            return

        port_list = [f"{p.device} - {p.description}" for p in ports]
        item, ok = QInputDialog.getItem(self, "连接 ZYNQ 开发板", "选择开发板串口号:", port_list, 0, False)
        if ok and item:
            port_name = item.split(" ")[0]
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = serial.Serial(port_name, 115200, timeout=0.1)
                self.lbl_hdr_serial.setText(f"● 已连接: {port_name}")
                self._set_prop(self.lbl_hdr_serial, "connected", "true")
                self.btn_serial.setText(f"已连接 [{port_name}]")
                self._set_prop(self.btn_serial, "connected", "true")
                self.serial_rx_timer.start(50)  # 50ms 轮询接收遥测包
                self.send_serial_cmd("M:1\n")   # 连接时自动确保开发板 HDMI 物理输出保持为纯净原画直通
            except Exception as e:
                QMessageBox.critical(self, "连接失败", f"无法打开串口 {port_name}: {str(e)}")

    def send_serial_cmd(self, cmd_str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('ascii'))
            except Exception as e:
                print(f"[Serial Write Error] {e}")

    def read_serial_data(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            # 持续清空串口接收缓冲区并解析硬件返回的文本日志/应答 (如 [ACK] 等), 绝不覆盖上位机真实视频分析数值
            while self.ser.in_waiting > 0:
                self.ser.read(min(self.ser.in_waiting, 512))
        except Exception:
            pass

    def toggle_chart_visibility(self):
        vis = not self.chart_container.isVisible()
        self.chart_container.setVisible(vis)
        self.btn_toggle_chart.setText("📈 隐藏实时振动频谱" if vis else "📈 显示实时振动频谱")
        # 展开图表时让底部面板分得一部分弹性空间, 收起时恢复紧凑
        self.main_layout.setStretchFactor(self.bottom_panel, 2 if vis else 0)

    def closeEvent(self, event):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.engine.stop()
        event.accept()


if __name__ == '__main__':
    # 高分屏 (125% / 150% / 4K) 下保持清晰且比例一致
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    window = EVMAdvancedDashboard()
    window.show()
    sys.exit(app.exec_())
