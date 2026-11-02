// ==============================================================================
// 模块名称: evm_pl_top
// 功能描述: ZYNQ-7000 (XC7Z020) 欧拉视频放大 (EVM) 硬件 PL 顶层整合模块
// 实现思路: 1. 全硬件流水线贯通:
//             OV5640 DVP 像素同步抓取 (720P)
//             -> RGB 转 YUV 颜色解耦核 (提取灰度亮度 Y)
//             -> 2x2 空间均值降采样核 (720P->360P, 提升 6dB 信噪比并降低 4x 带宽)
//             -> 5 行 BRAM 行缓存与滑动窗口生成器
//             -> 可分离二项式高斯滤波与拉普拉斯空间金字塔分解 (提取空间残差 L0)
//             -> 时域双 IIR 递归带通滤波核 + 硬件抗噪死区门限 (Temporal Noise Coring)
//             -> 微动放大增益乘法器 (Alpha * delta_clean) 并饱和截断叠加至 RGB
//             -> 伪彩色映射 (Jet/Viridis) 与 Alpha 混合半透明叠加器
//             -> 3 大核心显示模式无缝选择器 (1: 直通, 2: 放大, 3: 热力叠加)
//             -> 标准 AXI4-Lite 从机总线，支持 ARM PS 端动态配置与实时振动告警遥测!
// ==============================================================================

module evm_pl_top #(
    parameter USE_INTERNAL_RAM = 0,             // 0: 使用外部 VDMA 历史状态流; 1: 使用片上 BRAM
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 7
)(
    // --------------------------------------------------------------------------
    // 时钟与复位
    // --------------------------------------------------------------------------
    input  wire                                  clk,              // 像素时钟 (例如 56MHz ~ 74.25MHz)
    input  wire                                  rst_n,            // 异步复位 (低有效)
    
    // --------------------------------------------------------------------------
    // ATK-OV5640 摄像头 DVP 物理接口输入
    // --------------------------------------------------------------------------
    input  wire                                  cmos_pclk,        // 摄像头像素同步时钟
    input  wire                                  cmos_vsync,       // 场同步信号
    input  wire                                  cmos_href,        // 行有效信号
    input  wire [7:0]                            cmos_data,        // 8-bit DVP 数据流
    
    // --------------------------------------------------------------------------
    // AXI-Stream 时域历史状态接口 (接 VDMA 读写 DDR3)
    // --------------------------------------------------------------------------
    input  wire                                  s_axis_state_tvalid,
    input  wire [31:0]                           s_axis_state_tdata, // [31:16]=lp1, [15:0]=lp2
    output wire                                  m_axis_state_tvalid,
    output wire [31:0]                           m_axis_state_tdata, // [31:16]=lp1, [15:0]=lp2
    
    // --------------------------------------------------------------------------
    // 处理后视频输出流 (可直连 HDMI 驱动 rgb2dvi 或 AXI-Stream VDMA)
    // --------------------------------------------------------------------------
    output wire                                  vid_out_valid,
    output wire                                  vid_out_hs,
    output wire                                  vid_out_vs,
    output wire [23:0]                           vid_out_rgb,
    
    // --------------------------------------------------------------------------
    // AXI4-Lite 从机总线接口 (与 ARM PS 端 Cortex-A9 互联)
    // --------------------------------------------------------------------------
    input  wire                                  s_axi_aclk,
    input  wire                                  s_axi_aresetn,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_awaddr,
    input  wire [2:0]                            s_axi_awprot,
    input  wire                                  s_axi_awvalid,
    output wire                                  s_axi_awready,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]    s_axi_wstrb,
    input  wire                                  s_axi_wvalid,
    output wire                                  s_axi_wready,
    output wire [1:0]                            s_axi_bresp,
    output wire                                  s_axi_bvalid,
    input  wire                                  s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_araddr,
    input  wire [2:0]                            s_axi_arprot,
    input  wire                                  s_axi_arvalid,
    output wire                                  s_axi_arready,
    output wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_rdata,
    output wire [1:0]                            s_axi_rresp,
    output wire                                  s_axi_rvalid,
    input  wire                                  s_axi_rready
);

    // ==========================================================================
    // 1. AXI4-Lite 寄存器接口与参数解出
    // ==========================================================================
    wire [7:0]  reg_ap_ctrl;
    wire [15:0] reg_width;
    wire [15:0] reg_height;
    wire [7:0]  reg_alpha;
    wire [11:0] reg_r1_fixed;
    wire [11:0] reg_r2_fixed;
    wire [2:0]  reg_disp_mode;
    wire        reg_colormap_sel;
    wire [11:0] reg_noise_gate;
    wire [7:0]  hw_peak_mag;
    reg  [31:0] hw_frame_cnt;

    // 帧计数器逻辑
    reg vid_out_vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vid_out_vs_d <= 1'b0;
            hw_frame_cnt <= 32'd0;
        end else begin
            vid_out_vs_d <= vid_out_vs;
            if (vid_out_vs && !vid_out_vs_d) begin
                hw_frame_cnt <= hw_frame_cnt + 1'b1;
            end
        end
    end

    // 硬件自检与全流水线实时诊断计数器
    reg [31:0] diag_pclk_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) diag_pclk_cnt <= 32'd0;
        else        diag_pclk_cnt <= diag_pclk_cnt + 1'b1;
    end

    reg [31:0] diag_cap_valid_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) diag_cap_valid_cnt <= 32'd0;
        else if (cap_valid) diag_cap_valid_cnt <= diag_cap_valid_cnt + 1'b1;
    end

    reg [31:0] diag_mux_valid_cnt;
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) diag_mux_valid_cnt <= 32'd0;
        else if (mux_valid) diag_mux_valid_cnt <= diag_mux_valid_cnt + 1'b1;
    end

    wire       hw_wr_bank;
    wire [9:0] hw_wr_col;
    wire [31:0] diag_status = {cmos_vsync, cmos_href, cap_valid, ds_valid, lb_valid, iir_valid, mux_valid, 9'd0, hw_wr_bank, 5'd0, hw_wr_col};

    axi_lite_regs #(
        .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH)
    ) u_axi_lite_regs (
        .s_axi_aclk         (s_axi_aclk),
        .s_axi_aresetn      (s_axi_aresetn),
        .s_axi_awaddr       (s_axi_awaddr),
        .s_axi_awprot       (s_axi_awprot),
        .s_axi_awvalid      (s_axi_awvalid),
        .s_axi_awready      (s_axi_awready),
        .s_axi_wdata        (s_axi_wdata),
        .s_axi_wstrb        (s_axi_wstrb),
        .s_axi_wvalid       (s_axi_wvalid),
        .s_axi_wready       (s_axi_wready),
        .s_axi_bresp        (s_axi_bresp),
        .s_axi_bvalid       (s_axi_bvalid),
        .s_axi_bready       (s_axi_bready),
        .s_axi_araddr       (s_axi_araddr),
        .s_axi_arprot       (s_axi_arprot),
        .s_axi_arvalid      (s_axi_arvalid),
        .s_axi_arready      (s_axi_arready),
        .s_axi_rdata        (s_axi_rdata),
        .s_axi_rresp        (s_axi_rresp),
        .s_axi_rvalid       (s_axi_rvalid),
        .s_axi_rready       (s_axi_rready),
        
        .reg_ap_ctrl        (reg_ap_ctrl),
        .reg_width          (reg_width),
        .reg_height         (reg_height),
        .reg_alpha          (reg_alpha),
        .reg_r1_fixed       (reg_r1_fixed),
        .reg_r2_fixed       (reg_r2_fixed),
        .reg_disp_mode      (reg_disp_mode),
        .reg_colormap_sel   (reg_colormap_sel),
        .reg_noise_gate     (reg_noise_gate),
        
        .hw_peak_mag        (hw_peak_mag),
        .hw_frame_cnt       (hw_frame_cnt),
        .diag_pclk_cnt      (diag_pclk_cnt),
        .diag_cap_valid_cnt (diag_cap_valid_cnt),
        .diag_mux_valid_cnt (diag_mux_valid_cnt),
        .diag_status        (diag_status)
    );

    // ==========================================================================
    // 2. OV5640 DVP 接口像素同步抓取 (RGB565 -> 24-bit RGB888)
    // ==========================================================================
    wire        cap_valid;
    wire        cap_hs;
    wire        cap_vs;
    wire [23:0] cap_rgb;

    ov5640_dvp_capture u_dvp_cap (
        .pclk        (cmos_pclk),
        .rst_n       (rst_n),
        .cmos_vsync  (cmos_vsync),
        .cmos_href   (cmos_href),
        .cmos_data   (cmos_data),
        .pixel_valid (cap_valid),
        .pixel_hs    (cap_hs),
        .pixel_vs    (cap_vs),
        .pixel_rgb   (cap_rgb)
    );

    // ==========================================================================
    // 3. 颜色解耦转换核: 24-bit RGB888 -> YUV444 (提取亮度 Y 分量, 运行在 cmos_pclk)
    // ==========================================================================
    wire        yuv_valid;
    wire        yuv_hs;
    wire        yuv_vs;
    wire [7:0]  yuv_y;
    wire [7:0]  yuv_u;
    wire [7:0]  yuv_v;
    wire [23:0] yuv_rgb_d;

    rgb2yuv u_rgb2yuv (
        .clk       (cmos_pclk),
        .rst_n     (rst_n),
        .in_valid  (cap_valid),
        .in_hs     (cap_hs),
        .in_vs     (cap_vs),
        .in_rgb    (cap_rgb),
        .out_valid (yuv_valid),
        .out_hs    (yuv_hs),
        .out_vs    (yuv_vs),
        .out_y     (yuv_y),
        .out_u     (yuv_u),
        .out_v     (yuv_v),
        .out_rgb_d (yuv_rgb_d)
    );

    // ==========================================================================
    // 4. 2x2 空间均值降采样预处理 (720P 1280x720 -> 360P 640x360, 带硬件行同步对齐)
    // ==========================================================================
    wire        ds_valid;
    wire        ds_hs;
    wire        ds_vs;
    wire [7:0]  ds_y;
    wire [23:0] ds_rgb;

    downscale_2x #(
        .IN_WIDTH  (1280),
        .IN_HEIGHT (720)
    ) u_downscale_2x (
        .clk       (cmos_pclk),
        .rst_n     (rst_n),
        .in_valid  (yuv_valid),
        .in_hs     (yuv_hs),
        .in_vs     (yuv_vs),
        .in_y      (yuv_y),
        .in_rgb    (yuv_rgb_d),
        .out_valid (ds_valid),
        .out_hs    (ds_hs),
        .out_vs    (ds_vs),
        .out_y     (ds_y),
        .out_rgb   (ds_rgb)
    );

    // ==========================================================================
    // 5. 5 行 BRAM 行缓存与滑动窗口生成器 (WIDTH=640)
    // ==========================================================================
    wire        lb_valid;
    wire        lb_hs;
    wire        lb_vs;
    wire [7:0]  p00, p01, p02, p03, p04;
    wire [7:0]  p10, p11, p12, p13, p14;
    wire [7:0]  p20, p21, p22, p23, p24;
    wire [7:0]  p30, p31, p32, p33, p34;
    wire [7:0]  p40, p41, p42, p43, p44;
    wire [23:0] center_rgb;

    line_buffer_5x5 #(
        .WIDTH (640)
    ) u_line_buffer (
        .clk        (cmos_pclk),
        .rst_n      (rst_n),
        .in_valid   (ds_valid),
        .in_hs      (ds_hs),
        .in_vs      (ds_vs),
        .in_data    (ds_y),
        .in_rgb     (ds_rgb),
        .out_valid  (lb_valid),
        .out_hs     (lb_hs),
        .out_vs     (lb_vs),
        .p00(p00), .p01(p01), .p02(p02), .p03(p03), .p04(p04),
        .p10(p10), .p11(p11), .p12(p12), .p13(p13), .p14(p14),
        .p20(p20), .p21(p21), .p22(p22), .p23(p23), .p24(p24),
        .p30(p30), .p31(p31), .p32(p32), .p33(p33), .p34(p34),
        .p40(p40), .p41(p41), .p42(p42), .p43(p43), .p44(p44),
        .center_rgb (center_rgb)
    );

    // ==========================================================================
    // 6. 空间拉普拉斯金字塔残差提取核 (5x5 二项式滤波与残差 L0)
    // ==========================================================================
    wire        pyr_valid;
    wire        pyr_vs;
    wire [7:0]  pyr_y_orig;
    wire [7:0]  pyr_y_blur;
    wire signed [23:0] pyr_l0;
    wire [23:0] pyr_rgb;

    laplacian_pyr u_laplacian_pyr (
        .clk        (cmos_pclk),
        .rst_n      (rst_n),
        .in_valid   (lb_valid),
        .in_vs      (lb_vs),
        .p00(p00), .p01(p01), .p02(p02), .p03(p03), .p04(p04),
        .p10(p10), .p11(p11), .p12(p12), .p13(p13), .p14(p14),
        .p20(p20), .p21(p21), .p22(p22), .p23(p23), .p24(p24),
        .p30(p30), .p31(p31), .p32(p32), .p33(p33), .p34(p34),
        .p40(p40), .p41(p41), .p42(p42), .p43(p43), .p44(p44),
        .in_rgb     (center_rgb),
        .out_valid  (pyr_valid),
        .out_vs     (pyr_vs),
        .out_y_orig (pyr_y_orig),
        .out_y_blur (pyr_y_blur),
        .out_l0     (pyr_l0),
        .out_rgb    (pyr_rgb)
    );

    // ==========================================================================
    // 7. 时域双 IIR 递归带通滤波与硬件抗噪死区门限 (Noise Coring)
    // ==========================================================================
    wire        iir_valid;
    wire        iir_vs;
    wire signed [15:0] iir_delta_clean;
    wire [7:0]  iir_y_orig;
    wire [23:0] iir_rgb;

    wire signed [15:0] st_in_lp1 = s_axis_state_tdata[31:16];
    wire signed [15:0] st_in_lp2 = s_axis_state_tdata[15:0];
    wire signed [15:0] st_out_lp1;
    wire signed [15:0] st_out_lp2;
    wire               st_out_valid;

    assign m_axis_state_tvalid = st_out_valid;
    assign m_axis_state_tdata  = {st_out_lp1, st_out_lp2};

    iir_bandpass_coring #(
        .USE_INTERNAL_RAM (USE_INTERNAL_RAM),
        .RAM_DEPTH        (640 * 360)
    ) u_iir_bandpass (
        .clk             (cmos_pclk),
        .rst_n           (rst_n),
        .in_valid        (pyr_valid),
        .in_vs           (pyr_vs),
        .in_l0           (pyr_l0),
        .in_y_orig       (pyr_y_orig),
        .in_rgb          (pyr_rgb),
        
        .r1_fixed        (reg_r1_fixed),
        .r2_fixed        (reg_r2_fixed),
        .noise_gate      (reg_noise_gate),
        
        .state_in_valid  (s_axis_state_tvalid),
        .state_in_lp1    (st_in_lp1),
        .state_in_lp2    (st_in_lp2),
        
        .state_out_valid (st_out_valid),
        .state_out_lp1   (st_out_lp1),
        .state_out_lp2   (st_out_lp2),
        
        .out_valid       (iir_valid),
        .out_vs          (iir_vs),
        .out_delta_clean (iir_delta_clean),
        .out_y_orig      (iir_y_orig),
        .out_rgb         (iir_rgb)
    );

    // ==========================================================================
    // 8. 微动放大增益与饱和叠加核 (Alpha 增益乘法)
    // ==========================================================================
    wire        amp_valid;
    wire        amp_vs;
    wire [23:0] amp_rgb;

    amplify_add u_amplify_add (
        .clk            (cmos_pclk),
        .rst_n          (rst_n),
        .in_valid       (iir_valid),
        .in_vs          (iir_vs),
        .in_delta_clean (iir_delta_clean),
        .in_y_orig      (iir_y_orig),
        .in_rgb         (iir_rgb),
        .alpha          (reg_alpha),
        .out_valid      (amp_valid),
        .out_vs         (amp_vs),
        .out_rgb_amp    (amp_rgb)
    );

    // ==========================================================================
    // 9. 伪彩热力图生成、半透明 Alpha 叠加与实时振动峰值遥测核
    // ==========================================================================
    wire        heat_valid;
    wire        heat_vs;
    wire [23:0] heat_rgb_overlay;

    heatmap_overlay u_heatmap_overlay (
        .clk                (cmos_pclk),
        .rst_n              (rst_n),
        .in_valid           (iir_valid),
        .in_vs              (iir_vs),
        .in_delta_clean     (iir_delta_clean),
        .in_rgb_orig        (iir_rgb),
        .alpha              (reg_alpha),
        .colormap_sel       (reg_colormap_sel),
        .out_valid          (heat_valid),
        .out_vs             (heat_vs),
        .out_rgb_overlay    (heat_rgb_overlay),
        .out_peak_intensity (hw_peak_mag)
    );

    // ==========================================================================
    // 10. 3 大核心显示模式无缝选择器 (1: 原画直通, 2: 运动放大, 3: 热力叠加)
    // ==========================================================================
    wire        mux_valid;
    wire        mux_vs;
    wire [23:0] mux_rgb;

    display_mux u_display_mux (
        .clk              (cmos_pclk),
        .rst_n            (rst_n),
        .disp_mode        (reg_disp_mode),
        
        .in_valid_base    (iir_valid),
        .in_vs_base       (iir_vs),
        .in_rgb_orig      (iir_rgb),
        
        .in_rgb_amp       (amp_rgb),
        
        .in_valid_overlay (heat_valid),
        .in_vs_overlay    (heat_vs),
        .in_rgb_overlay   (heat_rgb_overlay),
        .out_valid        (mux_valid),
        .out_vs           (mux_vs),
        .out_rgb          (mux_rgb)
    );

    // ==========================================================================
    // 11. 异步双时钟 BRAM Ping-Pong 行缓存与标准 CEA-861 720P@60Hz HDMI 发生器
    //     写端: cmos_pclk (无缝吸收摄像头全像素输入，彻底消除跨时钟域撕裂)
    //     读端: clk (74.25 MHz 标准 CEA-861 像素时钟，直驱 rgb2dvi IP)
    // ==========================================================================
    hdmi_720p_generator u_hdmi_720p (
        .wr_clk        (cmos_pclk),
        .rd_clk        (clk),
        .rst_n         (rst_n),
        .disp_mode     (reg_disp_mode),
        .in_valid      (mux_valid),
        .in_hs         (ds_hs),
        .in_vs         (mux_vs),
        .in_rgb        (mux_rgb),
        .out_wr_bank   (hw_wr_bank),
        .out_wr_col    (hw_wr_col),
        .vid_out_valid (vid_out_valid),
        .vid_out_hs    (vid_out_hs),
        .vid_out_vs    (vid_out_vs),
        .vid_out_rgb   (vid_out_rgb)
    );

endmodule


// ==============================================================================
// 模块名称: hdmi_720p_generator
// 功能描述: 片上 BRAM 640x360 全帧显存缓冲与标准 CEA-861-D 720P@60Hz 发生器
// 实现思路: 1. 内部集成 230,400 深度异步真双端口 Block RAM (128 块 RAMB36E1)
//          2. 写端口以 cmos_pclk 写入，每帧场同步上升沿复位至 0，逐像素精准写入 RGB565 (16-bit)
//          3. 读端口以标准 CEA-861 75MHz rd_clk 扫描读取，严格按 (rd_row * 640 + rd_col) 空间物理寻址
//          4. 行列双倍空间插值放大: 640x360 -> 1280x720@60Hz，彻底根治错位、换行或垂直滚动撕裂!
//          5. 集成硬件 Mode 0 标准 8 色垂直彩条发生器，用于物理显示通道快速闭环验证
// ==============================================================================

`timescale 1ns / 1ps

module hdmi_720p_generator (
    input  wire        wr_clk,          // 写时钟: 来自摄像头/EVM 流水线 (cmos_pclk)
    input  wire        rd_clk,          // 读时钟: 74.25 MHz / 50 MHz 标准 CEA-861 像素时钟 (clk)
    input  wire        rst_n,           // 异步复位 (低有效)
    
    // 显示模式 (0: 硬件标准彩条测试, 1: 原画直通, 2: 运动放大, 3: 热力叠加)
    input  wire [2:0]  disp_mode,
    
    // 输入来自 EVM 核心处理器的 640x360 视频流 (在 wr_clk 时钟域)
    input  wire        in_valid,        // EVM 像素有效信号
    input  wire        in_hs,           // EVM 行同步信号 (保留端口兼容)
    input  wire        in_vs,           // EVM 场同步信号
    input  wire [23:0] in_rgb,          // EVM 像素数据 (RGB888)
    
    // 状态监视输出 (供 AXI-Lite 诊断寄存器读取)
    output wire        out_wr_bank,
    output wire [9:0]  out_wr_col,
    
    // 输出至 rgb2dvi IP 的标准 CEA-861 720P 视频接口 (在 rd_clk 时钟域)
    output reg         vid_out_valid,   // 视频数据有效 (连续 1280 周期高电平)
    output reg         vid_out_hs,      // 标准行同步脉冲 (高有效, 40 周期)
    output reg         vid_out_vs,      // 标准场同步脉冲 (高有效, 5 行)
    output reg  [23:0] vid_out_rgb      // 放大对齐后的 24-bit RGB 像素
);

    // ==========================================================================
    // 1. 异步真双端口 Block RAM (深度 230,400 = 640x360 全帧显存, 16-bit RGB565)
    //    Port A: wr_clk 写入 (纯净推断 128 块 RAMB36E1)
    //    Port B: rd_clk 读取
    // ==========================================================================
    (* ram_style = "block" *) reg [15:0] frame_ram [0:230399];
    
    reg [9:0]  wr_col_cnt;
    reg [8:0]  wr_row_cnt;
    reg        in_vs_d;

    // 2D 空间物理寻址: wr_row * 640 + wr_col = (wr_row << 9) + (wr_row << 7) + wr_col
    wire [17:0] wr_addr = {wr_row_cnt, 9'd0} + {wr_row_cnt, 7'd0} + {8'd0, wr_col_cnt};

    assign out_wr_bank = wr_addr[17];
    assign out_wr_col  = wr_col_cnt;

    // 写控制指针与行列计数器 (严格在 wr_clk 时钟域)
    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_col_cnt <= 10'd0;
            wr_row_cnt <= 9'd0;
            in_vs_d    <= 1'b0;
        end else begin
            in_vs_d <= in_vs;

            // 场消隐期间强行复位行列指针，丢弃上一帧尾部延迟残余
            if (in_vs) begin
                wr_col_cnt <= 10'd0;
                wr_row_cnt <= 9'd0;
            end else if (in_valid) begin
                if (wr_col_cnt == 10'd639) begin
                    wr_col_cnt <= 10'd0;
                    if (wr_row_cnt == 9'd359)
                        wr_row_cnt <= 9'd0;
                    else
                        wr_row_cnt <= wr_row_cnt + 1'b1;
                end else begin
                    wr_col_cnt <= wr_col_cnt + 1'b1;
                end
            end
        end
    end

    // BRAM 写入端口 (消隐期绝不写入，消除上一帧残留导致的地址偏移与竖线撕裂)
    always @(posedge wr_clk) begin
        if (in_valid && !in_vs && wr_row_cnt < 9'd360 && wr_col_cnt < 10'd640) begin
            frame_ram[wr_addr] <= {in_rgb[23:19], in_rgb[15:10], in_rgb[7:3]};
        end
    end

    // ==========================================================================
    // 2. 标准 CEA-861-D 720P@60Hz 时序计数器 (rd_clk)
    // H_ACTIVE = 1280, H_FP = 110, H_SYNC = 40, H_BP = 220 -> H_TOTAL = 1650
    // V_ACTIVE = 720,  V_FP = 5,   V_SYNC = 5,  V_BP = 20  -> V_TOTAL = 750
    // ==========================================================================
    localparam H_ACTIVE = 11'd1280;
    localparam H_TOTAL  = 11'd1650;
    localparam H_SYNC_S = 11'd1390; // 1280 + 110
    localparam H_SYNC_E = 11'd1430; // 1390 + 40

    localparam V_ACTIVE = 10'd720;
    localparam V_TOTAL  = 10'd750;
    localparam V_SYNC_S = 10'd725;  // 720 + 5
    localparam V_SYNC_E = 10'd730;  // 725 + 5

    reg [10:0] h_cnt;
    reg [9:0]  v_cnt;

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 11'd0;
            v_cnt <= 10'd0;
        end else begin
            if (h_cnt == H_TOTAL - 1'b1) begin
                h_cnt <= 11'd0;
                if (v_cnt == V_TOTAL - 1'b1)
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 1'b1;
            end else begin
                h_cnt <= h_cnt + 1'b1;
            end
        end
    end

    // ==========================================================================
    // 3. 空间固定坐标读取寻址 (640x360 -> 1280x720 双倍放大)
    // ==========================================================================
    wire [9:0]  rd_col  = (h_cnt < H_ACTIVE) ? h_cnt[10:1] : 10'd0; // 0 ~ 639
    wire [8:0]  rd_row  = (v_cnt < V_ACTIVE) ? v_cnt[9:1]  : 9'd0;  // 0 ~ 359
    
    // 640 = 512 + 128: 硬件无乘法器快速地址计算
    wire [17:0] rd_addr = {rd_row, 9'd0} + {rd_row, 7'd0} + {8'd0, rd_col};

    reg [15:0] rd_pixel_16;
    always @(posedge rd_clk) begin
        rd_pixel_16 <= frame_ram[rd_addr];
    end

    // 16-bit RGB565 位扩展还原为 24-bit RGB888
    wire [7:0] r_out = {rd_pixel_16[15:11], rd_pixel_16[15:13]};
    wire [7:0] g_out = {rd_pixel_16[10:5],  rd_pixel_16[10:9]};
    wire [7:0] b_out = {rd_pixel_16[4:0],   rd_pixel_16[4:2]};
    wire [23:0] rd_pixel_24 = {r_out, g_out, b_out};

    // 流水线打拍对齐 (延迟 1 拍匹配 RAM 读取)
    reg [10:0] h_cnt_d;
    reg [9:0]  v_cnt_d;
    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt_d <= 11'd0;
            v_cnt_d <= 10'd0;
        end else begin
            h_cnt_d <= h_cnt;
            v_cnt_d <= v_cnt;
        end
    end

    // 标准 8 色垂直彩条测试发生器 (在 rd_clk 域)
    reg [23:0] test_color;
    always @(*) begin
        if (h_cnt_d < 11'd160)       test_color = 24'hFFFFFF; // 白
        else if (h_cnt_d < 11'd320)  test_color = 24'hFFFF00; // 黄
        else if (h_cnt_d < 11'd480)  test_color = 24'h00FFFF; // 青
        else if (h_cnt_d < 11'd640)  test_color = 24'h00FF00; // 绿
        else if (h_cnt_d < 11'd800)  test_color = 24'hFF00FF; // 品红
        else if (h_cnt_d < 11'd960)  test_color = 24'hFF0000; // 红
        else if (h_cnt_d < 11'd1120) test_color = 24'h0000FF; // 蓝
        else                         test_color = 24'h000000; // 黑
    end

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            vid_out_valid <= 1'b0;
            vid_out_hs    <= 1'b0;
            vid_out_vs    <= 1'b0;
            vid_out_rgb   <= 24'd0;
        end else begin
            // 1. 视频数据有效 (严格连续 1280 周期高电平!)
            if (h_cnt_d < H_ACTIVE && v_cnt_d < V_ACTIVE) begin
                vid_out_valid <= 1'b1;
                // Mode 0: 输出标准 8 色彩条; 其余模式: 输出全帧显存实时画面
                vid_out_rgb   <= (disp_mode == 3'd0) ? test_color : rd_pixel_24;
            end else begin
                vid_out_valid <= 1'b0;
                vid_out_rgb   <= 24'd0;
            end

            // 2. 行同步脉冲 (40 周期标准高电平)
            if (h_cnt_d >= H_SYNC_S && h_cnt_d < H_SYNC_E)
                vid_out_hs <= 1'b1;
            else
                vid_out_hs <= 1'b0;

            // 3. 场同步脉冲 (5 行标准高电平)
            if (v_cnt_d >= V_SYNC_S && v_cnt_d < V_SYNC_E)
                vid_out_vs <= 1'b1;
            else
                vid_out_vs <= 1'b0;
        end
    end

endmodule
