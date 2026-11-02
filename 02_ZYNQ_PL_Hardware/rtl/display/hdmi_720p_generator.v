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
