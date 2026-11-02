// ==============================================================================
// 模块名称: downscale_2x
// 功能描述: 2x2 空间抽样降采样预处理模块 (720P 1280x720 -> 360P 640x360)
// 实现思路: 对 1280x720 输入视频进行 2x 规则空间点抽样 (偶数行、偶数列选通)，
//          无需巨型 RAM 缓存，零逻辑延迟，直接输出严格标准的 640x360 逐行像素流。
// ==============================================================================

`timescale 1ns / 1ps

module downscale_2x #(
    parameter IN_WIDTH  = 1280,
    parameter IN_HEIGHT = 720
)(
    input  wire        clk,
    input  wire        rst_n,
    
    // 输入原始流 (720P: 1280x720)
    input  wire        in_valid,
    input  wire        in_hs,           // 硬件行有效信号 (高电平表示一行有效)
    input  wire        in_vs,           // 场同步信号
    input  wire [7:0]  in_y,
    input  wire [23:0] in_rgb,
    
    // 输出降采样流 (360P: 640x360)
    output reg         out_valid,
    output reg         out_hs,          // 降采样后的行有效信号
    output reg         out_vs,
    output reg  [7:0]  out_y,
    output reg  [23:0] out_rgb
);

    reg [10:0] col_cnt;
    reg [9:0]  row_cnt;
    reg        in_hs_d;

    // 行列计数器 (由 in_hs / in_vs 严格硬件同步锁死)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt <= 11'd0;
            row_cnt <= 10'd0;
            in_hs_d <= 1'b0;
        end else begin
            in_hs_d <= in_hs;

            if (in_vs) begin
                col_cnt <= 11'd0;
                row_cnt <= 10'd0;
            end else if (!in_hs) begin
                // 行消隐期间: 强行将列计数器清零，确保下一行从列 0 开始!
                col_cnt <= 11'd0;
                // in_hs 下降沿 (完成一行): 行计数递增
                if (in_hs_d && !in_hs) begin
                    row_cnt <= (row_cnt == IN_HEIGHT - 1) ? 10'd0 : row_cnt + 1'b1;
                end
            end else if (in_valid) begin
                if (col_cnt == IN_WIDTH - 1) begin
                    col_cnt <= 11'd0;
                end else begin
                    col_cnt <= col_cnt + 1'b1;
                end
            end
        end
    end

    // 2x 抽样流水线输出 (偶数行且偶数列时输出 1 像素)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_hs    <= 1'b0;
            out_vs    <= 1'b0;
            out_y     <= 8'd0;
            out_rgb   <= 24'd0;
        end else begin
            out_vs <= in_vs;
            out_hs <= (row_cnt[0] == 1'b0) && in_hs;

            if (in_valid && (row_cnt[0] == 1'b0) && (col_cnt[0] == 1'b0)) begin
                out_valid <= 1'b1;
                out_y     <= in_y;
                out_rgb   <= in_rgb;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

endmodule
