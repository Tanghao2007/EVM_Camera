// ==============================================================================
// 模块名称: ov5640_dvp_capture
// 功能描述: ATK-OV5640 摄像头 8-bit DVP 数字接口像素抓取与 RGB888 解包模块
// 实现思路: OV5640 输出 RGB565 格式(每像素占用 2 个 PCLK: 高字节在前, 低字节在后)。
//          本模块将 2 个连续字节严格对齐拼接为完整 16-bit RGB565 像素，并位扩展为 24-bit RGB888。
// ==============================================================================

`timescale 1ns / 1ps

module ov5640_dvp_capture (
    input  wire        pclk,             // 摄像头像素同步时钟 (约 56MHz ~ 84MHz)
    input  wire        rst_n,            // 异步复位信号 (低电平有效)
    input  wire        cmos_vsync,       // 场同步信号 (高电平有效)
    input  wire        cmos_href,        // 行有效信号 (高电平有效)
    input  wire [7:0]  cmos_data,        // 8-bit DVP 数据输入
    
    output reg         pixel_valid,      // 像素有效标志脉冲 (每 2 个 PCLK 脉冲 1 拍)
    output reg         pixel_hs,         // 像素行有效信号
    output reg         pixel_vs,         // 像素场同步信号
    output reg  [23:0] pixel_rgb         // 解包后的 24-bit RGB888 像素输出
);

    reg [7:0] data_r;
    reg       href_r;
    reg       vsync_r;
    reg       byte_cnt;
    reg [7:0] high_byte;

    // 输入同步打拍
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            data_r  <= 8'd0;
            href_r  <= 1'b0;
            vsync_r <= 1'b0;
        end else begin
            data_r  <= cmos_data;
            href_r  <= cmos_href;
            vsync_r <= cmos_vsync;
        end
    end

    // 标准双字节组装状态机
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt    <= 1'b0;
            high_byte   <= 8'd0;
            pixel_valid <= 1'b0;
            pixel_hs    <= 1'b0;
            pixel_vs    <= 1'b0;
            pixel_rgb   <= 24'd0;
        end else if (vsync_r) begin
            // 场同步期间强行清零
            byte_cnt    <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_hs    <= 1'b0;
            pixel_vs    <= 1'b1;
        end else if (href_r) begin
            pixel_vs <= 1'b0;
            pixel_hs <= 1'b1;
            byte_cnt <= ~byte_cnt;

            if (byte_cnt == 1'b0) begin
                // 第 1 字节到来: 寄存高字节 (R[4:0], G[5:3])
                high_byte   <= data_r;
                pixel_valid <= 1'b0;
            end else begin
                // 第 2 字节到来: 组装当前完整 16-bit RGB565 像素并位扩展为 24-bit RGB888
                pixel_valid <= 1'b1;
                // Red: 高字节 [7:3] -> 扩展为 8-bit
                pixel_rgb[23:16] <= {high_byte[7:3], high_byte[7:5]};
                // Green: 高字节 [2:0] 与 低字节 [7:5] -> 扩展为 8-bit
                pixel_rgb[15:8]  <= {high_byte[2:0], data_r[7:5], high_byte[2:1]};
                // Blue: 低字节 [4:0] -> 扩展为 8-bit
                pixel_rgb[7:0]   <= {data_r[4:0], data_r[4:2]};
            end
        end else begin
            byte_cnt    <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_hs    <= 1'b0;
            pixel_vs    <= 1'b0;
        end
    end

endmodule
