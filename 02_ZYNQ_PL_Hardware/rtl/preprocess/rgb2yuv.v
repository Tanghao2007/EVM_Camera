// ==============================================================================
// 模块名称: rgb2yuv
// 功能描述: 24-bit RGB888 转 YUV444 颜色空间转换核 (提取灰度亮度 Y)
// 实现思路: 采用三级全流水线乘累加结构:
//          Y = ( 77*R + 150*G +  29*B) >> 8
//          U = (-43*R -  85*G + 128*B) >> 8 + 128
//          V = (128*R - 107*G -  21*B) >> 8 + 128
//          控制信号 (Valid/HS/VS) 同步打拍延迟 3 拍，保持时序严格对齐。
// ==============================================================================

module rgb2yuv (
    input  wire        clk,
    input  wire        rst_n,
    
    // 输入接口
    input  wire        in_valid,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire [23:0] in_rgb,           // [23:16]=R, [15:8]=G, [7:0]=B
    
    // 输出接口
    output reg         out_valid,
    output reg         out_hs,
    output reg         out_vs,
    output reg  [7:0]  out_y,            // 核心亮度分量 (供 EVM 算法处理)
    output reg  [7:0]  out_u,
    output reg  [7:0]  out_v,
    output reg  [23:0] out_rgb_d         // 对齐延迟后的原始 RGB (供原图叠加混合使用)
);

    // 第一级流水线: 乘法
    reg [15:0] mult_r_y, mult_g_y, mult_b_y;
    reg [15:0] mult_r_u, mult_g_u, mult_b_u;
    reg [15:0] mult_r_v, mult_g_v, mult_b_v;
    reg [23:0] rgb_d1;
    reg        valid_d1, hs_d1, vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_r_y <= 16'd0; mult_g_y <= 16'd0; mult_b_y <= 16'd0;
            mult_r_u <= 16'd0; mult_g_u <= 16'd0; mult_b_u <= 16'd0;
            mult_r_v <= 16'd0; mult_g_v <= 16'd0; mult_b_v <= 16'd0;
            rgb_d1   <= 24'd0;
            valid_d1 <= 1'b0;  hs_d1 <= 1'b0;     vs_d1 <= 1'b0;
        end else begin
            mult_r_y <= in_rgb[23:16] * 8'd77;
            mult_g_y <= in_rgb[15:8]  * 8'd150;
            mult_b_y <= in_rgb[7:0]   * 8'd29;

            mult_r_u <= in_rgb[23:16] * 8'd43;
            mult_g_u <= in_rgb[15:8]  * 8'd85;
            mult_b_u <= in_rgb[7:0]   * 8'd128;

            mult_r_v <= in_rgb[23:16] * 8'd128;
            mult_g_v <= in_rgb[15:8]  * 8'd107;
            mult_b_v <= in_rgb[7:0]   * 8'd21;

            rgb_d1   <= in_rgb;
            valid_d1 <= in_valid;
            hs_d1    <= in_hs;
            vs_d1    <= in_vs;
        end
    end

    // 第二级流水线: 累加求和
    reg [17:0] sum_y;
    reg [17:0] sum_u;
    reg [17:0] sum_v;
    reg [23:0] rgb_d2;
    reg        valid_d2, hs_d2, vs_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_y    <= 18'd0;
            sum_u    <= 18'd0;
            sum_v    <= 18'd0;
            rgb_d2   <= 24'd0;
            valid_d2 <= 1'b0; hs_d2 <= 1'b0; vs_d2 <= 1'b0;
        end else begin
            sum_y    <= mult_r_y + mult_g_y + mult_b_y;
            sum_u    <= mult_b_u - mult_r_u - mult_g_u + 18'd32768; // +128<<8
            sum_v    <= mult_r_v - mult_g_v - mult_b_v + 18'd32768;

            rgb_d2   <= rgb_d1;
            valid_d2 <= valid_d1;
            hs_d2    <= hs_d1;
            vs_d2    <= vs_d1;
        end
    end

    // 第三级流水线: 移位取高 8 位并输出
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_hs    <= 1'b0;
            out_vs    <= 1'b0;
            out_y     <= 8'd0;
            out_u     <= 8'd0;
            out_v     <= 8'd0;
            out_rgb_d <= 24'd0;
        end else begin
            out_valid <= valid_d2;
            out_hs    <= hs_d2;
            out_vs    <= vs_d2;
            out_y     <= sum_y[15:8];
            out_u     <= sum_u[15:8];
            out_v     <= sum_v[15:8];
            out_rgb_d <= rgb_d2;
        end
    end

endmodule
