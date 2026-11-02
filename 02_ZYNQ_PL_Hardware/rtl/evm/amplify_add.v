// ==============================================================================
// 模块名称: amplify_add
// 功能描述: 微动放大增益乘法器与 RGB 亮度自适应叠加饱和截断器
// 实现思路: 1. 采用定点乘法对纯净微动位移量进行增益放大:
//             amp_val = (alpha * delta_clean) >>> 12
//          2. 将 amp_val 叠加至原始亮度分量 Y_orig，并饱和截断至 [0, 255] 得到 Y_out:
//             delta_bright = Y_out - Y_orig
//          3. 将亮度增量保持色彩比例线性叠加至 R, G, B 各通道并做防溢出 Clamp
// ==============================================================================

module amplify_add (
    input  wire        clk,
    input  wire        rst_n,
    
    // 输入流 (来自 iir_bandpass_coring)
    input  wire        in_valid,
    input  wire        in_vs,
    input  wire signed [15:0] in_delta_clean, // 去噪后纯净微动量 (Q4.12)
    input  wire [7:0]  in_y_orig,             // 原始亮度 Y
    input  wire [23:0] in_rgb,                // 原始像素 RGB
    
    // 放大倍数 Alpha (0~150)
    input  wire [7:0]  alpha,
    
    // 输出放大后的 RGB 流
    output reg         out_valid,
    output reg         out_vs,
    output reg  [23:0] out_rgb_amp
);

    // --------------------------------------------------------------------------
    // 第一级流水线: 乘法器计算放大值
    // --------------------------------------------------------------------------
    wire signed [24:0] mult_amp = $signed({1'b0, alpha}) * in_delta_clean;

    reg signed [12:0] amp_int;
    reg [7:0]         y_orig_d1;
    reg [23:0]        rgb_d1;
    reg               valid_d1, vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            amp_int   <= 13'sd0;
            y_orig_d1 <= 8'd0;
            rgb_d1    <= 24'd0;
            valid_d1  <= 1'b0;
            vs_d1     <= 1'b0;
        end else begin
            valid_d1  <= in_valid;
            vs_d1     <= in_vs;
            y_orig_d1 <= in_y_orig;
            rgb_d1    <= in_rgb;

            if (in_valid) begin
                amp_int <= mult_amp[24:12]; // 定点数右移 12 位
            end else begin
                amp_int <= 13'sd0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第二级流水线: 计算亮度变化量并叠加至 RGB 三通道截断输出
    // --------------------------------------------------------------------------
    wire signed [13:0] y_amp_calc = $signed({6'd0, y_orig_d1}) + amp_int;
    wire [7:0] y_clamped = (y_amp_calc > 14'sd255) ? 8'd255 :
                           ((y_amp_calc < 14'sd0)   ? 8'd0   : y_amp_calc[7:0]);

    wire signed [9:0] delta_bright = $signed({2'b00, y_clamped}) - $signed({2'b00, y_orig_d1});

    wire signed [10:0] r_calc = $signed({3'd0, rgb_d1[23:16]}) + delta_bright;
    wire signed [10:0] g_calc = $signed({3'd0, rgb_d1[15:8]})  + delta_bright;
    wire signed [10:0] b_calc = $signed({3'd0, rgb_d1[7:0]})   + delta_bright;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid   <= 1'b0;
            out_vs      <= 1'b0;
            out_rgb_amp <= 24'd0;
        end else begin
            out_valid <= valid_d1;
            out_vs    <= vs_d1;

            if (valid_d1) begin
                // R 通道饱和截断
                if (r_calc > 11'sd255)
                    out_rgb_amp[23:16] <= 8'd255;
                else if (r_calc < 11'sd0)
                    out_rgb_amp[23:16] <= 8'd0;
                else
                    out_rgb_amp[23:16] <= r_calc[7:0];

                // G 通道饱和截断
                if (g_calc > 11'sd255)
                    out_rgb_amp[15:8]  <= 8'd255;
                else if (g_calc < 11'sd0)
                    out_rgb_amp[15:8]  <= 8'd0;
                else
                    out_rgb_amp[15:8]  <= g_calc[7:0];

                // B 通道饱和截断
                if (b_calc > 11'sd255)
                    out_rgb_amp[7:0]   <= 8'd255;
                else if (b_calc < 11'sd0)
                    out_rgb_amp[7:0]   <= 8'd0;
                else
                    out_rgb_amp[7:0]   <= b_calc[7:0];
            end else begin
                out_rgb_amp <= 24'd0;
            end
        end
    end

endmodule
