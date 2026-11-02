// ==============================================================================
// 模块名称: heatmap_overlay
// 功能描述: 微动能量场提取、伪彩色映射、半透明 Alpha 叠加与峰值遥测核
// 实现思路: 1. 提取去噪后位移量绝对值并定点缩放至 0~255 能量标量:
//             mag_scaled = (|delta_clean| * alpha) >> 9
//          2. 调用 colormap_lut 生成 Jet / Viridis 热力图 RGB;
//          3. Alpha 动态权重混合:
//             blend_w = (mag_u8 * 180) >> 8 (最大不透明度约 70%, 底图清晰可见)
//             RGB_overlay = ((256 - blend_w)*RGB_orig + blend_w*RGB_heat) >> 8
//          4. 硬件统计全帧最大微动峰值 out_peak_intensity，直连 AXI-Lite 供上位机
//             振动告警 (VIBRATION ALARM) 与频谱位移遥测使用!
// ==============================================================================

module heatmap_overlay (
    input  wire        clk,
    input  wire        rst_n,
    
    // 输入流
    input  wire        in_valid,
    input  wire        in_vs,
    input  wire signed [15:0] in_delta_clean, // 去噪后位移 (Q4.12)
    input  wire [23:0] in_rgb_orig,           // 原始像素 RGB
    
    // 参数配置
    input  wire [7:0]  alpha,
    input  wire        colormap_sel,          // 0: Jet, 1: Viridis
    
    // 输出叠加流
    output reg         out_valid,
    output reg         out_vs,
    output reg  [23:0] out_rgb_overlay,
    
    // 实时振动峰值幅值 (供 PS 端与上位机 VIBRATION ALARM 遥测)
    output reg  [7:0]  out_peak_intensity
);

    // --------------------------------------------------------------------------
    // 第一级流水线: 幅度标量计算与缩放
    // --------------------------------------------------------------------------
    wire [15:0] abs_clean = (in_delta_clean < 0) ? -in_delta_clean : in_delta_clean;
    wire [23:0] mult_mag  = abs_clean * alpha;
    wire [14:0] mag_shift = mult_mag[23:9]; // 右移 9 位 (12 - 3)

    reg [7:0]  mag_u8;
    reg [23:0] rgb_orig_d1;
    reg        valid_d1, vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_u8      <= 8'd0;
            rgb_orig_d1 <= 24'd0;
            valid_d1    <= 1'b0;
            vs_d1       <= 1'b0;
        end else begin
            valid_d1    <= in_valid;
            vs_d1       <= in_vs;
            rgb_orig_d1 <= in_rgb_orig;

            if (in_valid) begin
                mag_u8 <= (mag_shift > 15'd255) ? 8'd255 : mag_shift[7:0];
            end else begin
                mag_u8 <= 8'd0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第二级流水线: 查表获取伪彩色热力图
    // --------------------------------------------------------------------------
    wire [7:0] r_heat_w, g_heat_w, b_heat_w;

    colormap_lut u_lut (
        .clk          (clk),
        .rst_n        (rst_n),
        .colormap_sel (colormap_sel),
        .val          (mag_u8),
        .r_heat       (r_heat_w),
        .g_heat       (g_heat_w),
        .b_heat       (b_heat_w)
    );

    reg [7:0]  mag_d2;
    reg [23:0] rgb_orig_d2;
    reg        valid_d2, vs_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_d2      <= 8'd0;
            rgb_orig_d2 <= 24'd0;
            valid_d2    <= 1'b0;
            vs_d2       <= 1'b0;
        end else begin
            valid_d2    <= valid_d1;
            vs_d2       <= vs_d1;
            mag_d2      <= mag_u8;
            rgb_orig_d2 <= rgb_orig_d1;
        end
    end

    // --------------------------------------------------------------------------
    // 第三级流水线: Alpha 混合加权乘法
    // blend_w = (mag_d2 * 180) >> 8
    // --------------------------------------------------------------------------
    wire [15:0] blend_prod = mag_d2 * 8'd180;
    wire [8:0]  blend_w    = blend_prod[15:8];     // 0~180
    wire [8:0]  inv_w      = 9'd256 - blend_w;     // 256~76

    reg [16:0] sum_r, sum_g, sum_b;
    reg        valid_d3, vs_d3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_r    <= 17'd0;
            sum_g    <= 17'd0;
            sum_b    <= 17'd0;
            valid_d3 <= 1'b0;
            vs_d3    <= 1'b0;
        end else begin
            valid_d3 <= valid_d2;
            vs_d3    <= vs_d2;

            if (valid_d2) begin
                sum_r <= (inv_w * rgb_orig_d2[23:16]) + (blend_w * r_heat_w);
                sum_g <= (inv_w * rgb_orig_d2[15:8])  + (blend_w * g_heat_w);
                sum_b <= (inv_w * rgb_orig_d2[7:0])   + (blend_w * b_heat_w);
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第四级流水线: 移位取高 8 位并输出叠加像素
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid       <= 1'b0;
            out_vs          <= 1'b0;
            out_rgb_overlay <= 24'd0;
        end else begin
            out_valid <= valid_d3;
            out_vs    <= vs_d3;

            if (valid_d3) begin
                out_rgb_overlay[23:16] <= sum_r[15:8];
                out_rgb_overlay[15:8]  <= sum_g[15:8];
                out_rgb_overlay[7:0]   <= sum_b[15:8];
            end else begin
                out_rgb_overlay <= 24'd0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 帧峰值微动强度统计 (用于 VIBRATION ALARM 遥测)
    // --------------------------------------------------------------------------
    reg [7:0] frame_max_mag;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_max_mag      <= 8'd0;
            out_peak_intensity <= 8'd0;
        end else if (in_vs) begin
            out_peak_intensity <= frame_max_mag;
            frame_max_mag      <= 8'd0;
        end else if (in_valid) begin
            if (mag_u8 > frame_max_mag)
                frame_max_mag <= mag_u8;
        end
    end

endmodule
