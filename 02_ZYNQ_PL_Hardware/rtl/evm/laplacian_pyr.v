// ==============================================================================
// 模块名称: laplacian_pyr
// 功能描述: 5x5 可分离二项式高斯模糊与空间拉普拉斯金字塔残差提取核
// 实现思路: 二维 5x5 高斯核 [1,4,6,4,1] 展开归一化分母为 256。
//          由于其数学可分离性，采用两级全流水线架构（无任何乘法器，纯移位加法）:
//          第一级: 5 行水平分别做 [1,4,6,4,1] 滤波
//          第二级: 对 5 个中间列结果做垂直 [1,4,6,4,1] 滤波并右移 8 位得到 Y_blur
//          第三级: 计算高频空间残差 L0 = (Y_orig - Y_blur) << 12 (Q4.12 定点格式)
// ==============================================================================

module laplacian_pyr (
    input  wire        clk,
    input  wire        rst_n,
    
    // 来自 line_buffer_5x5 的滑动窗口与控制信号
    input  wire        in_valid,
    input  wire        in_vs,
    input  wire [7:0]  p00, p01, p02, p03, p04,
    input  wire [7:0]  p10, p11, p12, p13, p14,
    input  wire [7:0]  p20, p21, p22, p23, p24, // p22 为中心像素
    input  wire [7:0]  p30, p31, p32, p33, p34,
    input  wire [7:0]  p40, p41, p42, p43, p44,
    input  wire [23:0] in_rgb,                  // 严格对齐 p22 的原图 RGB
    
    // 输出流水线信号
    output reg         out_valid,
    output reg         out_vs,
    output reg  [7:0]  out_y_orig,              // 与输出对齐的原图中心灰度 Y
    output reg  [7:0]  out_y_blur,              // 5x5 高斯低通基准 Y_blur
    output reg  signed [23:0] out_l0,           // 空间高频残差 L0 (Q4.12 符号数)
    output reg  [23:0] out_rgb                  // 与输出对齐的原图 RGB
);

    // --------------------------------------------------------------------------
    // 第一级流水线: 5 行水平无乘法滤波
    // 权重: 1*x0 + 4*x1 + 6*x2 + 4*x3 + 1*x4
    // --------------------------------------------------------------------------
    reg [11:0] h_sum0, h_sum1, h_sum2, h_sum3, h_sum4;
    reg [7:0]  p22_d1;
    reg [23:0] rgb_d1;
    reg        valid_d1, vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_sum0   <= 12'd0;
            h_sum1   <= 12'd0;
            h_sum2   <= 12'd0;
            h_sum3   <= 12'd0;
            h_sum4   <= 12'd0;
            p22_d1   <= 8'd0;
            rgb_d1   <= 24'd0;
            valid_d1 <= 1'b0;
            vs_d1    <= 1'b0;
        end else begin
            valid_d1 <= in_valid;
            vs_d1    <= in_vs;
            p22_d1   <= p22;
            rgb_d1   <= in_rgb;

            if (in_valid) begin
                // 行 0 水平滤波
                h_sum0 <= {4'd0, p00} + ({2'd0, p01, 2'd0}) + 
                          ({2'd0, p02, 2'd0} + {3'd0, p02, 1'd0}) + 
                          ({2'd0, p03, 2'd0}) + {4'd0, p04};

                // 行 1 水平滤波
                h_sum1 <= {4'd0, p10} + ({2'd0, p11, 2'd0}) + 
                          ({2'd0, p12, 2'd0} + {3'd0, p12, 1'd0}) + 
                          ({2'd0, p13, 2'd0}) + {4'd0, p14};

                // 行 2 水平滤波 (中心行)
                h_sum2 <= {4'd0, p20} + ({2'd0, p21, 2'd0}) + 
                          ({2'd0, p22, 2'd0} + {3'd0, p22, 1'd0}) + 
                          ({2'd0, p23, 2'd0}) + {4'd0, p24};

                // 行 3 水平滤波
                h_sum3 <= {4'd0, p30} + ({2'd0, p31, 2'd0}) + 
                          ({2'd0, p32, 2'd0} + {3'd0, p32, 1'd0}) + 
                          ({2'd0, p33, 2'd0}) + {4'd0, p34};

                // 行 4 水平滤波
                h_sum4 <= {4'd0, p40} + ({2'd0, p41, 2'd0}) + 
                          ({2'd0, p42, 2'd0} + {3'd0, p42, 1'd0}) + 
                          ({2'd0, p43, 2'd0}) + {4'd0, p44};
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第二级流水线: 垂直方向无乘法滤波与截断四舍五入
    // 权重: 1*h0 + 4*h1 + 6*h2 + 4*h3 + 1*h4
    // --------------------------------------------------------------------------
    reg [16:0] v_sum;
    reg [7:0]  p22_d2;
    reg [23:0] rgb_d2;
    reg        valid_d2, vs_d2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_sum    <= 17'd0;
            p22_d2   <= 8'd0;
            rgb_d2   <= 24'd0;
            valid_d2 <= 1'b0;
            vs_d2    <= 1'b0;
        end else begin
            valid_d2 <= valid_d1;
            vs_d2    <= vs_d1;
            p22_d2   <= p22_d1;
            rgb_d2   <= rgb_d1;

            if (valid_d1) begin
                v_sum <= {5'd0, h_sum0} + ({3'd0, h_sum1, 2'd0}) +
                         ({3'd0, h_sum2, 2'd0} + {4'd0, h_sum2, 1'd0}) +
                         ({3'd0, h_sum3, 2'd0}) + {5'd0, h_sum4};
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第三级流水线: 计算空间残差 L0 并定点化左移 12 位
    // L0 = (Y_orig - Y_blur) << 12
    // --------------------------------------------------------------------------
    wire [7:0] y_blur_calc = (v_sum + 17'd128) >> 8; // 加 128 实现四舍五入
    wire signed [9:0] diff_spatial = $signed({2'b00, p22_d2}) - $signed({2'b00, y_blur_calc});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid  <= 1'b0;
            out_vs     <= 1'b0;
            out_y_orig <= 8'd0;
            out_y_blur <= 8'd0;
            out_l0     <= 24'sd0;
            out_rgb    <= 24'd0;
        end else begin
            out_valid  <= valid_d2;
            out_vs     <= vs_d2;
            out_y_orig <= p22_d2;
            out_y_blur <= y_blur_calc;
            out_rgb    <= rgb_d2;

            if (valid_d2) begin
                out_l0 <= diff_spatial <<< 12; // 扩展为 Q4.12 定点符号数
            end else begin
                out_l0 <= 24'sd0;
            end
        end
    end

endmodule
