// ==============================================================================
// 模块名称: iir_bandpass_coring
// 功能描述: 时域双 IIR 递归带通滤波核与硬件抗噪死区门限 (Noise Coring) 处理器
// 实现思路: 1. 采用两个并行的指数一阶低通滤波器提取高低频截止基准:
//             new_lp1 = ((4096 - r1)*prev_lp1 + r1*L0) >> 12
//             new_lp2 = ((4096 - r2)*prev_lp2 + r2*L0) >> 12
//          2. 差分提取带通微动量: delta_y = new_lp1 - new_lp2
//          3. 硬件软阈值抗噪死区 (Temporal Noise Coring):
//             当 |delta_y| <= noise_gate 时强行清零，剔除 CMOS 传感器暗电流与散粒白噪;
//             当 |delta_y| >  noise_gate 时保留真实物理微动 (扣除噪声基底)
//          4. 既支持 AXI-Stream/VDMA 外部 DDR3 历史状态流，也支持片上 RAM 仿真/独立模式
// ==============================================================================

module iir_bandpass_coring #(
    parameter USE_INTERNAL_RAM = 0,         // 0: 使用外部 VDMA/DDR3 状态流; 1: 使用片上 BRAM
    parameter RAM_DEPTH        = 640 * 360  // 片上 RAM 深度 (降采样 360P 规格)
)(
    input  wire        clk,
    input  wire        rst_n,
    
    // 输入空间残差与像素流
    input  wire        in_valid,
    input  wire        in_vs,
    input  wire signed [23:0] in_l0,        // 空间高频残差 L0 (Q4.12)
    input  wire [7:0]  in_y_orig,           // 原始亮度 Y
    input  wire [23:0] in_rgb,              // 原始像素 RGB
    
    // PS 端动态配置寄存器
    input  wire [11:0] r1_fixed,            // 因子 r1 (Q12, 默认 1638 对应 0.4)
    input  wire [11:0] r2_fixed,            // 因子 r2 (Q12, 默认 205  对应 0.05)
    input  wire [11:0] noise_gate,          // 抗噪死区门限 (Q12, 默认 35)
    
    // 外部时域历史状态接口 (接 VDMA / AXI-Stream)
    input  wire        state_in_valid,
    input  wire signed [15:0] state_in_lp1,
    input  wire signed [15:0] state_in_lp2,
    
    output reg         state_out_valid,
    output reg  signed [15:0] state_out_lp1,
    output reg  signed [15:0] state_out_lp2,
    
    // 滤波与降噪后输出流
    output reg         out_valid,
    output reg         out_vs,
    output reg  signed [15:0] out_delta_clean, // 去除噪声后的纯净微动位移量 (Q4.12)
    output reg  [7:0]  out_y_orig,
    output reg  [23:0] out_rgb
);

    // --------------------------------------------------------------------------
    // 历史状态读取仲裁 (外部 VDMA 流 或 片上 RAM)
    // --------------------------------------------------------------------------
    reg [18:0] pixel_addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_addr <= 19'd0;
        end else if (in_vs) begin
            pixel_addr <= 19'd0;
        end else if (in_valid) begin
            pixel_addr <= (pixel_addr == RAM_DEPTH - 1) ? 19'd0 : pixel_addr + 1'b1;
        end
    end

    wire signed [15:0] hist_lp1;
    wire signed [15:0] hist_lp2;

    generate
        if (USE_INTERNAL_RAM == 1) begin : gen_internal_ram
            reg signed [15:0] ram_lp1 [0:RAM_DEPTH-1];
            reg signed [15:0] ram_lp2 [0:RAM_DEPTH-1];
            reg signed [15:0] rdata_lp1, rdata_lp2;

            always @(posedge clk) begin
                if (in_valid) begin
                    rdata_lp1 <= ram_lp1[pixel_addr];
                    rdata_lp2 <= ram_lp2[pixel_addr];
                end
            end

            assign hist_lp1 = rdata_lp1;
            assign hist_lp2 = rdata_lp2;

            // 写入更新
            always @(posedge clk) begin
                if (state_out_valid) begin
                    ram_lp1[pixel_addr] <= state_out_lp1;
                    ram_lp2[pixel_addr] <= state_out_lp2;
                end
            end
        end else begin : gen_external_axis
            assign hist_lp1 = state_in_valid ? state_in_lp1 : 16'sd0;
            assign hist_lp2 = state_in_valid ? state_in_lp2 : 16'sd0;
        end
    endgenerate

    // --------------------------------------------------------------------------
    // 第一级流水线: DSP 乘法
    // --------------------------------------------------------------------------
    wire signed [15:0] one_minus_r1 = $signed({4'd0, (13'd4096 - {1'b0, r1_fixed})});
    wire signed [15:0] one_minus_r2 = $signed({4'd0, (13'd4096 - {1'b0, r2_fixed})});
    wire signed [15:0] r1_signed     = $signed({4'd0, r1_fixed});
    wire signed [15:0] r2_signed     = $signed({4'd0, r2_fixed});

    reg signed [31:0] p_lp1_old;
    reg signed [35:0] p_lp1_new;
    reg signed [31:0] p_lp2_old;
    reg signed [35:0] p_lp2_new;

    reg [7:0]  y_orig_d1;
    reg [23:0] rgb_d1;
    reg        valid_d1, vs_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_lp1_old <= 32'sd0; p_lp1_new <= 36'sd0;
            p_lp2_old <= 32'sd0; p_lp2_new <= 36'sd0;
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
                p_lp1_old <= one_minus_r1 * hist_lp1;
                p_lp1_new <= r1_signed     * in_l0;

                p_lp2_old <= one_minus_r2 * hist_lp2;
                p_lp2_new <= r2_signed     * in_l0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第二级流水线: 累加、移位与差分提取 DeltaY
    // --------------------------------------------------------------------------
    reg signed [15:0] next_lp1, next_lp2;
    reg signed [16:0] delta_raw;
    reg [7:0]  y_orig_d2;
    reg [23:0] rgb_d2;
    reg        valid_d2, vs_d2;

    wire signed [36:0] sum_lp1 = {p_lp1_old[31], p_lp1_old, 4'd0} + p_lp1_new; // 对齐 Q12
    wire signed [36:0] sum_lp2 = {p_lp2_old[31], p_lp2_old, 4'd0} + p_lp2_new;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            next_lp1  <= 16'sd0;
            next_lp2  <= 16'sd0;
            delta_raw <= 17'sd0;
            y_orig_d2 <= 8'd0;
            rgb_d2    <= 24'd0;
            valid_d2  <= 1'b0;
            vs_d2     <= 1'b0;
        end else begin
            valid_d2  <= valid_d1;
            vs_d2     <= vs_d1;
            y_orig_d2 <= y_orig_d1;
            rgb_d2    <= rgb_d1;

            if (valid_d1) begin
                next_lp1  <= sum_lp1[27:12]; // 右移 12 位定点截断
                next_lp2  <= sum_lp2[27:12];
                delta_raw <= $signed(sum_lp1[27:12]) - $signed(sum_lp2[27:12]);
            end
        end
    end

    // --------------------------------------------------------------------------
    // 第三级流水线: 硬件时域死区软阈值降噪 (Temporal Noise Coring)
    // --------------------------------------------------------------------------
    wire [15:0] abs_delta = (delta_raw < 0) ? -delta_raw[15:0] : delta_raw[15:0];
    wire [15:0] gate_ext  = {4'd0, noise_gate};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid       <= 1'b0;
            out_vs          <= 1'b0;
            out_delta_clean <= 16'sd0;
            out_y_orig      <= 8'd0;
            out_rgb         <= 24'd0;
            state_out_valid <= 1'b0;
            state_out_lp1   <= 16'sd0;
            state_out_lp2   <= 16'sd0;
        end else begin
            out_valid       <= valid_d2;
            out_vs          <= vs_d2;
            out_y_orig      <= y_orig_d2;
            out_rgb         <= rgb_d2;
            state_out_valid <= valid_d2;
            state_out_lp1   <= next_lp1;
            state_out_lp2   <= next_lp2;

            if (valid_d2) begin
                // 死区门限滤波
                if (abs_delta > gate_ext) begin
                    if (delta_raw > 0)
                        out_delta_clean <= $signed(abs_delta - gate_ext);
                    else
                        out_delta_clean <= -$signed(abs_delta - gate_ext);
                end else begin
                    out_delta_clean <= 16'sd0; // 小于门限的噪声彻底归零
                end
            end else begin
                out_delta_clean <= 16'sd0;
            end
        end
    end

endmodule
