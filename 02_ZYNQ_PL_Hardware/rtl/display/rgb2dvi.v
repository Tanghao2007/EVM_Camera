// ==============================================================================
// 模块名称: rgb2dvi
// 功能描述: 纯 Verilog 硬件 HDMI/DVI 视频发射机模块 (完美替代 Digilent rgb2dvi IP)
// 实现思路: 1. 兼容 DVI 1.0 标准 TMDS 8b/10b 编码算法 (直流平衡与跳变最小化)
//          2. 内部集成 Xilinx 7 系列 PLLE2 时钟生成器 (自适应 5x 串行时钟，无需外部 Clocking Wizard)
//          3. 原生例化 7 系列专用 ODDR 触发器与 OBUFDS 差分驱动原语，直驱 HDMI TMDS 引脚
//          4. 对接 evm_pl_top 的 vid_out_* 接口，支持直接通过 Vivado BD "Add Module" 引入
// ==============================================================================

`timescale 1ns / 1ps

module rgb2dvi #(
    parameter ENABLE_CLK_GEN = 1  // 1: 内部自生成 5x 串行时钟 (50MHz->250MHz); 0: 外部输入 SerialClk
)(
    // --------------------------------------------------------------------------
    // 时钟与复位
    // --------------------------------------------------------------------------
    input  wire        PixelClk,       // 像素主时钟 (接 FCLK_CLK0 50MHz 或 74.25MHz)
    input  wire        SerialClk,      // 5x 串行时钟 (若 ENABLE_CLK_GEN=1 可悬空或接地)
    input  wire        rst_n,          // 异步复位 (低有效)
    
    // --------------------------------------------------------------------------
    // 视频流输入 (直接连接 evm_pl_top 的对应端口)
    // --------------------------------------------------------------------------
    input  wire [23:0] vid_pData,      // 24-bit RGB888 [23:16]=R, [15:8]=G, [7:0]=B (接 vid_out_rgb)
    input  wire        vid_pVDE,       // 视频数据有效 (接 vid_out_valid)
    input  wire        vid_pHSync,     // 行同步 (若无直接置 1'b0)
    input  wire        vid_pVSync,     // 场同步 (接 vid_out_vs)
    
    // --------------------------------------------------------------------------
    // HDMI TMDS 物理差分端口 (右键 Make External 对应 pin_constraints.xdc)
    // --------------------------------------------------------------------------
    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_data_p,
    output wire [2:0]  tmds_data_n,
    output wire        hdmi_out_en     // HDMI 芯片使能 (恒输出 1'b1)
);

    assign hdmi_out_en = 1'b1;

    // ==========================================================================
    // 1. 时钟倍频逻辑 (PLLE2 生成 5x 串行时钟)
    // ==========================================================================
    wire clk_pixel_int;
    wire clk_5x_int;
    
    generate
        if (ENABLE_CLK_GEN) begin : gen_pll
            wire clkfb, clkfb_buf;
            wire clk0_unbuf, clk1_unbuf;
            
            PLLE2_BASE #(
                .BANDWIDTH          ("OPTIMIZED"),
                .CLKFBOUT_MULT      (20),       // 74.25MHz * 20 = 1485MHz VCO (800~1866MHz 允许范围)
                .CLKFBOUT_PHASE     (0.0),
                .CLKIN1_PERIOD      (13.468),   // 74.25MHz (13.468ns 标准 720P 像素时钟周期)
                .CLKOUT0_DIVIDE     (4),        // 1485MHz / 4 = 371.25MHz (5x 串行序列化时钟)
                .CLKOUT0_PHASE      (0.0),
                .CLKOUT1_DIVIDE     (20),       // 1485MHz / 20 = 74.25MHz (1x 像素基准时钟)
                .CLKOUT1_PHASE      (0.0),
                .DIVCLK_DIVIDE      (1),
                .STARTUP_WAIT       ("FALSE")
            ) u_pll (
                .CLKOUT0            (clk0_unbuf),
                .CLKOUT1            (clk1_unbuf),
                .CLKOUT2            (),
                .CLKOUT3            (),
                .CLKOUT4            (),
                .CLKOUT5            (),
                .CLKFBOUT           (clkfb),
                .CLKIN1             (PixelClk),
                .PWRDWN             (1'b0),
                .RST                (~rst_n),
                .CLKFBIN            (clkfb_buf)
            );
            
            BUFG u_bufg_fb (.I(clkfb),      .O(clkfb_buf));
            BUFG u_bufg_5x (.I(clk0_unbuf),  .O(clk_5x_int));
            BUFG u_bufg_1x (.I(clk1_unbuf),  .O(clk_pixel_int));
        end else begin : gen_ext_clk
            assign clk_5x_int    = SerialClk;
            assign clk_pixel_int = PixelClk;
        end
    endgenerate

    // ==========================================================================
    // 2. TMDS 8b/10b 编码器 (Blue=Ch0, Green=Ch1, Red=Ch2)
    // ==========================================================================
    wire [9:0] tmds_blue;
    wire [9:0] tmds_green;
    wire [9:0] tmds_red;
    
    // Blue 通道传输 HSync 与 VSync 控制字
    tmds_encoder u_enc_b (
        .clk      (clk_pixel_int),
        .rst_n    (rst_n),
        .data_in  (vid_pData[7:0]),
        .ctrl_in  ({vid_pVSync, vid_pHSync}),
        .de       (vid_pVDE),
        .data_out (tmds_blue)
    );
    
    // Green 通道传输固定 2'b00 控制字
    tmds_encoder u_enc_g (
        .clk      (clk_pixel_int),
        .rst_n    (rst_n),
        .data_in  (vid_pData[15:8]),
        .ctrl_in  (2'b00),
        .de       (vid_pVDE),
        .data_out (tmds_green)
    );
    
    // Red 通道传输固定 2'b00 控制字
    tmds_encoder u_enc_r (
        .clk      (clk_pixel_int),
        .rst_n    (rst_n),
        .data_in  (vid_pData[23:16]),
        .ctrl_in  (2'b00),
        .de       (vid_pVDE),
        .data_out (tmds_red)
    );

    // ==========================================================================
    // 3. 10:1 串行化器与 ODDR/OBUFDS 差分输出
    // ==========================================================================
    serializer_10_to_1 u_ser_b (
        .clk_pixel (clk_pixel_int),
        .clk_5x    (clk_5x_int),
        .rst_n     (rst_n),
        .data_in   (tmds_blue),
        .out_p     (tmds_data_p[0]),
        .out_n     (tmds_data_n[0])
    );

    serializer_10_to_1 u_ser_g (
        .clk_pixel (clk_pixel_int),
        .clk_5x    (clk_5x_int),
        .rst_n     (rst_n),
        .data_in   (tmds_green),
        .out_p     (tmds_data_p[1]),
        .out_n     (tmds_data_n[1])
    );

    serializer_10_to_1 u_ser_r (
        .clk_pixel (clk_pixel_int),
        .clk_5x    (clk_5x_int),
        .rst_n     (rst_n),
        .data_in   (tmds_red),
        .out_p     (tmds_data_p[2]),
        .out_n     (tmds_data_n[2])
    );

    // TMDS 像素时钟差分输出 (通过 ODDR 延时对齐)
    wire clk_serial;
    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_oddr_clk (
        .Q  (clk_serial),
        .C  (clk_pixel_int),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (~rst_n),
        .S  (1'b0)
    );

    OBUFDS u_bufds_clk (
        .I  (clk_serial),
        .O  (tmds_clk_p),
        .OB (tmds_clk_n)
    );

endmodule

// ==============================================================================
// 子模块: tmds_encoder (标准 DVI 1.0 8b/10b 编码器)
// ==============================================================================
module tmds_encoder (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_in,
    input  wire [1:0] ctrl_in,
    input  wire       de,
    output reg  [9:0] data_out
);
    wire [3:0] num_ones = data_in[0] + data_in[1] + data_in[2] + data_in[3] +
                          data_in[4] + data_in[5] + data_in[6] + data_in[7];
    
    wire use_xnor = (num_ones > 4'd4) || ((num_ones == 4'd4) && (data_in[0] == 1'b0));
    
    wire [8:0] q_m;
    assign q_m[0] = data_in[0];
    assign q_m[1] = use_xnor ? ~(q_m[0] ^ data_in[1]) : (q_m[0] ^ data_in[1]);
    assign q_m[2] = use_xnor ? ~(q_m[1] ^ data_in[2]) : (q_m[1] ^ data_in[2]);
    assign q_m[3] = use_xnor ? ~(q_m[2] ^ data_in[3]) : (q_m[2] ^ data_in[3]);
    assign q_m[4] = use_xnor ? ~(q_m[3] ^ data_in[4]) : (q_m[3] ^ data_in[4]);
    assign q_m[5] = use_xnor ? ~(q_m[4] ^ data_in[5]) : (q_m[4] ^ data_in[5]);
    assign q_m[6] = use_xnor ? ~(q_m[5] ^ data_in[6]) : (q_m[5] ^ data_in[6]);
    assign q_m[7] = use_xnor ? ~(q_m[6] ^ data_in[7]) : (q_m[6] ^ data_in[7]);
    assign q_m[8] = use_xnor ? 1'b0 : 1'b1;
    
    wire [3:0] ones_qm = q_m[0] + q_m[1] + q_m[2] + q_m[3] +
                         q_m[4] + q_m[5] + q_m[6] + q_m[7];
    wire [3:0] zeros_qm = 4'd8 - ones_qm;
    wire signed [4:0] diff_qm = $signed({1'b0, ones_qm}) - $signed({1'b0, zeros_qm});
    
    reg signed [4:0] disparity;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disparity <= 5'sd0;
            data_out  <= 10'd0;
        end else begin
            if (!de) begin
                disparity <= 5'sd0;
                case (ctrl_in)
                    2'b00:   data_out <= 10'b1101010100;
                    2'b01:   data_out <= 10'b0010101011;
                    2'b10:   data_out <= 10'b0101010100;
                    default: data_out <= 10'b1010101011;
                endcase
            end else begin
                if ((disparity == 5'sd0) || (ones_qm == 4'd4)) begin
                    data_out[9]   <= ~q_m[8];
                    data_out[8]   <= q_m[8];
                    data_out[7:0] <= q_m[8] ? q_m[7:0] : ~q_m[7:0];
                    if (q_m[8] == 1'b0)
                        disparity <= disparity - diff_qm;
                    else
                        disparity <= disparity + diff_qm;
                end else begin
                    if ((disparity > 5'sd0 && ones_qm > 4'd4) ||
                        (disparity < 5'sd0 && ones_qm < 4'd4)) begin
                        data_out[9]   <= 1'b1;
                        data_out[8]   <= q_m[8];
                        data_out[7:0] <= ~q_m[7:0];
                        disparity     <= disparity + {q_m[8], 1'b0} - diff_qm;
                    end else begin
                        data_out[9]   <= 1'b0;
                        data_out[8]   <= q_m[8];
                        data_out[7:0] <= q_m[7:0];
                        disparity     <= disparity - {~q_m[8], 1'b0} + diff_qm;
                    end
                end
            end
        end
    end
endmodule

// ==============================================================================
// 子模块: serializer_10_to_1 (7系列 ODDR 10:1 双沿串行发送与差分缓冲)
// ==============================================================================
module serializer_10_to_1 (
    input  wire       clk_pixel,
    input  wire       clk_5x,
    input  wire       rst_n,
    input  wire [9:0] data_in,
    output wire       out_p,
    output wire       out_n
);
    // 捕获像素时钟边沿以对齐串行发送相位
    (* ASYNC_REG = "TRUE" *) reg clk_p_d1, clk_p_d2;
    always @(posedge clk_5x or negedge rst_n) begin
        if (!rst_n) begin
            clk_p_d1 <= 1'b0;
            clk_p_d2 <= 1'b0;
        end else begin
            clk_p_d1 <= clk_pixel;
            clk_p_d2 <= clk_p_d1;
        end
    end
    wire sync_load = clk_p_d1 & ~clk_p_d2;

    reg [9:0] shift_reg;
    always @(posedge clk_5x or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 10'd0;
        end else if (sync_load) begin
            shift_reg <= data_in;
        end else begin
            shift_reg <= {2'b00, shift_reg[9:2]};
        end
    end

    wire d1 = shift_reg[0];
    wire d2 = shift_reg[1];
    wire serial_single;

    ODDR #(
        .DDR_CLK_EDGE ("SAME_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("ASYNC")
    ) u_oddr (
        .Q  (serial_single),
        .C  (clk_5x),
        .CE (1'b1),
        .D1 (d1),
        .D2 (d2),
        .R  (~rst_n),
        .S  (1'b0)
    );

    OBUFDS u_obufds (
        .I  (serial_single),
        .O  (out_p),
        .OB (out_n)
    );
endmodule
