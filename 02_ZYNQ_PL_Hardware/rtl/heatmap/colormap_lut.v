// ==============================================================================
// 模块名称: colormap_lut
// 功能描述: 256 阶双模式伪彩色硬件查找核 (Jet 彩虹色谱 / Viridis 翠绿色谱)
// 实现思路: 采用硬件分段线性逼近逻辑代替笨重的大容量 ROM，
//          0 消耗 Block RAM，全流水线纯组合+单拍寄存输出，时序达到 200MHz+。
// ==============================================================================

module colormap_lut (
    input  wire        clk,
    input  wire        rst_n,
    
    input  wire        colormap_sel,     // 0: Jet 色谱; 1: Viridis 色谱
    input  wire [7:0]  val,              // 归一化振动强度标量 (0~255)
    
    output reg  [7:0]  r_heat,
    output reg  [7:0]  g_heat,
    output reg  [7:0]  b_heat
);

    // --------------------------------------------------------------------------
    // 伪彩色 Jet 映射
    // --------------------------------------------------------------------------
    reg [7:0] jet_r, jet_g, jet_b;
    always @(*) begin
        if (val < 8'd32) begin
            jet_r = 8'd0;
            jet_g = 8'd0;
            jet_b = 8'd128 + {val[5:0], 2'b00};
        end else if (val < 8'd96) begin
            jet_r = 8'd0;
            jet_g = (val - 8'd32) << 2;
            jet_b = 8'd255;
        end else if (val < 8'd160) begin
            jet_r = (val - 8'd96) << 2;
            jet_g = 8'd255;
            jet_b = 8'd255 - ((val - 8'd96) << 2);
        end else if (val < 8'd224) begin
            jet_r = 8'd255;
            jet_g = 8'd255 - ((val - 8'd160) << 2);
            jet_b = 8'd0;
        end else begin
            jet_r = 8'd255 - ((val - 8'd224) << 2);
            jet_g = 8'd0;
            jet_b = 8'd0;
        end
    end

    // --------------------------------------------------------------------------
    // 伪彩色 Viridis 映射
    // --------------------------------------------------------------------------
    reg [7:0] vir_r, vir_g, vir_b;
    wire [7:0] sub1 = val - 8'd64;
    wire [7:0] sub2 = val - 8'd128;
    wire [7:0] sub3 = val - 8'd192;

    always @(*) begin
        if (val < 8'd64) begin
            vir_r = 8'd68 - ((val * 8'd30) >> 6);
            vir_g = 8'd1  + ((val * 8'd83) >> 6);
            vir_b = 8'd84 + ((val * 8'd50) >> 6);
        end else if (val < 8'd128) begin
            vir_r = 8'd38  - ((sub1 * 8'd7)  >> 6);
            vir_g = 8'd84  + ((sub1 * 8'd50) >> 6);
            vir_b = 8'd134 + ((sub1 * 8'd9)  >> 6);
        end else if (val < 8'd192) begin
            vir_r = 8'd31  + ((sub2 * 8'd80) >> 6);
            vir_g = 8'd134 + ((sub2 * 8'd44) >> 6);
            vir_b = 8'd143 - ((sub2 * 8'd65) >> 6);
        end else begin
            vir_r = 8'd111 + ((sub3 * 8'd142) >> 6);
            vir_g = 8'd178 + ((sub3 * 8'd43)  >> 6);
            vir_b = 8'd78  - ((sub3 * 8'd40)  >> 6);
        end
    end

    // --------------------------------------------------------------------------
    // 单拍寄存输出
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_heat <= 8'd0;
            g_heat <= 8'd0;
            b_heat <= 8'd0;
        end else begin
            if (colormap_sel == 1'b0) begin
                r_heat <= jet_r;
                g_heat <= jet_g;
                b_heat <= jet_b;
            end else begin
                r_heat <= vir_r;
                g_heat <= vir_g;
                b_heat <= vir_b;
            end
        end
    end

endmodule
