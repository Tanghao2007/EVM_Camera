// ==============================================================================
// 模块名称: display_mux
// 功能描述: 3 大核心显示模式无缝选择器与多通路延迟对齐仲裁模块
// 实现思路: 1. 对应用户确定的 3 种实际应用模式:
//             1: 原始视频直通 (Bypass)
//             2: EVM 运动微动放大画面 (Motion Magnified)
//             3: 热力图半透明叠加原画 (Heatmap Overlay - 核心推荐)
//          2. 内部通过移位寄存器对各分支进行精确打拍对齐 (均对齐至 4 拍延迟)，
//             确保在按键/串口无缝切换显示模式时，画面绝不发生抖动、撕裂或色彩错位!
// ==============================================================================

module display_mux (
    input  wire        clk,
    input  wire        rst_n,
    
    // 显示模式配置 (1: 直通, 2: 放大, 3: 热力叠加)
    input  wire [2:0]  disp_mode,
    
    // 输入数据分支 (严格保证各分支时钟同源)
    input  wire        in_valid_base,    // 基准有效脉冲 (来自 iir_bandpass_coring)
    input  wire        in_vs_base,       // 基准场同步
    input  wire [23:0] in_rgb_orig,      // 原画流 (延迟 0 拍)
    
    input  wire [23:0] in_rgb_amp,       // 放大流 (来自 amplify_add, 延迟 2 拍)
    
    input  wire        in_valid_overlay, // 叠加流有效信号 (延迟 4 拍)
    input  wire        in_vs_overlay,    // 叠加流场同步
    input  wire [23:0] in_rgb_overlay,   // 叠加流 (来自 heatmap_overlay, 延迟 4 拍)
    
    // 最终选通输出流 (送往 HDMI / VDMA)
    output reg         out_valid,
    output reg         out_vs,
    output reg  [23:0] out_rgb
);

    // --------------------------------------------------------------------------
    // 原画分支延迟 4 拍，匹配 heatmap_overlay
    // --------------------------------------------------------------------------
    reg [23:0] orig_d1, orig_d2, orig_d3, orig_d4;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            orig_d1 <= 24'd0;
            orig_d2 <= 24'd0;
            orig_d3 <= 24'd0;
            orig_d4 <= 24'd0;
        end else begin
            orig_d1 <= in_rgb_orig;
            orig_d2 <= orig_d1;
            orig_d3 <= orig_d2;
            orig_d4 <= orig_d3;
        end
    end

    // --------------------------------------------------------------------------
    // 放大分支延迟 2 拍，匹配 heatmap_overlay (2 + 2 = 4 拍)
    // --------------------------------------------------------------------------
    reg [23:0] amp_d1, amp_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            amp_d1 <= 24'd0;
            amp_d2 <= 24'd0;
        end else begin
            amp_d1 <= in_rgb_amp;
            amp_d2 <= amp_d1;
        end
    end

    // --------------------------------------------------------------------------
    // 多路无缝选择 Mux
    // --------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_vs    <= 1'b0;
            out_rgb   <= 24'd0;
        end else begin
            out_valid <= in_valid_overlay;
            out_vs    <= in_vs_overlay;

            case (disp_mode)
                3'd1:    out_rgb <= orig_d4;          // 模式 1: 原始视频直通
                3'd2:    out_rgb <= amp_d2;           // 模式 2: 微动放大
                3'd3:    out_rgb <= in_rgb_overlay;   // 模式 3: 热力图半透明叠加
                default: out_rgb <= in_rgb_overlay;   // 默认模式 3
            endcase
        end
    end

endmodule
