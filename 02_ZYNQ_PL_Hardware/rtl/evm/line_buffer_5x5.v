// ==============================================================================
// 模块名称: line_buffer_5x5
// 功能描述: 4 行 BRAM 缓存与 5x5 像素滑动窗口生成器
// 实现思路: 使用 4 块单时钟 RAM 分别缓存前 4 行像素。当前输入为第 5 行。
//          配合 5 级寄存器构建 5x5 滑动滤波窗口，每个时钟周期并行输出 25 个像素。
// ==============================================================================

module line_buffer_5x5 #(
    parameter WIDTH = 640
)(
    input  wire        clk,
    input  wire        rst_n,
    
    input  wire        in_valid,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire [7:0]  in_data,
    input  wire [23:0] in_rgb,
    
    output reg         out_valid,
    output reg         out_hs,
    output reg         out_vs,
    // 5x5 滑动窗口像素输出 (p_row_col: row 0~4, col 0~4)
    output reg  [7:0]  p00, p01, p02, p03, p04,
    output reg  [7:0]  p10, p11, p12, p13, p14,
    output reg  [7:0]  p20, p21, p22, p23, p24, // p22 为中心像素
    output reg  [7:0]  p30, p31, p32, p33, p34,
    output reg  [7:0]  p40, p41, p42, p43, p44,
    output reg  [23:0] center_rgb               // 与中心像素严格对齐的原 RGB
);

    reg [10:0] col_addr;
    
    // 4 行 BRAM 缓存 (Y 亮度分量)
    reg [7:0] line_ram0 [0:WIDTH-1];
    reg [7:0] line_ram1 [0:WIDTH-1];
    reg [7:0] line_ram2 [0:WIDTH-1];
    reg [7:0] line_ram3 [0:WIDTH-1];

    // 2 行 BRAM 缓存 (RGB 分量，用于使原图中心像素 center_rgb 与 p22 空间绝对对齐)
    reg [23:0] line_ram_rgb2 [0:WIDTH-1];
    reg [23:0] line_ram_rgb3 [0:WIDTH-1];

    // 读出数据寄存器
    wire [7:0]  rdata0 = line_ram0[col_addr];
    wire [7:0]  rdata1 = line_ram1[col_addr];
    wire [7:0]  rdata2 = line_ram2[col_addr];
    wire [7:0]  rdata3 = line_ram3[col_addr];
    wire [7:0]  rdata4 = in_data;

    wire [23:0] rdata_rgb2 = line_ram_rgb2[col_addr];

    // 行写入与滚动更新
    always @(posedge clk) begin
        if (in_valid) begin
            line_ram0[col_addr] <= rdata1;
            line_ram1[col_addr] <= rdata2;
            line_ram2[col_addr] <= rdata3;
            line_ram3[col_addr] <= in_data;

            line_ram_rgb2[col_addr] <= line_ram_rgb3[col_addr];
            line_ram_rgb3[col_addr] <= in_rgb;
        end
    end

    // 列地址计数器 (在 in_vs 或 !in_hs 行消隐期强行复位归零)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_addr <= 11'd0;
        end else if (in_vs || !in_hs) begin
            col_addr <= 11'd0;
        end else if (in_valid) begin
            col_addr <= (col_addr == WIDTH - 1) ? 11'd0 : col_addr + 1'b1;
        end
    end

    // RGB 水平移位寄存器 (2 拍延迟，匹配 p22 中心位置)
    reg [23:0] rgb_s1, rgb_s2;

    // 5x5 窗口移位寄存器
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_hs    <= 1'b0;
            out_vs    <= 1'b0;
            p00 <= 8'd0; p01 <= 8'd0; p02 <= 8'd0; p03 <= 8'd0; p04 <= 8'd0;
            p10 <= 8'd0; p11 <= 8'd0; p12 <= 8'd0; p13 <= 8'd0; p14 <= 8'd0;
            p20 <= 8'd0; p21 <= 8'd0; p22 <= 8'd0; p23 <= 8'd0; p24 <= 8'd0;
            p30 <= 8'd0; p31 <= 8'd0; p32 <= 8'd0; p33 <= 8'd0; p34 <= 8'd0;
            p40 <= 8'd0; p41 <= 8'd0; p42 <= 8'd0; p43 <= 8'd0; p44 <= 8'd0;
            rgb_s1     <= 24'd0;
            rgb_s2     <= 24'd0;
            center_rgb <= 24'd0;
        end else begin
            out_valid <= in_valid;
            out_hs    <= in_hs;
            out_vs    <= in_vs;

            if (in_valid) begin
                // 每行分别做 5 级移位
                p04 <= rdata0; p03 <= p04; p02 <= p03; p01 <= p02; p00 <= p01;
                p14 <= rdata1; p13 <= p14; p12 <= p13; p11 <= p12; p10 <= p11;
                p24 <= rdata2; p23 <= p24; p22 <= p23; p21 <= p22; p20 <= p21;
                p34 <= rdata3; p33 <= p34; p32 <= p33; p31 <= p32; p30 <= p31;
                p44 <= rdata4; p43 <= p44; p42 <= p43; p41 <= p42; p40 <= p41;

                // 与 p22 (row2, col2) 严格同轴对齐
                rgb_s1     <= rdata_rgb2;
                rgb_s2     <= rgb_s1;
                center_rgb <= rgb_s2;
            end
        end
    end

endmodule
