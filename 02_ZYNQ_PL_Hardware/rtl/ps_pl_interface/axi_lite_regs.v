// ==============================================================================
// 模块名称: axi_lite_regs
// 功能描述: AXI4-Lite 从机接口寄存器映射模块 (实现 PS-PL 软硬件无缝交互与参数动态调整)
// 实现思路: 遵循标准 ARM AMBA AXI4-Lite 协议规范。
//          映射基地址 0x43C00000 上的控制/配置/遥测寄存器:
//          0x00: 控制与状态寄存器 (AP_CTRL: Start/Auto-restart)
//          0x10: 图像宽度寄存器 (WIDTH, 默认 640 / 1280)
//          0x18: 图像高度寄存器 (HEIGHT, 默认 360 / 720)
//          0x20: 放大倍数寄存器 (ALPHA, 0~150, 默认 30)
//          0x28: 高通截止因子 r1 (R1_FIXED, Q12, 默认 1638 对应 0.4)
//          0x30: 低通截止因子 r2 (R2_FIXED, Q12, 默认 205  对应 0.05)
//          0x38: 显示模式寄存器 (DISP_MODE: 1=Bypass, 2=Amplify, 3=Heatmap)
//          0x40: 调色板选择寄存器 (COLORMAP: 0=Jet, 1=Viridis)
//          0x48: 硬件时域抗噪死区门限 (NOISE_GATE, Q12, 默认 35)
//          0x50: [只读] 实时振动峰值幅度 (PEAK_MAG: 供 VIBRATION ALARM 遥测)
//          0x58: [只读] 帧计数器与硬件流水线运行状态 (STATUS)
// ==============================================================================

module axi_lite_regs #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 7
)(
    // AXI4-Lite 从机总线时钟与复位
    input  wire                                  s_axi_aclk,
    input  wire                                  s_axi_aresetn,
    
    // 写地址通道
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_awaddr,
    input  wire [2:0]                            s_axi_awprot,
    input  wire                                  s_axi_awvalid,
    output wire                                  s_axi_awready,
    
    // 写数据通道
    input  wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]    s_axi_wstrb,
    input  wire                                  s_axi_wvalid,
    output wire                                  s_axi_wready,
    
    // 写响应通道
    output wire [1:0]                            s_axi_bresp,
    output wire                                  s_axi_bvalid,
    input  wire                                  s_axi_bready,
    
    // 读地址通道
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        s_axi_araddr,
    input  wire [2:0]                            s_axi_arprot,
    input  wire                                  s_axi_arvalid,
    output wire                                  s_axi_arready,
    
    // 读数据通道
    output wire [C_S_AXI_DATA_WIDTH-1:0]        s_axi_rdata,
    output wire [1:0]                            s_axi_rresp,
    output wire                                  s_axi_rvalid,
    input  wire                                  s_axi_rready,
    
    // --------------------------------------------------------------------------
    // 对外解出的 PL 端控制配置信号
    // --------------------------------------------------------------------------
    output reg  [7:0]   reg_ap_ctrl,
    output reg  [15:0]  reg_width,
    output reg  [15:0]  reg_height,
    output reg  [7:0]   reg_alpha,
    output reg  [11:0]  reg_r1_fixed,
    output reg  [11:0]  reg_r2_fixed,
    output reg  [2:0]   reg_disp_mode,
    output reg          reg_colormap_sel,
    output reg  [11:0]  reg_noise_gate,
    
    // 来自硬件流水线的只读遥测状态输入
    input  wire [7:0]   hw_peak_mag,            // 实时振动峰值幅度 (VIBRATION ALARM)
    input  wire [31:0]  hw_frame_cnt,           // 帧计数器
    input  wire [31:0]  diag_pclk_cnt,          // 摄像头 PCLK 物理时钟计数器
    input  wire [31:0]  diag_cap_valid_cnt,     // DVP 抓取有效像素计数器
    input  wire [31:0]  diag_mux_valid_cnt,     // EVM 最终选通有效像素计数器
    input  wire [31:0]  diag_status             // 流水线状态信号字
);

    // 内部总线握手逻辑
    reg axi_awready;
    reg axi_wready;
    reg [1:0] axi_bresp;
    reg axi_bvalid;
    reg axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0] axi_rdata;
    reg [1:0] axi_rresp;
    reg axi_rvalid;

    assign s_axi_awready = axi_awready;
    assign s_axi_wready  = axi_wready;
    assign s_axi_bresp   = axi_bresp;
    assign s_axi_bvalid  = axi_bvalid;
    assign s_axi_arready = axi_arready;
    assign s_axi_rdata   = axi_rdata;
    assign s_axi_rresp   = axi_rresp;
    assign s_axi_rvalid  = axi_rvalid;

    // 写地址握手
    reg [C_S_AXI_ADDR_WIDTH-1:0] axi_awaddr_reg;
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_awready    <= 1'b0;
            axi_awaddr_reg <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if (~axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                axi_awready    <= 1'b1;
                axi_awaddr_reg <= s_axi_awaddr;
            end else begin
                axi_awready    <= 1'b0;
            end
        end
    end

    // 写数据握手
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_wready <= 1'b0;
        end else begin
            if (~axi_wready && s_axi_wvalid && s_axi_awvalid) begin
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    // 写响应握手
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b00;
        end else begin
            if (axi_awready && s_axi_awvalid && ~axi_bvalid && axi_wready && s_axi_wvalid) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b00; // 'OKAY' 响应
            end else if (s_axi_bready && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------------
    // 写寄存器逻辑
    // --------------------------------------------------------------------------
    wire slv_reg_wren = axi_wready && s_axi_wvalid && axi_awready && s_axi_awvalid;

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            reg_ap_ctrl      <= 8'h81;     // 默认 Auto-restart 启动
            reg_width        <= 16'd640;   // 默认 360P 降采样后宽度
            reg_height       <= 16'd360;   // 默认 360P 降采样后高度
            reg_alpha        <= 8'd30;     // 默认 30 倍放大
            reg_r1_fixed     <= 12'd1638;  // r1 = 0.4  * 4096 = 1638
            reg_r2_fixed     <= 12'd205;   // r2 = 0.05 * 4096 = 205
            reg_disp_mode    <= 3'd3;      // 默认模式 3: 热力图叠加
            reg_colormap_sel <= 1'b0;      // 默认 Jet 色谱
            reg_noise_gate   <= 12'd35;    // 默认抗噪死区门限
        end else if (slv_reg_wren) begin
            case (axi_awaddr_reg[6:0])
                7'h00: reg_ap_ctrl      <= s_axi_wdata[7:0];
                7'h10: reg_width        <= s_axi_wdata[15:0];
                7'h18: reg_height       <= s_axi_wdata[15:0];
                7'h20: reg_alpha        <= s_axi_wdata[7:0];
                7'h28: reg_r1_fixed     <= s_axi_wdata[11:0];
                7'h30: reg_r2_fixed     <= s_axi_wdata[11:0];
                7'h38: reg_disp_mode    <= s_axi_wdata[2:0];
                7'h40: reg_colormap_sel <= s_axi_wdata[0];
                7'h48: reg_noise_gate   <= s_axi_wdata[11:0];
                default: ;
            endcase
        end
    end

    // --------------------------------------------------------------------------
    // 读地址与数据握手
    // --------------------------------------------------------------------------
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_arready <= 1'b0;
        end else begin
            if (~axi_arready && s_axi_arvalid) begin
                axi_arready <= 1'b1;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b00;
        end else begin
            if (axi_arready && s_axi_arvalid && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b00;
            end else if (axi_rvalid && s_axi_rready) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    // 读寄存器多路复用器
    always @(*) begin
        case (s_axi_araddr[6:0])
            7'h00: axi_rdata = {24'd0, reg_ap_ctrl};
            7'h10: axi_rdata = {16'd0, reg_width};
            7'h18: axi_rdata = {16'd0, reg_height};
            7'h20: axi_rdata = {24'd0, reg_alpha};
            7'h28: axi_rdata = {20'd0, reg_r1_fixed};
            7'h30: axi_rdata = {20'd0, reg_r2_fixed};
            7'h38: axi_rdata = {29'd0, reg_disp_mode};
            7'h40: axi_rdata = {31'd0, reg_colormap_sel};
            7'h48: axi_rdata = {20'd0, reg_noise_gate};
            7'h50: axi_rdata = {24'd0, hw_peak_mag};   // 实时峰值遥测
            7'h58: axi_rdata = hw_frame_cnt;          // 帧计数
            7'h60: axi_rdata = diag_pclk_cnt;         // PCLK 周期计数
            7'h64: axi_rdata = diag_cap_valid_cnt;    // 摄像头有效像素计数
            7'h68: axi_rdata = diag_mux_valid_cnt;    // 输出有效像素计数
            7'h6c: axi_rdata = diag_status;           // 流水线实时信号字
            default: axi_rdata = 32'h00000000;
        endcase
    end

endmodule
