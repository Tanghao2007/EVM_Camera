# ==============================================================================
# 正点原子 ZYNQ-7000 (XC7Z020-CLG400-2) 领航者开发板 EVM 运动放大工程物理引脚约束文件
# 硬件对应关系：
# 1. 摄像头：ATK-OV5640-DUAL 双目模块插在底板 J3 (2x20 排母) 插座上，使用 CMOS1 (左摄，奇数脚)
# 2. 视频输出：底板原生 HDMI 输出接口 (Bank 35 TMDS 差分对与 5V 使能控制)
# ==============================================================================

# ----------------- 1. ATK-OV5640-DUAL 摄像头 DVP 物理接口 (底板 J3 扩展口 CMOS1 左摄 / cam_0) -----------------
# 物理验证依据: 正点原子《领航者ZYNQ之嵌入式(SDK)开发指南》第三十二章/三十三章管脚约束 (Page 593)
# PCLK 像素时钟 (cam_pclk_0: N17)
set_property PACKAGE_PIN N17 [get_ports cmos_pclk_0]
set_property IOSTANDARD LVCMOS33 [get_ports cmos_pclk_0]

# VSYNC 场同步 (cam_vsync_0: M18)
set_property PACKAGE_PIN M18 [get_ports cmos_vsync_0]
set_property IOSTANDARD LVCMOS33 [get_ports cmos_vsync_0]

# HREF 行同步 (cam_href_0: T14)
set_property PACKAGE_PIN T14 [get_ports cmos_href_0]
set_property IOSTANDARD LVCMOS33 [get_ports cmos_href_0]

# 8 位数据总线 D[7:0] (严格对应正点原子官方双目左摄 cam_data_0[7:0])
# D0: W13
# D1: T12
# D2: U12
# D3: V15
# D4: W15
# D5: P15
# D6: P16
# D7: T17
set_property PACKAGE_PIN W13 [get_ports {cmos_data_0[0]}]
set_property PACKAGE_PIN T12 [get_ports {cmos_data_0[1]}]
set_property PACKAGE_PIN U12 [get_ports {cmos_data_0[2]}]
set_property PACKAGE_PIN V15 [get_ports {cmos_data_0[3]}]
set_property PACKAGE_PIN W15 [get_ports {cmos_data_0[4]}]
set_property PACKAGE_PIN P15 [get_ports {cmos_data_0[5]}]
set_property PACKAGE_PIN P16 [get_ports {cmos_data_0[6]}]
set_property PACKAGE_PIN T17 [get_ports {cmos_data_0[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {cmos_data_0[*]}]

# 时钟约束: OV5640 像素时钟 (~56MHz/72MHz/84MHz)
create_clock -period 13.888 -name cmos_pclk_0 [get_ports cmos_pclk_0]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -hierarchical *cmos_pclk*]

# ----------------- 2. ATK-OV5640-DUAL 控制引脚与 SCCB/I2C 物理接口 -----------------
# 摄像头复位信号 (cam_rst_n_0: V12, 低有效)
set_property PACKAGE_PIN V12 [get_ports -filter {NAME =~ *cmos_rst_n_0*}]
set_property IOSTANDARD LVCMOS33 [get_ports -filter {NAME =~ *cmos_rst_n_0*}]

# 摄像头休眠控制 (cam_pwdn_0: R18, 高有效，驱动为低电平 0 退出休眠工作模式)
set_property PACKAGE_PIN R18 [get_ports -filter {NAME =~ *cmos_pwdn_0*}]
set_property IOSTANDARD LVCMOS33 [get_ports -filter {NAME =~ *cmos_pwdn_0*}]

# 摄像头主输入时钟 XCLK (双目模块自带24MHz有源晶振，此处绑定至J3空闲NC引脚J18确保DRC通过，严禁绑定至R16以防与右摄D6输出冲突)
set_property PACKAGE_PIN J18 [get_ports -filter {NAME =~ *cmos_xclk_0*}]
set_property IOSTANDARD LVCMOS33 [get_ports -filter {NAME =~ *cmos_xclk_0*}]

# 摄像头 SCCB/I2C 总线: SCL = M17, SDA = T15
set_property PACKAGE_PIN M17 [get_ports IIC_0_0_scl_io]
set_property IOSTANDARD LVCMOS33 [get_ports IIC_0_0_scl_io]
set_property PULLUP true [get_ports IIC_0_0_scl_io]

set_property PACKAGE_PIN T15 [get_ports IIC_0_0_sda_io]
set_property IOSTANDARD LVCMOS33 [get_ports IIC_0_0_sda_io]
set_property PULLUP true [get_ports IIC_0_0_sda_io]

# ----------------- 3. HDMI 输出接口 (正点原子底板原生 HDMI 接口，Bank 35 TMDS 差分对与 5V 供电控制) -----------------
# TMDS 时钟差分对 (Bank 35 IO_L12P/N_T1_MRCC_35: L14为IOBM正端，L15为IOBS负端)
set_property PACKAGE_PIN L14 [get_ports tmds_clk_p_0]
set_property IOSTANDARD TMDS_33 [get_ports tmds_clk_p_0]
set_property PACKAGE_PIN L15 [get_ports tmds_clk_n_0]
set_property IOSTANDARD TMDS_33 [get_ports tmds_clk_n_0]

# TMDS 数据通道 0 差分对 (Bank 35 IO_L9P/N_T1_SRCC_35, 底板 net: HDMI_D0_P/N)
set_property PACKAGE_PIN K19 [get_ports {tmds_data_p_0[0]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_p_0[0]}]
set_property PACKAGE_PIN J19 [get_ports {tmds_data_n_0[0]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_n_0[0]}]

# TMDS 数据通道 1 差分对 (Bank 35 IO_L7P/N_T1_AD2P/N_35, 底板 net: HDMI_D1_P/N)
set_property PACKAGE_PIN M14 [get_ports {tmds_data_p_0[1]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_p_0[1]}]
set_property PACKAGE_PIN M15 [get_ports {tmds_data_n_0[1]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_n_0[1]}]

# TMDS 数据通道 2 差分对 (Bank 35 IO_L6P/N_T0_35, 底板 net: HDMI_D2_P/N)
set_property PACKAGE_PIN L16 [get_ports {tmds_data_p_0[2]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_p_0[2]}]
set_property PACKAGE_PIN L17 [get_ports {tmds_data_n_0[2]}]
set_property IOSTANDARD TMDS_33 [get_ports {tmds_data_n_0[2]}]

# HDMI 5V 供电芯片 TPS2051 使能引脚 (Bank 35 G17, 高电平开启底板 +5V 与上拉)
set_property PACKAGE_PIN G17 [get_ports hdmi_out_en_0]
set_property IOSTANDARD LVCMOS33 [get_ports hdmi_out_en_0]

# ----------------- 4. 跨时钟域 (CDC) 与异步时钟组约束 -----------------
# 4.1 独立外部晶振摄像头时钟 (cmos_pclk_0) 与 PS 内部时钟 (clk_fpga_0) 完全异步
set_clock_groups -asynchronous \
    -group [get_clocks -filter {NAME =~ *cmos_pclk*}] \
    -group [get_clocks -filter {NAME =~ *clk_fpga* || NAME =~ *FCLK_CLK0*}]

# 4.2 rgb2dvi 模块内异步跨时钟同步寄存器与复位 False Path
set_false_path -through [get_pins -hierarchical -filter {NAME =~ *rgb2dvi*/rst_n}]
set_false_path -to [get_pins -hierarchical -filter {NAME =~ *rgb2dvi*/*clk_p_d1_reg*/D}]
