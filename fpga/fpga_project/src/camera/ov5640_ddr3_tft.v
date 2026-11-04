module ov5640_ddr3_tft(
    //System clock reset
    input       clk50m        , //系统时钟输入，50MHz
    input       reset_n       , //复位信号输入

    //IMU 姿态（度×10，有符号）
    input  wire signed [15:0] imu_roll  ,
    input  wire signed [15:0] imu_pitch ,
    input  wire signed [15:0] imu_yaw   ,

    //IMU 原始数据（调试显示）
    input  wire signed [15:0] imu_ax    ,
    input  wire signed [15:0] imu_ay    ,
    input  wire signed [15:0] imu_az    ,
    input  wire signed [15:0] imu_gx    ,
    input  wire signed [15:0] imu_gy    ,
    input  wire signed [15:0] imu_gz    ,

    //激光测距电压 ×100（0.01V）
    input  wire [15:0]        volt_x100 ,

    //编码器计数（OSD 第 5 行显示）
    input  wire [15:0]        enc_cnt   ,

    //IMU 对零进行中（屏幕显示 ZERO...）
    input  wire               imu_zeroing ,

    //触摸按钮状态（用于按钮高亮）
    input  wire               tp_pressed,
    input  wire [3:0]         tp_cell,

    //模式按钮选择（0=无 1=避障 2=绕障）
    input  wire [1:0]         mode_sel,

    //YOLO 目标框（来自以太网回传）
    input  wire [4:0]         box_cnt,
    input  wire [255:0]       box_x1,
    input  wire [255:0]       box_y1,
    input  wire [255:0]       box_x2,
    input  wire [255:0]       box_y2,
    input  wire [127:0]       box_score,
    input  wire [127:0]       box_cls,

    //触摸光标
    input  wire               cur_pressed,
    input  wire [11:0]        cur_x,
    input  wire [11:0]        cur_y,

    //camera interface
    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,
    //led
    output [3:0]led           ,

    //摄像头像素流输出（以太网发送用，pclk 域）
    output wire        cam_pclk  ,
    output wire [15:0] cam_pixel ,
    output wire        cam_valid ,
    output wire        cam_hs    ,
    output wire        cam_vs    ,
    output wire [11:0] cam_x     ,
    output wire [11:0] cam_y     ,

    //TFT Interface               
    output [15:0]   TFT_rgb       , //TFT数据输出
    output          TFT_hs        , //TFT行同步信号
    output          TFT_vs        , //TFT场同步信号
    output          TFT_clk       , //TFT像素时钟
    output          TFT_de        , //TFT数据使能
    output          TFT_pwm       , //TFT背光控制

    //ddr
    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

);
    //Set IMAGE Size  
    parameter IMAGE_WIDTH  = 800;
    parameter IMAGE_HEIGHT = 480;

    wire          loc_clk24m;
    wire          g_rst_p;
    wire          clk_200M;
    wire          pll_lock;
  wire          loc_clk50m;
  wire          loc_clk200m;
  wire          loc_clk33m;
  wire          loc_clk165m;
    //camera
    wire          camera_init_done;
    wire          pclk_bufg_o;

    //ddr
	wire fifo_aclr;
	wire fifo_wrreq;
	wire [15:0] fifo_wrdata;
    wire init_calib_complete;
    
    //hdmi
    wire clk_vga;//74.25m
	wire clk_vgax5;//371.25m
    wire hdmi_pll_lock;
    wire loc_clk33m;
    wire loc_clk50m;
    wire clk_disp;

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;
 
	wire video_hs;
	wire video_vs;
	wire video_de;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;


    assign  g_rst_p = ~pll_lock;

    assign led = {~g_rst_p,camera_init_done,init_calib_complete,pll_lock};


    ddr3_pll ddr3_pll(
        .lock(pll_lock), //output lock
        .clkout0(loc_clk50m), //output clkout0
        .clkout1(loc_clk200m), //output clkout1
        .clkin(clk50m), //input clkin
        .reset(~reset_n), //input reset
        .enclk0(1'b1), //input enclk0
        .enclk1(1'b1), //input enclk1
        .enclk2(pll_stop) //input enclk2
    );

    Gowin_PLL Gowin_PLL(
        .clkout0(loc_clk165m), //output clkout0
        .clkout1(loc_clk24m), //output clkout1
        .clkout3(loc_clk33m), //output clkout3
        .clkin(clk50m), //input clkin
        .reset(~reset_n) //input reset
    );

    assign camera_xclk = loc_clk24m;
    //摄像头
    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ),//系统时钟采用50MHz
        .SCL_CLOCK      ( 400_000      ),//SCL总线时钟采用400kHz
        .CAMERA_TYPE    ( "ov5640"     ),//"ov5640" or "ov7725"
        .IMAGE_TYPE     ( 0            ),// 0: RGB; 1: JPEG
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ),// 图片宽度
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ),// 图片高度
        .IMAGE_FLIP_EN  ( 0            ),// 0: 不翻转，1: 上下翻转
        .IMAGE_MIRROR_EN( 0            ) // 0: 不镜像，1: 左右镜像
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (~g_rst_p         ),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );
    

    assign pclk_bufg_o = camera_pclk;

    DVP_Capture DVP_Capture(
        .Rst_n      (reset_n         ),//input
        .PCLK       (pclk_bufg_o      ),//input
        .Vsync      (camera_vsync     ),//input
        .Href       (camera_href      ),//input
        .Data       (camera_data      ),//input     [7:0]

        .ImageState (fifo_aclr        ),//output reg
        .DataValid  (fifo_wrreq       ),//output
        .DataPixel  (fifo_wrdata      ),//output    [15:0]
        .DataHs     (dvp_hs           ),//output
        .DataVs     (wr_load          ),//output
        .Xaddr      (dvp_x            ),//output    [11:0],start is 1
        .Yaddr      (dvp_y            )   //output    [11:0],start is 1
    );
    

    wire        dvp_hs;
    wire [11:0] dvp_x, dvp_y;

    assign cam_pclk  = pclk_bufg_o;
    assign cam_pixel = fifo_wrdata;
    assign cam_valid = fifo_wrreq;
    assign cam_hs    = dvp_hs;
    assign cam_vs    = wr_load;
    assign cam_x     = dvp_x;
    assign cam_y     = dvp_y;

    wire rdfifo_rden;
    wire wrfifo_clr;
    wire [15:0] wrfifo_din;
    wire wrfifo_clk;
    wire rdfifo_clr;
    wire rdfifo_clk;
    wire [15:0] rdfifo_dout;
    wire Frame_Begin;
    
    assign wrfifo_clr = fifo_aclr;
    assign wrfifo_clk = pclk_bufg_o;
    assign rdfifo_clr = 0;
    assign rdfifo_clk = clk_vga;
    assign wrfifo_wren = fifo_wrreq;
    assign wrfifo_din = fifo_wrdata;
    assign disp_data = {rdfifo_dout[15:11],3'd0,rdfifo_dout[10:5],2'd0,rdfifo_dout[4:0],3'd0}; 

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0] burst_len = IMAGE_WIDTH[10:3];

    ddr3_ctrl_2port ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,      //50M时钟信号
        .pll_stop(pll_stop)                ,
        .pll_lock(pll_lock)            ,
        .clk_200m(loc_clk200m)            ,      //DDR3参考时钟信号
        .sys_rst_n(reset_n)           ,      //外部复位信号
        .init_calib_complete(init_calib_complete) ,    //DDR初始化完成信号

        //用户接口
        .rd_load(TFT_vs)             ,   //输出源更新信号
        .wr_load(wr_load)             ,   //输入源更新信号
        .app_addr_rd_min(28'd0)     ,   //读DDR3的起始地址
        .app_addr_rd_max(app_addr_max)     ,   //读DDR3的结束地址
        .rd_bust_len(burst_len)         ,   //从DDR3中读数据时的突发长度
        .app_addr_wr_min(28'd0)     ,   //写DD3的起始地址
        .app_addr_wr_max(app_addr_max)     ,   //写DDR的结束地址
        .wr_bust_len(burst_len)         ,   //向DDR3中写数据时的突发长度

        .wr_clk(wrfifo_clk)             ,//wr_fifo的写时钟信号
        .wfifo_wren(wrfifo_wren)          , //wr_fifo的写使能信号
        .wfifo_din(wrfifo_din)           , //写入到wr_fifo中的数据
        .wrfifo_full(),
        .rd_clk(clk_disp)              , //rd_fifo的读时钟信号
        .rfifo_rden(rdfifo_rden)          , //rd_fifo的读使能信号
        .rdfifo_empty(),
        .rfifo_dout(rdfifo_dout)          , //rd_fifo读出的数据信号 

        //DDR3   
        .ddr3_dq(IO_ddr_dq)             ,   //DDR3 数据
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,   //DDR3 dqs负
        .ddr3_dqs_p(IO_ddr_dqs)          ,   //DDR3 dqs正  
        .ddr3_addr(O_ddr_addr)           ,   //DDR3 地址   
        .ddr3_ba(O_ddr_ba)             ,   //DDR3 banck 选择
        .ddr3_ras_n(O_ddr_ras_n)          ,   //DDR3 行选择
        .ddr3_cas_n(O_ddr_cas_n)          ,   //DDR3 列选择
        .ddr3_we_n(O_ddr_we_n)           ,   //DDR3 读写选择
        .ddr3_reset_n(O_ddr_reset_n)        ,   //DDR3 复位
        .ddr3_ck_p(O_ddr_clk)          ,   //DDR3 时钟正
        .ddr3_ck_n(O_ddr_clk_n)           ,   //DDR3 时钟负
        .ddr3_cke(O_ddr_cke)            ,   //DDR3 时钟使能
        .ddr3_cs_n(O_ddr_cs_n)           ,   //DDR3 片选
        .ddr3_dm(O_ddr_dqm)             ,   //DDR3_dm
        .ddr3_odt(O_ddr_odt)                //DDR3_odt   
    );
    
    assign clk_disp = loc_clk33m;

    // ---------------- IMU 文字叠加 ----------------
    wire [11:0] osd_h;
    wire [11:0] osd_v;
    wire [15:0] osd_rgb;
    wire        osd_en;

    osd_text #(.RAW_MODE(0)) u_osd(
        .clk       (clk_disp),
        .rst_n     (reset_n),
        .de        (rdfifo_rden),
        .h         (osd_h),
        .v         (osd_v),
        .roll_x10  (imu_roll),
        .pitch_x10 (imu_pitch),
        .yaw_x10   (imu_yaw),
        .volt_x100 (volt_x100),
        .enc_cnt   (enc_cnt),
        .imu_zeroing (imu_zeroing),
        .rgb       (osd_rgb),
        .en        (osd_en)
    );

    // ---------------- 九宫格按钮叠加 ----------------
    wire [15:0] btn_rgb;
    wire        btn_en;

    btn_grid u_btn(
        .clk   (clk_disp),
        .rst_n (reset_n),
        .de    (rdfifo_rden),
        .h     (osd_h),
        .v     (osd_v),
        .pressed (tp_pressed),
        .cell_sel (tp_cell),
        .mode_sel (mode_sel),
        .rgb   (btn_rgb),
        .en    (btn_en)
    );

    // ---------------- 触摸光标 ----------------
    wire [15:0] cur_rgb;
    wire        cur_en;
    cursor u_cursor(
        .clk     (clk_disp),
        .rst_n   (reset_n),
        .de      (rdfifo_rden),
        .h       (osd_h),
        .v       (osd_v),
        .pressed (cur_pressed),
        .cx      (cur_x),
        .cy      (cur_y),
        .rgb     (cur_rgb),
        .en      (cur_en)
    );

    // ---------------- YOLO 目标框叠加 ----------------
    wire [15:0] box_rgb;
    wire        box_en;
    box_overlay u_box (
        .clk   (clk_disp),
        .de    (rdfifo_rden),
        .h     (osd_h),
        .v     (osd_v),
        .cnt   (box_cnt),
        .x1    (box_x1),
        .y1    (box_y1),
        .x2    (box_x2),
        .y2    (box_y2),
        .score (box_score),
        .cls   (box_cls),
        .rgb   (box_rgb),
        .en    (box_en)
    );

    wire [15:0] disp_rgb = cur_en ? cur_rgb : (box_en ? box_rgb :
                           (btn_en ? btn_rgb : (osd_en ? osd_rgb : rdfifo_dout)));


    //TFT显示
    disp_driver disp_driver(
		.ClkDisp(clk_disp),
		.Rst_n(reset_n),
		.Data(disp_rgb),
		.DataReq(rdfifo_rden),
		.H_Addr(osd_h),
		.V_Addr(osd_v),
		.Disp_HS(TFT_hs),
		.Disp_VS(TFT_vs),
		.Disp_Red(TFT_rgb[4:0]),
		.Disp_Green(TFT_rgb[10:5]),
		.Disp_Blue(TFT_rgb[15:11]),
        .Frame_Begin(Frame_Begin),
		.Disp_DE(TFT_de),
		.Disp_PCLK(TFT_clk)
	);
    assign TFT_pwm = 1'b1;
	

endmodule