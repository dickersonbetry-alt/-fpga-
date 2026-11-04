module piano(
	input 			clk,
	input 			rst_n,
	
	// key debug line
	input 			key,//按下后显示当前触摸坐标线位置
	
//	//seg show
	output 			seg_sclk,
	output 			seg_rclk,
	output 			seg_dio,
	
	//tft output
	output  [15:0]  tft_rgb,  //TFT数据输出
	output          tft_hs,   //TFT行同步信号
	output          tft_vs,   //TFT场同步信号
	output          tft_clk,  //TFT像素时钟
	output          tft_de,   //TFT数据使能
	output          tft_pwm,  //TFT背光控制
	//tft touch
	output 			tp_scl,
	inout 			tp_sda,
	inout 			tp_int,
	output  		tp_rst,
	
	//beep
	output			beep
);
	wire clk_50m;
	wire clk_vga;//33m

    Gowin_PLL Gowin_PLL(
        .clkin(clk), //input  clkin
        .init_clk(clk), //input  init_clk
        .clkout0(clk_50m), //output  clkout0
        .clkout1(clk_vga), //output  clkout1
        .lock(lock), //output  lock
        .reset(~rst_n) //input  reset
);

	
	wire [31:0] position;
	wire        PointFlag;
	gt9147_driver u_gt9147_driver(
		.clk			(clk_vga				),
		.rst_n			(rst_n					),
		.seg_sclk		(seg_sclk				),
		.seg_rclk		(seg_rclk				),
		.seg_dio		(seg_dio				),
		.position		(position				),
		.PointFlag		(PointFlag				),
		.tp_scl			(tp_scl					),
		.tp_sda			(tp_sda					),
		.tp_int			(tp_int					),
		.tp_rst			(tp_rst					)
	);
	
	wire key_line;
	key key_inst(
		.clk			(clk_50m				),
		.rst_n			(rst_n					),
		.key_mode		(key					),
		.en_mode		(key_line				)
	);
	
	wire [11:0] H_Addr;
	wire [11:0] V_Addr;
	wire [23:0] Data;
	piano_ctrl u_piano_ctrl(
		.clk			(clk_vga				),
		.rst_n			(rst_n					),
		.key_line		(key_line				),
		.position		(position				),
		.PointFlag		(PointFlag				),
		.h_addr			(H_Addr					),
		.v_addr			(V_Addr					),
		.de				(video_de				),
		.data			(Data					)
	);
	
	buzzer u_buzzer(
		.clk			(clk_50m				), 
		.rst_n			(rst_n					),
		.key_value		(position				),
		.PointFlag		(PointFlag				),
		.led			(led),
		.pwm			(beep					)
	);
	
	wire        video_hs;
	wire        video_vs;
	wire        video_de;
	wire        video_clk;
	wire [7:0]  video_r;
	wire [7:0]  video_g;
	wire [7:0]  video_b;
	
	disp_driver disp_driver(
		.ClkDisp		(clk_vga				),
		.Rst_n			(rst_n					),
		.Data			(Data					),
		.DataReq		(        				),
		.H_Addr			(H_Addr					),
		.V_Addr			(V_Addr					),
		.Disp_HS		(video_hs				),
		.Disp_VS		(video_vs				),
		.Disp_Red		(video_r				),
		.Disp_Green		(video_g				),
		.Disp_Blue		(video_b				),
		.Disp_DE		(video_de				),
		.Disp_PCLK		(video_clk				)
	);
	
	//tft display
	assign tft_rgb = {video_r[7:3], video_g[7:2], video_b[7:3]};  //TFT数据输出
	assign tft_hs = video_hs;
	assign tft_vs = video_vs;
	assign tft_clk = video_clk;
	assign tft_de = video_de;
	assign tft_pwm = rst_n;

endmodule
