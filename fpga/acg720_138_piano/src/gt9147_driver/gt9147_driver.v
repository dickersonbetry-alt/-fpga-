//-------------------------------------------------------------------
//
//  gt9147_driver
//
//	*****************************************************************
//	* power on 
//	* IIC addr:0x28/0x29
// 	*                               _________________
//	* tp_rst_______________________|           |
// 	*                 _____________|___________|
//	* tp_int_________|-  >100us   -|-  >5ms   -|
//	*
//	*****************************************************************
//
//	*****************************************************************
//	* power on 
//	* IIC addr:0xba/0xbb
// 	*                               _________________
//	* tp_rst_______________________|           |
//	*                |-  >100us   -|-  >5ms   -|
//	* tp_int_________|_____________|___________|
//	*
//	*****************************************************************
//
//-------------------------------------------------------------------

module gt9147_driver(
	input 	clk,
	input 	rst_n,
	
	output seg_sclk,
	output seg_rclk,
	output seg_dio,
	
	output reg [31:0] position,
	output reg        PointFlag,
	//output            PointFlag,
	
	output 	tp_scl,//g15
	inout 	tp_sda,//g11
	inout 	tp_int,//f13
	output  tp_rst //f14
);
	wire set_id_done;
	gt9147_set_id u_gt9147_set_id(
		.clk			(clk),
		.rst_n			(rst_n),
		.tp_int			(tp_int),
		.tp_rst			(tp_rst),
		.set_id_done	(set_id_done)
	);
	
	reg rdreg_req;
	reg [3:0] rdreg_num;//读取寄存器的个数
	reg wrreg_req;
	reg [15:0] addr;
	wire [31:0] rddata;
	reg [7:0] wrdata;
	wire RW_Done;
	localparam addr_mode = 1'b1;
	localparam device_id = 8'h28;
	//localparam device_id = 8'hba;
	i2c_control i2c_control(
		.Clk		(clk		),
		.Rst_n		(rst_n		),
		.wrreg_req	(wrreg_req	),
		.rdreg_req	(rdreg_req	),
		.rdreg_num  (rdreg_num	),
		.addr		(addr		),
		.addr_mode	(addr_mode	),
		.wrdata		(wrdata		),
		.rddata		(rddata		),
		.device_id	(device_id	),
		.RW_Done	(RW_Done	),
		.ack		(			),		
		.i2c_sclk	(tp_scl		),
		.i2c_sdat	(tp_sda		)
	);
	
//	issp issp_inst(
//		.probe ( state ),
//		.source (  )
//	);
	
	localparam TouchStateReg  = 16'h814E, //GT9147当前触摸情况寄存器地址
	           TouchPoint1Reg = 16'h8150, //GT9147触摸点1寄存器起始地址
	           TouchPoint2Reg = 16'h8158, //GT9147触摸点2寄存器起始地址
	           TouchPoint3Reg = 16'h8160, //GT9147触摸点3寄存器起始地址
	           TouchPoint4Reg = 16'h8168, //GT9147触摸点4寄存器起始地址
	           TouchPoint5Reg = 16'h8170; //GT9147触摸点5寄存器起始地址
	
	localparam IDLE             = 8'h01,  //空闲状态
			   WAIT_INT         = 8'h02,  //等待触摸中断信号(触摸操作)
			   READ_STATE       = 8'h04,  //读取触摸状态寄存器
			   STATE_CHECK      = 8'h08,  //触摸状态寄存器值判断
			   READ_POINT       = 8'h10,  //读取坐标值
			   READ_POINT_DONE  = 8'h20,  //读取坐标值完成
			   CLEAN_STATE      = 8'h40,  //清除状态寄存器
			   CLEAN_STATE_DONE = 8'h80;  //清除状态寄存器完成
	
	reg [7:0] state;
	reg [7:0] TouchState;	//触摸状态寄存器
	reg [3:0] cnt_point;	//当前触点个数
	reg [3:0] cnt;			//当前读取触点
	reg [15:0] PonitX [0:4];//当前坐标x值，5个点，16位
	reg [15:0] PonitY [0:4];//当前坐标y值，5个点，16位
	reg PointState;			//高电平,有按键值输出
	wire tp_int_state;		//高电平,有触点按下
	
	always @ (*) begin
		PointState = (cnt_point > 0) ? 1'b1 : 1'b0;
		PointFlag  = (cnt_point > 0) ? 1'b1 : 1'b0;
	end
	
	always @ (posedge clk or negedge rst_n)
	if (!rst_n) begin
		state <= IDLE;
		rdreg_req <= 1'b0;
		wrreg_req <= 1'b0;
		TouchState <= 8'd0;
		wrdata <= 8'd0;
		cnt_point <= 'd0;
		cnt <= 'd0;
		rdreg_num <= 4'd0;
		//PointState <= 1'b0;
	end else
		case (state)
			IDLE:
				if (set_id_done)
					state <= WAIT_INT;
				else
					state <= IDLE;
			WAIT_INT:
				begin
					cnt_point <= 'd0;
					if (tp_int_state) begin//tp_int
					//if (tp_int) begin
						rdreg_req <= 1'b1;
						rdreg_num <= 4'd1;
						addr <= TouchStateReg;
						state <= READ_STATE;
					end
				end
			READ_STATE:
				begin
					rdreg_req <= 1'b0;
					if (RW_Done) begin
						state <= STATE_CHECK;
						TouchState <= rddata[31:24];
					end
				end
			STATE_CHECK:
				begin
					if (TouchState[7]) begin
						cnt_point <= TouchState[3:0];
						cnt <= 'd0;
						state <= READ_POINT;
					end else begin
						state <= CLEAN_STATE;//清除寄存器状态
						rdreg_req <= 1'b0;
						wrreg_req <= 1'b0;
					end
				end
			READ_POINT:
				begin
					if (cnt < cnt_point) begin
						rdreg_num <= 4'd4;
						rdreg_req <= 1'b1;
						state <= READ_POINT_DONE;
						//PointState <= 1'b1;
						case (cnt)
							'd0: begin addr <= TouchPoint1Reg; end
							'd1: begin addr <= TouchPoint2Reg; end
							'd2: begin addr <= TouchPoint3Reg; end
							'd3: begin addr <= TouchPoint4Reg; end
							'd4: begin addr <= TouchPoint5Reg; end
						endcase
					end else begin
						state <= CLEAN_STATE;//清除寄存器状态
						//PointState <= 1'b0;
						rdreg_req <= 1'b0;
						wrreg_req <= 1'b0;
					end
				end
			READ_POINT_DONE:
				begin
					rdreg_req <= 1'b0;
					if (RW_Done) begin
						{PonitY[cnt], PonitX[cnt]} <= rddata;
						cnt <= cnt + 1'b1;
						state <= READ_POINT;
					end
				end
			CLEAN_STATE:
				begin
					wrreg_req <= 1'b1;
					addr <= TouchStateReg;
					wrdata <= 8'd0;
					state <= CLEAN_STATE_DONE;
				end
			CLEAN_STATE_DONE:
				begin
					wrreg_req <= 1'b0;
					if (RW_Done)
					state <= WAIT_INT;
				end
			default : state <= CLEAN_STATE;
		endcase
	
//	reg flag;
//	ibeep u_ibeep(
//		.Clk		(clk),
//		.Rst_n		(rst_n),
//		.En_beep	(flag),
//		.Beep		(PointFlag)
//	);
	
	always @ (posedge clk or negedge rst_n)
	if (!rst_n) begin
		position <= 'd0;
		//PointFlag <= 1'b0;
		
		//flag <= 1'b0;
	end else if (PointState) begin
		position <= disp_data;
		//PointFlag <= 1'b1;
		
		//flag <= 1'b1;
	end
	//below 2 line for clean point
	
	//else
		//PointFlag <= 1'b0;
		
		//flag <= 1'b0;
	
	//-------------------------------------------------
	// 以下为非驱动关键部分，显示坐标值作用
	//-------------------------------------------------
	key_filter u_key_filter0(
		.Clk		(clk			),
		.Rst_n		(rst_n			),
		.key_in		(tp_int			),
		.key_flag	(				),
		.key_state	(tp_int_state	)
	);
	
	wire [15:0] n0;
	wire [15:0] n0_bcd;
	wire [15:0] n1;
	wire [15:0] n1_bcd;
	
	assign n0 = disp_data[31:16];
	assign n1 = disp_data[15:0];
	bintobcd #(.WIDTH_BIN(16), .WIDTH_BCD(16))//WIDTH_BIN:二进制的宽度;WIDTH_BCD;转换的BCD码的宽度
		bintobcd_inst0(.bin(n0), .bcd(n0_bcd));
	
	bintobcd #(.WIDTH_BIN(16), .WIDTH_BCD(16))//WIDTH_BIN:二进制的宽度;WIDTH_BCD;转换的BCD码的宽度
		bintobcd_inst1(.bin(n1), .bcd(n1_bcd));
	
	reg [31:0]  disp_data;
	always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		disp_data <= 'd0;
	else if (PointState)
		disp_data <= {PonitX[0], PonitY[0]};
//	else
//		disp_data <= 'd0;
	
	HEX_top u_HEX_top(
		.Clk		(clk			),
		.Rst_n		(rst_n			),
		.disp_data	({n0_bcd,n1_bcd}),
		.SEG7_SCLK	(seg_sclk		),
		.SEG7_RCLK	(seg_rclk		),
		.SEG7_DIO	(seg_dio		)
	);
	
endmodule

//-------------------------------------------------------------------
//
//  gt9147_set_id
//
//	*****************************************************************
//	* power on 
//	* IIC addr:0x28/0x29
// 	*                               _________________
//	* tp_rst_______________________|           |
// 	*                 _____________|___________|
//	* tp_int_________|-  >100us   -|-  >5ms   -|
//	*
//	*****************************************************************
//
//	*****************************************************************
//	* power on 
//	* IIC addr:0xba/0xbb
// 	*                               _________________
//	* tp_rst_______________________|           |
//	*                |-  >100us   -|-  >5ms   -|
//	* tp_int_________|_____________|___________|
//	*
//	*****************************************************************
//
//-------------------------------------------------------------------

module gt9147_set_id(
	input 	   clk,
	input 	   rst_n,
	
	inout 	   tp_int,
	output reg tp_rst,
	
	output reg set_id_done  //为高表示，设置触摸屏芯片的IIC地址为0x28
);
	//---------------------------------------------------------
	// 设置触摸屏芯片的IIC地址为0x28
	//---------------------------------------------------------
	reg tp_int_oe, tp_int_od;
	assign tp_int = tp_int_oe ? tp_int_od : 1'bz;
	
	//reg set_id_done;//为高表示，设置触摸屏芯片的IIC地址为0x28
	reg [2:0] state_init;
	reg [25:0] cnt_dly;
	always @ (posedge clk or negedge rst_n)
	if (!rst_n) begin
		state_init <= 'd0;
		cnt_dly <= 'd0;
		tp_int_oe <= 1'b0;
		tp_int_od <= 1'b0;
		tp_rst <= 1'b0;
		set_id_done <= 1'b0;
	end else
		case (state_init)
			'd0://RST和INT都为0,延时10ms
				begin
					tp_int_oe <= 1'b1;
					tp_int_od <= 1'b0;
					tp_rst <= 1'b0;
					set_id_done <= 1'b0;
					if (cnt_dly < 'd500_000) begin
						cnt_dly <= cnt_dly + 1'b1;
						state_init <= 'd0;
					end else begin
						cnt_dly <= 'd0;
						state_init <= 'd1;
					end
				end
			'd1://INT拉高为1,RST为0,延时200us
				begin
					tp_int_oe <= 1'b1;
					tp_int_od <= 1'b1;
					tp_rst <= 1'b0;
					if (cnt_dly < 'd10_000)
						cnt_dly <= cnt_dly + 1'b1;
					else begin
						cnt_dly <= 'd0;
						state_init <= 'd2;
					end
				end
			'd2://RST设为1，INT为1，完成将ID设置为0x28的功能,延时6ms
				begin
					tp_int_oe <= 1'b1;
					tp_int_od <= 1'b1;
					tp_rst <= 1'b1;
					
					if (cnt_dly < 'd300_000)
						cnt_dly <= cnt_dly + 1'b1;
					else begin
						cnt_dly <= 'd0;
						state_init <= 'd3;
					end
				end
			'd3://设置Int信号为输入，RST继续为输出
				begin
					set_id_done <= 1'b1;
					state_init <= 'd3;
					tp_int_oe <= 1'b0;
					tp_rst <= 1'b1;
				end
			default:
				begin
					tp_int_oe <= 1'b0;
					tp_rst <= 1'b1;
					set_id_done <= 1'b0;
					state_init <= 'd3;
				end
		endcase
	
endmodule
