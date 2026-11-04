/***************************************************
*	Module Name		:	HEX8		   
*	Engineer		:	小梅哥
*	Target Device	:	EP4CE10F17C8
*	Tool versions	:	Quartus II 13.0
*	Create Date		:	2017-3-31
*	Revision		:	v1.0
*	Description		:  8位7段数码管显示设计
**************************************************/

module HEX8(
		Clk,
		Rst_n,
		En,
		disp_data,
		sel,
		seg
	);

	input Clk;	//50M
	input Rst_n;
	input En;	//数码管显示使能，1使能，0关闭
	
	input [31:0]disp_data;
	
	output [7:0] sel;//数码管位选（选择当前要显示的数码管）
	output reg [7:0] seg;//数码管段选（当前要显示的内容）
	
	reg [14:0]divider_cnt;//25000-1
	
	reg clk_1K;
	reg [7:0]sel_r;
	
	reg [4:0]data_tmp;//数据缓存 ;data_tmp[4]用来控制DP显示

//	分频计数器计数模块
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		divider_cnt <= 15'd0;
	else if(!En)
		divider_cnt <= 15'd0;
	else if(divider_cnt == 24999)
		divider_cnt <= 15'd0;
	else
		divider_cnt <= divider_cnt + 1'b1;

//1K扫描时钟生成模块		
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		clk_1K <= 1'b0;
	else if(divider_cnt == 24999)
		clk_1K <= ~clk_1K;
	else
		clk_1K <= clk_1K;
		
//8位循环移位寄存器
	always@(posedge clk_1K or negedge Rst_n)
	if(!Rst_n)
		sel_r <= 8'b0000_0001;
	else if(sel_r == 8'b1000_0000)
		sel_r <= 8'b0000_0001;
	else
		sel_r <=  sel_r << 1;
		
	always@(*)
		case(sel_r)
			8'b0000_0001:data_tmp = {1'b1, disp_data[3:0]};
			8'b0000_0010:data_tmp = {1'b1, disp_data[7:4]};//1'b0
			8'b0000_0100:data_tmp = {1'b1, disp_data[11:8]};
			8'b0000_1000:data_tmp = {1'b1, disp_data[15:12]};
			8'b0001_0000:data_tmp = {1'b0, disp_data[19:16]};//1'b1
			8'b0010_0000:data_tmp = {1'b1, disp_data[23:20]};
			8'b0100_0000:data_tmp = {1'b1, disp_data[27:24]};
			8'b1000_0000:data_tmp = {1'b1, disp_data[31:28]};
			default:data_tmp = 5'b00000;
		endcase
		
	always@(*) begin
		case(data_tmp[3:0])
			4'h0:seg[6:0] = 7'b1000000; /*0*/
			4'h1:seg[6:0] = 7'b1111001; /*1*/
			4'h2:seg[6:0] = 7'b0100100; /*2*/
			4'h3:seg[6:0] = 7'b0110000; /*3*/
			4'h4:seg[6:0] = 7'b0011001; /*4*/
			4'h5:seg[6:0] = 7'b0010010; /*5*/
			4'h6:seg[6:0] = 7'b0000010; /*6*/
			4'h7:seg[6:0] = 7'b1111000; /*7*/
			4'h8:seg[6:0] = 7'b0000000; /*8*/
			4'h9:seg[6:0] = 7'b0010000; /*9*/
			4'ha:seg[6:0] = 7'b0001000; /*a*/
			4'hb:seg[6:0] = 7'b0000011; /*b*/
			4'hc:seg[6:0] = 7'b1000110; /*c*/
			4'hd:seg[6:0] = 7'b0100001; /*d*/
			4'he:seg[6:0] = 7'b0000110; /*e*/
			4'hf:seg[6:0] = 7'b0001110; /*f*/
		endcase
		seg[7] = data_tmp[4];	/*用来控制小数点*/
	end
	
	assign sel = (En)?sel_r:8'b0000_0000;

endmodule
