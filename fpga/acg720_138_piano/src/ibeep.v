/***************************************************
*	Note	:	En_beep来一个脉冲，beep会产生一段时间的
*				脉冲
**************************************************/

module ibeep(
	Clk,
	Rst_n,
	En_beep,
	Beep
);
	input Clk;
	input Rst_n;
    input En_beep;
	output Beep;

	reg [1:0] c_state, n_state;
	reg	[25:0] cnt;

	reg	[14:0] cnt_beep;
	reg	beep_state;
	wire flag_beep;
	
	//parameter T_xs   = 15_000_000;	//change beep time
	parameter T_xs   = 13_000_000;	//change beep time
	
	localparam OFF = 2'b01;
	localparam ON  = 2'b10;
	
	always @ (posedge Clk or negedge Rst_n)
	if (!Rst_n)
		c_state <= OFF;
	else
		c_state <= n_state;
	
	always @ (*)
	case (c_state)
		OFF	:	if (En_beep == 1'b1)
						n_state <= ON;
					else
						n_state <= OFF;
						
		ON	:	if (cnt < T_xs - 1'b1)
						n_state <= ON;
					else
						n_state <= OFF;
						
		default	:		n_state <= OFF;
	endcase
	
	always @ (posedge Clk or negedge Rst_n)
	if (!Rst_n)
		cnt <= 'd0;
	else
		case (c_state)
			OFF	:	cnt <= 'd0;
						
			ON	:	if (cnt < T_xs - 1'b1)
						cnt <= cnt + 1'b1;
					else
						cnt <= 'd0;
				
			default	:	cnt <= 'd0;
		endcase
	
	always @ (posedge Clk or negedge Rst_n)
	if(!Rst_n)
		beep_state <= 1'b0;
	else
		if (c_state == ON)
			beep_state <= 1'b1;
		else
			beep_state <= ~beep_state;
	
	assign Beep = beep_state;

endmodule 