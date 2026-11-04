module key_filters (

	input	wire					clk,
	input	wire					rst_n,
	
	input	wire					key, // low valid
	
	output	reg						key_state // low valid
);

	parameter		MASK_TIME	=	500_000;//10ms

	localparam		KEY_OFF		=	4'b0001;//∂¿»»¬Î
	localparam		ON_SHAKE	=	4'b0010;
	localparam		KEY_ON		=	4'b0100;
	localparam		OFF_SHAKE	=	4'b1000;
	
	reg				[3:0]			c_state;
	reg				[3:0]			n_state;
	
	reg				[18:0]			cnt;		

	reg				[1:0]			buf_key;
	wire							key_n;
	
	always @ (posedge clk) buf_key <= {buf_key[0], key};
	assign key_n = buf_key[1];
	
	always @ (posedge clk, negedge rst_n) begin
		if (rst_n == 1'b0)
			c_state <= KEY_OFF;
		else
			c_state <= n_state;
	end
	
	always @ * begin
		case (c_state)
			KEY_OFF		:	if (key_n == 1'b0)
								n_state = ON_SHAKE;
							else
								n_state = KEY_OFF;
			
			ON_SHAKE	:	if (key_n == 1'b1)
								n_state = KEY_OFF;
							else	
								if (cnt < MASK_TIME - 1'b1)
									n_state = ON_SHAKE;
								else
									n_state = KEY_ON;
			
			KEY_ON		:	if (key_n == 1'b1)
								n_state = OFF_SHAKE;
							else
								n_state = KEY_ON;
			
			OFF_SHAKE	:	if (key_n == 1'b0)
								n_state = KEY_ON;
							else
								if (cnt < MASK_TIME - 1'b1)
									n_state = OFF_SHAKE;
								else
									n_state = KEY_OFF;
			
			default		:	n_state = KEY_OFF;
		endcase
	end
	
	
	always @ (posedge clk, negedge rst_n) begin
		if (rst_n == 1'b0)
			cnt <= 19'd0;
		else	
			case (c_state)
				KEY_OFF		:	cnt <= 19'd0;
				
				ON_SHAKE	:	if (key_n == 1'b1)
									cnt <= 19'd0;
								else
									if (cnt < MASK_TIME - 1'b1)
										cnt <= cnt + 1'b1;
									else
										cnt <= 19'd0;
										
				KEY_ON		:	cnt <= 19'd0;
				
				OFF_SHAKE	:	if (key_n == 1'b0)
									cnt <= 19'd0;
								else	
									if (cnt < MASK_TIME - 1'b1)
										cnt <= cnt + 1'b1;
									else
										cnt <= 19'd0;
										
				default		:	cnt <= 19'd0;
				
			endcase
	end
	
	always @ (posedge clk, negedge rst_n) begin
		if (rst_n == 1'b0)
			key_state <= 1'b1;
		else
			if (c_state == KEY_ON || c_state == OFF_SHAKE)
				key_state <= 1'b0;
			else
				key_state <= 1'b1;
	end
	
endmodule 