/********************************************************************************************
*	Usage :
*	bintobcd #(.WIDTH_BIN(7), .WIDTH_BCD(8))
*			bintobcd_inst0(.bin(number1), .bcd(number1_bcd));
*		//WIDTH_BIN:二进制的宽度;WIDTH_BCD;转换的BCD码的宽度
********************************************************************************************/
module bintobcd #(
	parameter WIDTH_BIN = 8,
	parameter WIDTH_BCD = 12
	)(
	bin,
	bcd
	);
	input [WIDTH_BIN - 1:0] bin;
        output [WIDTH_BCD - 1:0] bcd;
	
	wire [WIDTH_BIN + WIDTH_BCD - 1:0] temp [WIDTH_BIN:0];
	
	genvar i;
		
	assign temp[0] = {{WIDTH_BCD{1'b0}}, bin};
	
	generate
		for(i = 0; i < WIDTH_BIN - 1; i = i + 1) 
                begin : s_a
			shift_adjust #(.WIDTH_BIN(WIDTH_BIN),.WIDTH_BCD(WIDTH_BCD)) 
				shift_adjust_inst(.idata(temp[i]),.odata(temp[i+1]));
		end
	endgenerate
	
	assign temp[WIDTH_BIN] = temp[WIDTH_BIN - 1] << 1'b1;
	assign bcd = temp[WIDTH_BIN][WIDTH_BIN + WIDTH_BCD - 1:WIDTH_BIN];

endmodule