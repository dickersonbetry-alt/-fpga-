module shift_adjust #(
	parameter WIDTH_BIN = 15,
	parameter WIDTH_BCD = 20
	)(
	idata, 
	odata
	);
	
	input [WIDTH_BIN + WIDTH_BCD - 1:0] idata;
        output [WIDTH_BIN + WIDTH_BCD - 1:0] odata;

	wire [WIDTH_BIN + WIDTH_BCD - 1:0] temp;
	
	genvar i;
	
	assign temp = idata << 1'b1;
	
	generate
		for (i = 0; i < WIDTH_BCD / 4; i = i + 1)
                begin : a
			adjust adjust_inst0(
				.idata(temp[WIDTH_BIN + WIDTH_BCD - 1 - 4*i : WIDTH_BIN + WIDTH_BCD - 4 - 4*i]),
				.odata(odata[WIDTH_BIN + WIDTH_BCD - 1 - 4*i : WIDTH_BIN + WIDTH_BCD - 4 - 4*i])
			);
		end
	endgenerate
	
	assign odata[WIDTH_BIN-1:0] = temp[WIDTH_BIN-1:0];

endmodule