module edge_check (

	input	wire			clk,
	
	input	wire			wave,
	
	output	wire			flag_neg,
	output	wire			flag_pos
);

	reg						buf_wave;
	reg						reg_check;
	
	always @ (posedge clk) begin
		buf_wave <= wave;
	end
	
	always @ (posedge clk) begin
		reg_check <= buf_wave;
	end
	
	assign flag_neg = reg_check & (~buf_wave);
	assign flag_pos = (~reg_check) & buf_wave;

endmodule 