module key(
	input		wire		clk,
	input 		wire 		rst_n, 
	
	input		wire		key_mode,
	output		wire		en_mode
);
	
	wire					key_state1;
	
	key_filters	key_filter_mode (
			.clk			(clk),
			.rst_n			(rst_n),
			.key			(key_mode), // low valid
			.key_state		(key_state1) // low valid
		);
		
	edge_check	edge_check_mode (
			.clk			(clk),
			.wave			(key_state1),
			.flag_neg		(en_mode),
			.flag_pos		()
		);
	
endmodule 