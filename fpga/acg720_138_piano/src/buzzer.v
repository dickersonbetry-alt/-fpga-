module buzzer(
	input clk,
	input rst_n,
	input [31:0] key_value,
	input PointFlag,
	output led,
	output reg pwm
);
	reg [16:0] count;
	reg [16:0] T;
	
	wire flag;
	
	//assign led = !flag;
	assign led = !PointFlag;
	ibeep u_ibeep(
		.Clk		(clk),
		.Rst_n		(rst_n),
		.En_beep	(PointFlag),
		.Beep		(flag)
	);
	
	always @ (posedge clk or negedge rst_n)
	begin
		if (!rst_n)
			begin
				count <= 17'd0;
				pwm <= 1'b1;
			end
		else begin
			if (T==0) begin
				count <= 17'd0;
				pwm <= 1'b1;
			end else begin
				if (flag)
				//if (PointFlag)
					if (count < T)
						count <= count + 1'b1;
					else 
						begin
							count <= 17'd0;
							pwm <= ~pwm;
						end
				else begin
					count <= 17'd0;
					pwm <= 1'b1;
				end
			end
		end
	end
	
	reg [7:0] key;
	always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		key <= 'd0;
	else if (373 < key_value[15:0])
		key <= (5   <= key_value[31:16] && key_value[31:16] < 45 ) ? 'd1 :
			   (50  <= key_value[31:16] && key_value[31:16] < 100) ? 'd2 :
			   (105 <= key_value[31:16] && key_value[31:16] < 145) ? 'd3 :
			   (150 <= key_value[31:16] && key_value[31:16] < 190) ? 'd4 :
			   (195 <= key_value[31:16] && key_value[31:16] < 245) ? 'd5 :
			   (250 <= key_value[31:16] && key_value[31:16] < 300) ? 'd6 :
			   (305 <= key_value[31:16] && key_value[31:16] < 345) ? 'd7 :
			   (350 <= key_value[31:16] && key_value[31:16] < 390) ? 'd8 :
			   (395 <= key_value[31:16] && key_value[31:16] < 445) ? 'd9 :
			   (450 <= key_value[31:16] && key_value[31:16] < 490) ? 'd10:
			   (495 <= key_value[31:16] && key_value[31:16] < 535) ? 'd11:
			   (540 <= key_value[31:16] && key_value[31:16] < 590) ? 'd12:
			   (595 <= key_value[31:16] && key_value[31:16] < 645) ? 'd13:
			   (650 <= key_value[31:16] && key_value[31:16] < 690) ? 'd14:
			   (695 <= key_value[31:16] && key_value[31:16] < 735) ? 'd15:
			   (740 <= key_value[31:16] && key_value[31:16] < 790) ? 'd16:
				'd0;
	else
		key <= 'd0;
	
	always @ (posedge clk or negedge rst_n)
	begin
		if (!rst_n)
			T <= 17'd0;
		else begin
			case (key)
				8'd1:  T <= 17'd95554;		// Z, low tone 1
				8'd2:  T <= 17'd85129;		// X, low tone 2
				8'd3:  T <= 17'd75842;		// C, low tone 3
				8'd4:  T <= 17'd71585;		// V, low tone 4
				8'd5:  T <= 17'd63776;		// B, low tone 5
				8'd6:  T <= 17'd56817;		// N, low tone 6
				8'd7:  T <= 17'd50618;		// M, low tone 7
				8'd8:  T <= 17'd46969;		// A, tone 1
				8'd9:  T <= 17'd42565;		// S, tone 2
				8'd10: T <= 17'd37921;		// D, tone 3
				8'd11: T <= 17'd35792;		// F, tone 4
				8'd12: T <= 17'd31887;		// G, tone 5
				8'd13: T <= 17'd28408;		// H, tone 6
				8'd14: T <= 17'd25308;		// J, tone 7
				8'd15: T <= 17'd23888;		// K, high tone 1
				8'd16: T <= 17'd21282;		// L, high tone 2
				default : T <= 17'd0;
			endcase
		end
	end
	
endmodule 