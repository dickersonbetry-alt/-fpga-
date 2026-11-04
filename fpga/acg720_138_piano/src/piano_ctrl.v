`include "vga/disp_parameter_cfg.v"
module piano_ctrl(
	input 	          clk,
	input 	          rst_n,
	input 	          key_line,
	input      [31:0] position,//like key
	input             PointFlag,
	input      [11:0] h_addr,
	input      [11:0] v_addr,
	input             de,
	output reg [23:0] data
);
	reg black_area;	//ºÚ°×¼üÇøÓò
	reg key_area;	//°´¼üÇøÓò
	
	parameter PINK  = 24'h9350BC, //·ÛºìÉ«
			  GREEN = 24'h00FFFF, //ÂÌÉ«
			  WHITE = 24'hFFFFFF, //°×
			  GRAY  = 24'hC8C8C8, //»Ò
			  RED   = 24'hFF0000, //ºì
			  BLACK = 24'h000000; //ºÚ
	
	reg show_line;
	always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		show_line <= 1'b0;
	else if (key_line)
		show_line <= ~show_line;
	
	//»®·ÖÇøÓò
	always @ (posedge clk or negedge rst_n)
	if (!rst_n) begin
		black_area <= 1'b0;
		key_area <= 1'b0;	
	end else begin
		if((159 < v_addr) && (v_addr < 374))
			black_area <= 1'b1;
		else if((373 < v_addr) && (v_addr <= 480))
			key_area <= 1'b1;
		else begin
			black_area <= 1'b0;
			key_area <= 1'b0;
		end
	end
	
	wire flag;
	ibeep u_ibeep(
		.Clk		(clk),
		.Rst_n		(rst_n),
		.En_beep	(PointFlag),
		.Beep		(flag)
	);
	
	reg [7:0] key;
	always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		key <= 'd0;
	else if (flag)
		if (373 < position[15:0])
			key <= (5   <= position[31:16] && position[31:16] < 45 ) ? 'd1 :
				   (50  <= position[31:16] && position[31:16] < 100) ? 'd2 :
				   (105 <= position[31:16] && position[31:16] < 145) ? 'd3 :
				   (150 <= position[31:16] && position[31:16] < 190) ? 'd4 :
				   (195 <= position[31:16] && position[31:16] < 245) ? 'd5 :
				   (250 <= position[31:16] && position[31:16] < 300) ? 'd6 :
				   (305 <= position[31:16] && position[31:16] < 345) ? 'd7 :
				   (350 <= position[31:16] && position[31:16] < 390) ? 'd8 :
				   (395 <= position[31:16] && position[31:16] < 445) ? 'd9 :
				   (450 <= position[31:16] && position[31:16] < 490) ? 'd10:
				   (495 <= position[31:16] && position[31:16] < 535) ? 'd11:
				   (540 <= position[31:16] && position[31:16] < 590) ? 'd12:
				   (595 <= position[31:16] && position[31:16] < 645) ? 'd13:
				   (650 <= position[31:16] && position[31:16] < 690) ? 'd14:
				   (695 <= position[31:16] && position[31:16] < 735) ? 'd15:
				   (740 <= position[31:16] && position[31:16] < 790) ? 'd16:
					'd0;
		else
			key <= 'd0;
	else
		key <= 'd0;
	
	// black_area
	wire B_BLACK0  = (0   <= h_addr && h_addr < 5  );
	wire B_WHITE0  = (5   <= h_addr && h_addr < 35 );
	wire B_BLACK1  = (35  <= h_addr && h_addr < 60 );
	wire B_WHITE1  = (60  <= h_addr && h_addr < 90 );
	wire B_BLACK2  = (90  <= h_addr && h_addr < 115);
	wire B_WHITE2  = (115 <= h_addr && h_addr < 145);
	wire B_BLACK3  = (145 <= h_addr && h_addr < 150);
	wire B_WHITE3  = (150 <= h_addr && h_addr < 180);
	wire B_BLACK4  = (180 <= h_addr && h_addr < 205);
	wire B_WHITE4  = (205 <= h_addr && h_addr < 235);
	wire B_BLACK5  = (235 <= h_addr && h_addr < 260);
	wire B_WHITE5  = (260 <= h_addr && h_addr < 290);
	wire B_BLACK6  = (290 <= h_addr && h_addr < 315);
	wire B_WHITE6  = (315 <= h_addr && h_addr < 345);
	wire B_BLACK7  = (345 <= h_addr && h_addr < 350);
	wire B_WHITE7  = (350 <= h_addr && h_addr < 380);
	wire B_BLACK8  = (380 <= h_addr && h_addr < 405);
	wire B_WHITE8  = (405 <= h_addr && h_addr < 435);
	wire B_BLACK9  = (435 <= h_addr && h_addr < 460);
	wire B_WHITE9  = (460 <= h_addr && h_addr < 490);
	wire B_BLACK10 = (490 <= h_addr && h_addr < 495);
	wire B_WHITE10 = (495 <= h_addr && h_addr < 525);
	wire B_BLACK11 = (525 <= h_addr && h_addr < 550);
	wire B_WHITE11 = (550 <= h_addr && h_addr < 580);
	wire B_BLACK12 = (580 <= h_addr && h_addr < 605);
	wire B_WHITE12 = (605 <= h_addr && h_addr < 635);
	wire B_BLACK13 = (635 <= h_addr && h_addr < 660);
	wire B_WHITE13 = (660 <= h_addr && h_addr < 690);
	wire B_BLACK14 = (690 <= h_addr && h_addr < 695);
	wire B_WHITE14 = (695 <= h_addr && h_addr < 725);
	wire B_BLACK15 = (725 <= h_addr && h_addr < 750);
	wire B_WHITE15 = (750 <= h_addr && h_addr < 780);
	wire B_BLACK16 = (780 <= h_addr && h_addr < 795);
	wire B_WHITE16 = (795 <= h_addr && h_addr < 800);
	// key_area
	wire K_BLACK0  = (0   <= h_addr && h_addr < 5  );
	wire K_WHITE0  = (5   <= h_addr && h_addr < 45 );
	wire K_BLACK1  = (45  <= h_addr && h_addr < 50 );
	wire K_WHITE1  = (50  <= h_addr && h_addr < 100);
	wire K_BLACK2  = (100 <= h_addr && h_addr < 105);
	wire K_WHITE2  = (105 <= h_addr && h_addr < 145);
	wire K_BLACK3  = (145 <= h_addr && h_addr < 150);
	wire K_WHITE3  = (150 <= h_addr && h_addr < 190);
	wire K_BLACK4  = (190 <= h_addr && h_addr < 195);
	wire K_WHITE4  = (195 <= h_addr && h_addr < 245);
	wire K_BLACK5  = (245 <= h_addr && h_addr < 250);
	wire K_WHITE5  = (250 <= h_addr && h_addr < 300);
	wire K_BLACK6  = (300 <= h_addr && h_addr < 305);
	wire K_WHITE6  = (305 <= h_addr && h_addr < 345);
	wire K_BLACK7  = (345 <= h_addr && h_addr < 350);
	wire K_WHITE7  = (350 <= h_addr && h_addr < 390);
	wire K_BLACK8  = (390 <= h_addr && h_addr < 395);
	wire K_WHITE8  = (395 <= h_addr && h_addr < 445);
	wire K_BLACK9  = (445 <= h_addr && h_addr < 450);
	wire K_WHITE9  = (450 <= h_addr && h_addr < 490);
	wire K_BLACK10 = (490 <= h_addr && h_addr < 495);
	wire K_WHITE10 = (495 <= h_addr && h_addr < 535);
	wire K_BLACK11 = (535 <= h_addr && h_addr < 540);
	wire K_WHITE11 = (540 <= h_addr && h_addr < 590);
	wire K_BLACK12 = (590 <= h_addr && h_addr < 595);
	wire K_WHITE12 = (595 <= h_addr && h_addr < 645);
	wire K_BLACK13 = (645 <= h_addr && h_addr < 650);
	wire K_WHITE13 = (650 <= h_addr && h_addr < 690);
	wire K_BLACK14 = (690 <= h_addr && h_addr < 695);
	wire K_WHITE14 = (695 <= h_addr && h_addr < 735);
	wire K_BLACK15 = (735 <= h_addr && h_addr < 740);
	wire K_WHITE15 = (740 <= h_addr && h_addr < 790);
	wire K_BLACK16 = (790 <= h_addr && h_addr < 795);
	wire K_WHITE16 = (795 <= h_addr && h_addr < 800);
	
	//»­¸ÖÇÙ
	always @ (posedge clk or negedge rst_n)
	if (!rst_n)
		data <= BLACK; //ºÚ
	else begin
		if (de)
			if (show_line == 1'b1 && (h_addr == position[27:16] | v_addr == position[11:0]))//¹â±êÎ»ÖÃ(cursor location)
				data <= RED;
			else if ((!black_area) && (!key_area))//²»ÔÚÇøÓò1ÇÒ²»ÔÚÇøÓò2
				data <= GREEN;
			else if (black_area) begin    //ÔÚ¿Õ°×ÇøÓòµÄÊ±ºò
				if (B_BLACK0)
					data <= GREEN;
				else if (B_WHITE0)
					begin 
						if(key == 8'd1)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK1)
					data <= BLACK;
				else if (B_WHITE1)
					begin 
						if(key == 8'd2)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK2)
					data <= BLACK;
				else if (B_WHITE2)
					begin 
						if(key == 8'd3)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK3)
					data <= BLACK;
				else if (B_WHITE3)
					begin 
						if(key == 8'd4)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK4)
					data <= BLACK;
				else if (B_WHITE4)
					begin 
						if(key == 8'd5)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK5)
					data <= BLACK;
				else if (B_WHITE5)
					begin 
						if(key == 8'd6)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK6)
					data <= BLACK;
				else if (B_WHITE6)
					begin 
						if(key == 8'd7)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK7)
					data <= BLACK;
				else if (B_WHITE7)
					begin 
						if(key == 8'd8)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK8)
					data <= BLACK;
				else if (B_WHITE8)
					begin 
						if(key == 8'd9)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK9)
					data <= BLACK;
				else if (B_WHITE9)
					begin 
						if(key == 8'd10)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK10)
					data <= BLACK;
				else if (B_WHITE10)
					begin 
						if(key == 8'd11)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK11)
					data <= BLACK;
				else if (B_WHITE11)
					begin 
						if(key == 8'd12)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK12)
					data <= BLACK;
				else if (B_WHITE12)
					begin 
						if(key == 8'd13)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK13)
					data <= BLACK;
				else if (B_WHITE13)
					begin 
						if(key == 8'd14)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (B_BLACK14)
					data <= BLACK;
				else if (B_WHITE14)
					begin 
						if(key == 8'd15)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK15)
					data <= BLACK;
				else if (B_WHITE15)
					begin 
						if(key == 8'd16)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (B_BLACK16)
					data <= BLACK;
				else if (B_WHITE16)
					data <= GREEN;
				else
					data <= BLACK;
			end else if (key_area) begin
				if (K_BLACK0)
					data <= GREEN;
				else if (K_WHITE0)
					begin 
						if(key == 8'd1)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK1)
					data <= BLACK;
				else if (K_WHITE1)
					begin 
						if(key == 8'd2)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK2)
					data <= BLACK;
				else if (K_WHITE2)
					begin 
						if(key == 8'd3)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK3)
					data <= BLACK;
				else if (K_WHITE3)
					begin 
						if(key == 8'd4)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK4)
					data <= BLACK;
				else if (K_WHITE4)
					begin 
						if(key == 8'd5)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK5)
					data <= BLACK;
				else if (K_WHITE5)
					begin 
						if(key == 8'd6)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK6)
					data <= BLACK;
				else if (K_WHITE6)
					begin 
						if(key == 8'd7)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK7)
					data <= BLACK;
				else if (K_WHITE7)
					begin 
						if(key == 8'd8)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK8)
					data <= BLACK;
				else if (K_WHITE8)
					begin 
						if(key == 8'd9)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK9)
					data <= BLACK;
				else if (K_WHITE9)
					begin 
						if(key == 8'd10)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK10)
					data <= BLACK;
				else if (K_WHITE10)
					begin 
						if(key == 8'd11)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK11)
					data <= BLACK;
				else if (K_WHITE11)
					begin 
						if(key == 8'd12)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK12)
					data <= BLACK;
				else if (K_WHITE12)
					begin 
						if(key == 8'd13)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK13)
					data <= BLACK;
				else if (K_WHITE13)
					begin 
						if(key == 8'd14)
							data <= GRAY;//·ÛºìÉ«
						else if(key == 'd0)
							data <= GRAY;//°×
						else
							data <= GRAY;
					end
				else if (K_BLACK14)
					data <= BLACK;
				else if (K_WHITE14)
					begin 
						if(key == 8'd15)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK15)
					data <= BLACK;
				else if (K_WHITE15)
					begin 
						if(key == 8'd16)
							data <= WHITE;//·ÛºìÉ«
						else if(key == 'd0)
							data <= WHITE;//°×
						else
							data <= WHITE;
					end
				else if (K_BLACK16)
					data <= BLACK;
				else if (K_WHITE16)
					data <= GREEN;
				else
					data <= BLACK;
			end else
				data <= BLACK;
		else
			data <= BLACK;
	end
	
endmodule
