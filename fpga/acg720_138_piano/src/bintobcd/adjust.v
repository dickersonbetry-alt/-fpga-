module adjust(idata, odata);

	input [3:0] idata;
        output [3:0] odata;

	assign odata = (idata > 3'd4) ? idata + 4'd3 : idata;

endmodule