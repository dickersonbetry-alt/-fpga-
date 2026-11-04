module pwm
#(
    parameter [31:0] T = 50_000
)(
    input clk,
    input rst_n,
    input [31:0] duty_count,
    output wire pwm
);
    reg [31:0] cnt;
    always @(posedge clk or negedge rst_n)
        begin
            if(rst_n == 0)
                cnt <= 0;
            else if(cnt >= T-1)
                cnt <= 0;
            else
                cnt <= cnt+1;
        end
    assign pwm = rst_n && (cnt<duty_count);
endmodule