// ============================================================
// cursor.v : 触摸光标（按下显示十字准星，抬起消失）
// ============================================================
module cursor(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de,
    input  wire [11:0] h,
    input  wire [11:0] v,
    input  wire        pressed,
    input  wire [11:0] cx,
    input  wire [11:0] cy,
    output reg  [15:0] rgb,
    output reg         en
);

    wire [11:0] dx = (h > cx) ? (h - cx) : (cx - h);
    wire [11:0] dy = (v > cy) ? (v - cy) : (cy - v);

    wire cross = de && pressed && (((dx <= 12'd1) && (dy <= 12'd10)) ||
                                   ((dy <= 12'd1) && (dx <= 12'd10)));
    wire dot   = de && pressed && (dx <= 12'd2) && (dy <= 12'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en  <= 1'b0;
            rgb <= 16'd0;
        end else begin
            en <= cross || dot;
            rgb <= dot ? 16'hFFFF : (cross ? 16'hFFE0 : 16'h0000);
        end
    end

endmodule
