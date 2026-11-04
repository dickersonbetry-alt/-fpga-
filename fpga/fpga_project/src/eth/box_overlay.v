// ============================================================
// box_overlay.v : 在 TFT 像素流上叠加 YOLO 目标框
//
// 坐标系与图像一致（800×480），框线宽 2 像素，绿色/黄色
// ============================================================
module box_overlay(
    input  wire        clk,
    input  wire        de,
    input  wire [11:0] h,
    input  wire [11:0] v,
    input  wire [4:0]  cnt,
    input  wire [255:0] x1,
    input  wire [255:0] y1,
    input  wire [255:0] x2,
    input  wire [255:0] y2,
    input  wire [127:0] score,
    input  wire [127:0] cls,
    output reg  [15:0] rgb = 16'd0,
    output reg         en  = 1'b0
);

    integer i;
    reg        hit;
    reg [15:0] bx1, by1, bx2, by2;
    reg [7:0]  sc;

    always @(*) begin
        hit = 1'b0;
        sc  = 8'd0;
        for (i = 0; i < 16; i = i + 1) begin
            if (i < cnt) begin
                bx1 = x1[i*16 +: 16];
                by1 = y1[i*16 +: 16];
                bx2 = x2[i*16 +: 16];
                by2 = y2[i*16 +: 16];
                if (h >= bx1 && h <= bx2 && v >= by1 && v <= by2) begin
                    // 2 像素边框
                    if (h <= bx1 + 1 || h + 1 >= bx2 ||
                        v <= by1 + 1 || v + 1 >= by2) begin
                        hit = 1'b1;
                        sc  = score[i*8 +: 8];
                    end
                end
            end
        end
    end

    always @(posedge clk) begin
        en  <= de && hit;
        rgb <= (sc >= 8'd50) ? 16'h07E0 : 16'hFFE0;   // 绿 / 黄
    end

endmodule
