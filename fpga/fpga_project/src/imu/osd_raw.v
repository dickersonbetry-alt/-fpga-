// ============================================================
// osd_raw.v : 在 TFT 上叠加 6 行 IMU 原始数据（调试用）
//
//   AX +0123
//   AY -0456
//   AZ +0789
//   GX +0012
//   GY -0034
//   GZ +0005
//
// 数值为 imu_parser 解析出的 int16 原始值（未缩放），
// 超出 ±9999 时显示饱和值 9999。8x8 点阵 2 倍放大。
// ============================================================
module osd_raw #(
    parameter [11:0] X0 = 12'd16,
    parameter [11:0] Y0 = 12'd16
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               de,
    input  wire [11:0]        h,
    input  wire [11:0]        v,
    input  wire signed [15:0] ax,
    input  wire signed [15:0] ay,
    input  wire signed [15:0] az,
    input  wire signed [15:0] gx,
    input  wire signed [15:0] gy,
    input  wire signed [15:0] gz,
    output reg  [15:0]        rgb = 16'd0,
    output reg                en  = 1'b0
);

    wire [11:0] x  = h - X0;
    wire [11:0] yy = v - Y0;

    wire in_area = de && (h >= X0) && (h < X0 + 12'd112) && (v >= Y0) && (v < Y0 + 12'd96);

    wire [2:0] line = yy[6:4];     // 0..5
    wire [2:0] col  = x[6:4];      // 0..6
    wire [2:0] frow = yy[3:1];
    wire [2:0] fcol = x[3:1];

    // ---------------- 每行数值 ----------------
    reg signed [15:0] val;
    always @(*) begin
        case (line)
        3'd0: val = ax;
        3'd1: val = ay;
        3'd2: val = az;
        3'd3: val = gx;
        3'd4: val = gy;
        default: val = gz;
        endcase
    end

    wire [15:0] av = val[15] ? -val : val;
    wire [15:0] av4 = (av > 16'd9999) ? 16'd9999 : av;

    wire [3:0] d3 = (av4 / 16'd1000) % 16'd10;
    wire [3:0] d2 = (av4 / 16'd100)  % 16'd10;
    wire [3:0] d1 = (av4 / 16'd10)   % 16'd10;
    wire [3:0] d0 = av4 % 16'd10;

    // ---------------- 行标签 ----------------
    reg [7:0] l0, l1;
    always @(*) begin
        case (line)
        3'd0: begin l0 = 8'h41; l1 = 8'h58; end // AX
        3'd1: begin l0 = 8'h41; l1 = 8'h59; end // AY
        3'd2: begin l0 = 8'h41; l1 = 8'h5A; end // AZ
        3'd3: begin l0 = 8'h47; l1 = 8'h58; end // GX
        3'd4: begin l0 = 8'h47; l1 = 8'h59; end // GY
        default: begin l0 = 8'h47; l1 = 8'h5A; end // GZ
        endcase
    end

    // ---------------- 当前字符 ----------------
    reg [7:0] ch;
    always @(*) begin
        case (col)
        3'd0: ch = l0;
        3'd1: ch = l1;
        3'd2: ch = val[15] ? 8'h2D : 8'h2B;   // '-' / '+'
        3'd3: ch = 8'h30 + d3;
        3'd4: ch = 8'h30 + d2;
        3'd5: ch = 8'h30 + d1;
        default: ch = 8'h30 + d0;
        endcase
    end

    // ---------------- 8x8 点阵 ----------------
    function [7:0] glyph;
        input [7:0] c;
        input [2:0] row;
        begin
            case (c)
            8'h41: case (row) 3'd0:glyph=8'h18; 3'd1:glyph=8'h24; 3'd2:glyph=8'h42; 3'd3:glyph=8'h7E;
                             3'd4:glyph=8'h42; 3'd5:glyph=8'h42; 3'd6:glyph=8'h42; default:glyph=8'h00; endcase
            8'h58: case (row) 3'd0:glyph=8'h42; 3'd1:glyph=8'h24; 3'd2:glyph=8'h18; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h24; 3'd6:glyph=8'h42; default:glyph=8'h00; endcase
            8'h47: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h42; 3'd2:glyph=8'h40; 3'd3:glyph=8'h4E;
                             3'd4:glyph=8'h42; 3'd5:glyph=8'h42; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h59: case (row) 3'd0:glyph=8'h66; 3'd1:glyph=8'h66; 3'd2:glyph=8'h3C; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h18; 3'd6:glyph=8'h18; default:glyph=8'h00; endcase
            8'h5A: case (row) 3'd0:glyph=8'h7E; 3'd1:glyph=8'h06; 3'd2:glyph=8'h0C; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h30; 3'd5:glyph=8'h60; 3'd6:glyph=8'h7E; default:glyph=8'h00; endcase
            8'h2B: case (row) 3'd0:glyph=8'h00; 3'd1:glyph=8'h18; 3'd2:glyph=8'h18; 3'd3:glyph=8'h7E;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h18; 3'd6:glyph=8'h00; default:glyph=8'h00; endcase
            8'h2D: case (row) 3'd0:glyph=8'h00; 3'd1:glyph=8'h00; 3'd2:glyph=8'h00; 3'd3:glyph=8'h7E;
                             3'd4:glyph=8'h00; 3'd5:glyph=8'h00; 3'd6:glyph=8'h00; default:glyph=8'h00; endcase
            8'h30: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h6E; 3'd3:glyph=8'h76;
                             3'd4:glyph=8'h66; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h31: case (row) 3'd0:glyph=8'h18; 3'd1:glyph=8'h38; 3'd2:glyph=8'h18; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h18; 3'd6:glyph=8'h7E; default:glyph=8'h00; endcase
            8'h32: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h06; 3'd3:glyph=8'h0C;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h30; 3'd6:glyph=8'h7E; default:glyph=8'h00; endcase
            8'h33: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h06; 3'd3:glyph=8'h1C;
                             3'd4:glyph=8'h06; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h34: case (row) 3'd0:glyph=8'h0C; 3'd1:glyph=8'h1C; 3'd2:glyph=8'h3C; 3'd3:glyph=8'h6C;
                             3'd4:glyph=8'h7E; 3'd5:glyph=8'h0C; 3'd6:glyph=8'h0C; default:glyph=8'h00; endcase
            8'h35: case (row) 3'd0:glyph=8'h7E; 3'd1:glyph=8'h60; 3'd2:glyph=8'h7C; 3'd3:glyph=8'h06;
                             3'd4:glyph=8'h06; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h36: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h60; 3'd3:glyph=8'h7C;
                             3'd4:glyph=8'h66; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h37: case (row) 3'd0:glyph=8'h7E; 3'd1:glyph=8'h06; 3'd2:glyph=8'h0C; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h30; 3'd5:glyph=8'h30; 3'd6:glyph=8'h30; default:glyph=8'h00; endcase
            8'h38: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h3C;
                             3'd4:glyph=8'h66; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            8'h39: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h3E;
                             3'd4:glyph=8'h06; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
            default: glyph = 8'h00;
            endcase
        end
    endfunction

    wire [7:0] bits = glyph(ch, frow);
    wire pix = bits[3'd7 - fcol];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgb <= 16'd0;
            en  <= 1'b0;
        end else begin
            en <= in_area;
            if (in_area)
                rgb <= pix ? 16'hFFFF : 16'h0000;
            else
                rgb <= 16'd0;
        end
    end

endmodule
