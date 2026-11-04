// ============================================================
// osd_text.v : 在 TFT 画面上叠加 4 行文字
//
// 显示内容（左上角，2 倍放大 8x8 点阵，每行 7 个字符）：
//   R +123
//   P -045
//   Y +178
//   V 05.67      <- 激光测距 0-10V 模拟量电压
//
// 输入 roll/pitch/yaw 为 度×10 的有符号数，这里取整数度显示。
// volt_x100 为电压×100（0.01V 单位）。
// ============================================================
module osd_text #(
    parameter [11:0] X0 = 12'd16,   // 文字左上角
    parameter [11:0] Y0 = 12'd16,
    parameter RAW_MODE = 0          // 1 = 直接显示原始值(0-4095)，0 = 显示 值/10
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               de,        // TFT 数据有效
    input  wire [11:0]        h,         // 像素 x
    input  wire [11:0]        v,         // 像素 y
    input  wire signed [15:0] roll_x10,
    input  wire signed [15:0] pitch_x10,
    input  wire signed [15:0] yaw_x10,
    input  wire [15:0]        volt_x100, // 电压×100（0.01V）
    input  wire [15:0]        enc_cnt,   // 编码器计数（4 位十进制）
    input  wire               imu_zeroing, // 1 = IMU 对零中，显示 ZERO...
    output reg  [15:0]        rgb = 16'd0,  // RGB565
    output reg                en  = 1'b0    // 1 = 覆盖摄像头像素
);

    // ---------------- 度×10 -> 整数度 + 符号 + 3 位 BCD ----------------
    wire signed [15:0] r = roll_x10;
    wire signed [15:0] p = pitch_x10;
    wire signed [15:0] y = yaw_x10;

    wire [15:0] ra = r[15] ? -r : r;
    wire [15:0] pa = p[15] ? -p : p;
    wire [15:0] ya = y[15] ? -y : y;

    // 整数度
    wire [15:0] rd = RAW_MODE ? ra : (ra / 16'd10);
    wire [15:0] pd = RAW_MODE ? pa : (pa / 16'd10);
    wire [15:0] yd = RAW_MODE ? ya : (ya / 16'd10);

    // BCD：百/十/个
    wire [3:0] r_th = (rd / 16'd1000) % 16'd10;
    wire [3:0] r_h = (rd / 16'd100) % 16'd10;
    wire [3:0] r_t = (rd / 16'd10)  % 16'd10;
    wire [3:0] r_u = rd % 16'd10;
    wire [3:0] p_th = (pd / 16'd1000) % 16'd10;
    wire [3:0] p_h = (pd / 16'd100) % 16'd10;
    wire [3:0] p_t = (pd / 16'd10)  % 16'd10;
    wire [3:0] p_u = pd % 16'd10;
    wire [3:0] y_th = (yd / 16'd1000) % 16'd10;
    wire [3:0] y_h = (yd / 16'd100) % 16'd10;
    wire [3:0] y_t = (yd / 16'd10)  % 16'd10;
    wire [3:0] y_u = yd % 16'd10;

    // 编码器计数 4 位 BCD
    wire [15:0] ev   = enc_cnt;
    wire [3:0]  e_th = (ev / 16'd1000) % 16'd10;
    wire [3:0]  e_h  = (ev / 16'd100)  % 16'd10;
    wire [3:0]  e_t  = (ev / 16'd10)   % 16'd10;
    wire [3:0]  e_u  = ev % 16'd10;

    // ---------------- 像素坐标 -> 字符/点阵 ----------------
    wire [11:0] x = h - X0;
    wire [11:0] yy = v - Y0;

    wire in_area = de && (h >= X0) && (h < X0 + 12'd112) && (v >= Y0) && (v < Y0 + 12'd80);

    wire [2:0] line = yy[6:4];     // 0..4
    wire [2:0] col  = x[6:4];      // 0..6
    wire [2:0] frow = yy[3:1];     // 8 行点阵
    wire [2:0] fcol = x[3:1];      // 8 列点阵

    // 电压：0.01V 单位 -> 整数伏/小数位
    wire [15:0] vv     = volt_x100;
    wire [15:0] v_int  = vv / 16'd100;               // 0..10 V
    wire [15:0] v_frac = vv % 16'd100;               // 0..99
    wire [3:0]  v_t    = (v_int / 16'd10) % 16'd10;  // 十位
    wire [3:0]  v_u    = v_int % 16'd10;             // 个位
    wire [3:0]  f_t    = (v_frac / 16'd10) % 16'd10; // 十分位
    wire [3:0]  f_u    = v_frac % 16'd10;            // 百分位

    // 每行的数值
    reg [3:0] d_th = 4'd0, d_h = 4'd0, d_t = 4'd0, d_u = 4'd0;
    reg       sign_bit = 1'b0;
    always @(*) begin
        case (line)
        3'd0: begin d_th = r_th; d_h = r_h; d_t = r_t; d_u = r_u; sign_bit = r[15]; end
        3'd1: begin d_th = p_th; d_h = p_h; d_t = p_t; d_u = p_u; sign_bit = p[15]; end
        3'd2: begin d_th = y_th; d_h = y_h; d_t = y_t; d_u = y_u; sign_bit = y[15]; end
        default: begin d_h = 4'd0; d_t = 4'd0; d_u = 4'd0; sign_bit = 1'b0; end
        endcase
    end

    // 当前字符
    reg [7:0] ch = 8'h20;
    always @(*) begin
        if (line == 3'd4) begin
            // E 1234
            case (col)
            3'd0: ch = 8'h45;              // E
            3'd1: ch = 8'h20;
            3'd2: ch = 8'h30 + e_th;
            3'd3: ch = 8'h30 + e_h;
            3'd4: ch = 8'h30 + e_t;
            3'd5: ch = 8'h30 + e_u;
            default: ch = 8'h20;
            endcase
        end else if (line == 3'd3) begin
            // V 05.67
            case (col)
            3'd0: ch = 8'h56;              // V
            3'd1: ch = 8'h20;              // 空格
            3'd2: ch = 8'h30 + v_t;
            3'd3: ch = 8'h30 + v_u;
            3'd4: ch = 8'h2E;              // '.'
            3'd5: ch = 8'h30 + f_t;
            default: ch = 8'h30 + f_u;
            endcase
        end else if (imu_zeroing) begin
            // 对零中：第一行显示 ZERO...，其它行空白，不显示角度数据
            if (line == 2'd0) begin
                case (col)
                3'd0: ch = 8'h5A;          // Z
                3'd1: ch = 8'h45;          // E
                3'd2: ch = 8'h52;          // R
                3'd3: ch = 8'h4F;          // O
                default: ch = 8'h2E;       // .
                endcase
            end else begin
                ch = 8'h20;
            end
        end else begin
            case (col)
            3'd0: ch = (line == 2'd0) ? 8'h52 : (line == 2'd1) ? 8'h50 : 8'h59; // R/P/Y
            3'd1: ch = RAW_MODE ? (8'h30 + d_th) : (sign_bit ? 8'h2D : 8'h2B);
            3'd2: ch = 8'h30 + d_h;
            3'd3: ch = 8'h30 + d_t;
            3'd4: ch = 8'h30 + d_u;
            default: ch = 8'h20;                   // 空格
            endcase
        end
    end

    // ---------------- 8x8 点阵 ----------------
    function [7:0] glyph;
        input [7:0] c;
        input [2:0] row;
        begin
            case (c)
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
            8'h52: case (row) 3'd0:glyph=8'h7C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h7C;
                             3'd4:glyph=8'h6C; 3'd5:glyph=8'h66; 3'd6:glyph=8'h66; default:glyph=8'h00; endcase
            8'h50: case (row) 3'd0:glyph=8'h7C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h7C;
                             3'd4:glyph=8'h60; 3'd5:glyph=8'h60; 3'd6:glyph=8'h60; default:glyph=8'h00; endcase
            8'h59: case (row) 3'd0:glyph=8'h66; 3'd1:glyph=8'h66; 3'd2:glyph=8'h3C; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h18; 3'd5:glyph=8'h18; 3'd6:glyph=8'h18; default:glyph=8'h00; endcase
            8'h56: case (row) 3'd0:glyph=8'h66; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h66;
                             3'd4:glyph=8'h66; 3'd5:glyph=8'h3C; 3'd6:glyph=8'h18; default:glyph=8'h00; endcase
            8'h2E: case (row) 3'd0:glyph=8'h00; 3'd1:glyph=8'h00; 3'd2:glyph=8'h00; 3'd3:glyph=8'h00;
                             3'd4:glyph=8'h00; 3'd5:glyph=8'h18; 3'd6:glyph=8'h18; default:glyph=8'h00; endcase
            8'h5A: case (row) 3'd0:glyph=8'h7E; 3'd1:glyph=8'h06; 3'd2:glyph=8'h0C; 3'd3:glyph=8'h18;
                             3'd4:glyph=8'h30; 3'd5:glyph=8'h60; 3'd6:glyph=8'h7E; default:glyph=8'h00; endcase
            8'h45: case (row) 3'd0:glyph=8'h7E; 3'd1:glyph=8'h60; 3'd2:glyph=8'h60; 3'd3:glyph=8'h7C;
                             3'd4:glyph=8'h60; 3'd5:glyph=8'h60; 3'd6:glyph=8'h7E; default:glyph=8'h00; endcase
            8'h4F: case (row) 3'd0:glyph=8'h3C; 3'd1:glyph=8'h66; 3'd2:glyph=8'h66; 3'd3:glyph=8'h66;
                             3'd4:glyph=8'h66; 3'd5:glyph=8'h66; 3'd6:glyph=8'h3C; default:glyph=8'h00; endcase
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
                rgb <= pix ? 16'hFFFF : 16'h0000;   // 白字黑底
            else
                rgb <= 16'd0;
        end
    end

endmodule
