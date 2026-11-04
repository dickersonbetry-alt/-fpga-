// ============================================================
// btn_grid.v : TFT 右侧按钮叠加（中文 16x16 点阵）
//   AV(避障) / TRK(追踪) 两个模式按钮 + 3x3 九宫格
//   左前 前 右前 / 左 停止 右 / 左后 后 右后
//   按下/锁定按钮底色变红
// ============================================================
module btn_grid #(
    parameter [11:0] X0 = 12'd608,
    parameter [11:0] Y0 = 12'd60,
    parameter [11:0] CW = 12'd60,
    parameter [11:0] CH = 12'd80,
    parameter [11:0] SX = 12'd64,
    parameter [11:0] SY = 12'd92,
    parameter [11:0] MY0 = 12'd8,
    parameter [11:0] MH  = 12'd40
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de,
    input  wire [11:0] h,
    input  wire [11:0] v,
    input  wire        pressed,
    input  wire [3:0]  cell_sel,
    input  wire [1:0]  mode_sel,
    output reg  [15:0] rgb,
    output reg         en
);

    // 0=前 1=左 2=右 3=后 4=停 5=止 6=避 7=障 8=绕
    function [15:0] hanzi;
        input [3:0] idx;
        input [3:0] row;
        begin
            case (idx)
            4'd0: case (row)
                4'd0:hanzi=16'h0000; 4'd1:hanzi=16'h0430; 4'd2:hanzi=16'h0630; 4'd3:hanzi=16'h7FFE;
                4'd4:hanzi=16'h7FFE; 4'd5:hanzi=16'h0004; 4'd6:hanzi=16'h3F2C; 4'd7:hanzi=16'h2164;
                4'd8:hanzi=16'h3F64; 4'd9:hanzi=16'h3364; 4'd10:hanzi=16'h3364; 4'd11:hanzi=16'h3F64;
                4'd12:hanzi=16'h2124; 4'd13:hanzi=16'h210C; 4'd14:hanzi=16'h271C; default:hanzi=16'h2200; endcase
            4'd1: case (row)
                4'd0:hanzi=16'h0300; 4'd1:hanzi=16'h0300; 4'd2:hanzi=16'h330C; 4'd3:hanzi=16'h3FFE;
                4'd4:hanzi=16'h0600; 4'd5:hanzi=16'h0600; 4'd6:hanzi=16'h0400; 4'd7:hanzi=16'h0FFC;
                4'd8:hanzi=16'h0CC0; 4'd9:hanzi=16'h18C0; 4'd10:hanzi=16'h30C0; 4'd11:hanzi=16'h60C0;
                4'd12:hanzi=16'h40C0; 4'd13:hanzi=16'h1FFE; 4'd14:hanzi=16'h0000; default:hanzi=16'h0000; endcase
            4'd2: case (row)
                4'd0:hanzi=16'h0100; 4'd1:hanzi=16'h0100; 4'd2:hanzi=16'h0300; 4'd3:hanzi=16'h7FFE;
                4'd4:hanzi=16'h0200; 4'd5:hanzi=16'h0600; 4'd6:hanzi=16'h0400; 4'd7:hanzi=16'h0FF8;
                4'd8:hanzi=16'h1C08; 4'd9:hanzi=16'h3C08; 4'd10:hanzi=16'h7C08; 4'd11:hanzi=16'h6C08;
                4'd12:hanzi=16'h0FF8; 4'd13:hanzi=16'h0FF8; 4'd14:hanzi=16'h0C08; default:hanzi=16'h0000; endcase
            4'd3: case (row)
                4'd0:hanzi=16'h0008; 4'd1:hanzi=16'h1FFC; 4'd2:hanzi=16'h1E00; 4'd3:hanzi=16'h1000;
                4'd4:hanzi=16'h1FFE; 4'd5:hanzi=16'h1FFE; 4'd6:hanzi=16'h3000; 4'd7:hanzi=16'h3000;
                4'd8:hanzi=16'h37FC; 4'd9:hanzi=16'h340C; 4'd10:hanzi=16'h240C; 4'd11:hanzi=16'h640C;
                4'd12:hanzi=16'h67FC; 4'd13:hanzi=16'h440C; 4'd14:hanzi=16'h0400; default:hanzi=16'h0000; endcase
            4'd4: case (row)
                4'd0:hanzi=16'h0000; 4'd1:hanzi=16'h1860; 4'd2:hanzi=16'h17FE; 4'd3:hanzi=16'h3000;
                4'd4:hanzi=16'h33F8; 4'd5:hanzi=16'h730C; 4'd6:hanzi=16'h73FC; 4'd7:hanzi=16'h5000;
                4'd8:hanzi=16'h17FE; 4'd9:hanzi=16'h1402; 4'd10:hanzi=16'h13FC; 4'd11:hanzi=16'h1060;
                4'd12:hanzi=16'h1020; 4'd13:hanzi=16'h1060; 4'd14:hanzi=16'h10E0; default:hanzi=16'h0000; endcase
            4'd5: case (row)
                4'd0:hanzi=16'h0000; 4'd1:hanzi=16'h0080; 4'd2:hanzi=16'h0080; 4'd3:hanzi=16'h0080;
                4'd4:hanzi=16'h0080; 4'd5:hanzi=16'h1880; 4'd6:hanzi=16'h18FC; 4'd7:hanzi=16'h1880;
                4'd8:hanzi=16'h1880; 4'd9:hanzi=16'h1880; 4'd10:hanzi=16'h1880; 4'd11:hanzi=16'h1880;
                4'd12:hanzi=16'h1880; 4'd13:hanzi=16'h7FFE; 4'd14:hanzi=16'h7FFE; default:hanzi=16'h0000; endcase
            4'd6: case (row)
                4'd0:hanzi=16'h0000; 4'd1:hanzi=16'h0018; 4'd2:hanzi=16'h67BC; 4'd3:hanzi=16'h24FE;
                4'd4:hanzi=16'h24A4; 4'd5:hanzi=16'h07B4; 4'd6:hanzi=16'h6C3C; 4'd7:hanzi=16'h7C7E;
                4'd8:hanzi=16'h3F98; 4'd9:hanzi=16'h3E98; 4'd10:hanzi=16'h3EFE; 4'd11:hanzi=16'h3E98;
                4'd12:hanzi=16'h3798; 4'd13:hanzi=16'h7C08; 4'd14:hanzi=16'h47FE; default:hanzi=16'h0000; endcase
            4'd7: case (row)
                4'd0:hanzi=16'h0000; 4'd1:hanzi=16'h0060; 4'd2:hanzi=16'h7DFC; 4'd3:hanzi=16'h49FE;
                4'd4:hanzi=16'h4898; 4'd5:hanzi=16'h5BFF; 4'd6:hanzi=16'h5000; 4'd7:hanzi=16'h59FC;
                4'd8:hanzi=16'h4D04; 4'd9:hanzi=16'h4DFC; 4'd10:hanzi=16'h4DFC; 4'd11:hanzi=16'h59FC;
                4'd12:hanzi=16'h4366; 4'd13:hanzi=16'h43FE; 4'd14:hanzi=16'h4020; default:hanzi=16'h0020; endcase
            // 追（宋体 16x16）
            4'd8: case (row)
                4'd0:hanzi=16'h0020; 4'd1:hanzi=16'h2040; 4'd2:hanzi=16'h13FC; 4'd3:hanzi=16'h1204;
                4'd4:hanzi=16'h0204; 4'd5:hanzi=16'h03FC; 4'd6:hanzi=16'hF200; 4'd7:hanzi=16'h1200;
                4'd8:hanzi=16'h13FE; 4'd9:hanzi=16'h1202; 4'd10:hanzi=16'h1202; 4'd11:hanzi=16'h13FE;
                4'd12:hanzi=16'h1202; 4'd13:hanzi=16'h2800; 4'd14:hanzi=16'h47FE; default:hanzi=16'h0000; endcase
            // 踪（宋体 16x16）
            4'd9: case (row)
                4'd0:hanzi=16'h0040; 4'd1:hanzi=16'h7820; 4'd2:hanzi=16'h4BFE; 4'd3:hanzi=16'h4A02;
                4'd4:hanzi=16'h4C04; 4'd5:hanzi=16'h79F8; 4'd6:hanzi=16'h1000; 4'd7:hanzi=16'h1000;
                4'd8:hanzi=16'h53FE; 4'd9:hanzi=16'h5C20; 4'd10:hanzi=16'h5128; 4'd11:hanzi=16'h5124;
                4'd12:hanzi=16'h5A22; 4'd13:hanzi=16'hE422; 4'd14:hanzi=16'h00A0; default:hanzi=16'h0040; endcase
            default: case (row)
                4'd0:hanzi=16'h1080; 4'd1:hanzi=16'h10DC; 4'd2:hanzi=16'h33F0; 4'd3:hanzi=16'h234C;
                4'd4:hanzi=16'h4C78; 4'd5:hanzi=16'h78F0; 4'd6:hanzi=16'h1B9E; 4'd7:hanzi=16'h100C;
                4'd8:hanzi=16'h23FE; 4'd9:hanzi=16'h78F0; 4'd10:hanzi=16'h0090; 4'd11:hanzi=16'h0D90;
                4'd12:hanzi=16'h7B12; 4'd13:hanzi=16'h061E; 4'd14:hanzi=16'h0000; default:hanzi=16'h0000; endcase
            endcase
        end
    endfunction

    // ---------------- 模式按钮 AV / BY ----------------
    localparam [11:0] AVX0 = 12'd608, AVX1 = 12'd700;
    localparam [11:0] BYX0 = 12'd704, BYX1 = 12'd796;

    wire in_av = de && (h >= AVX0) && (h <= AVX1) && (v >= MY0) && (v <= MY0 + MH);
    wire in_by = de && (h >= BYX0) && (h <= BYX1) && (v >= MY0) && (v <= MY0 + MH);

    wire av_border = in_av && ((h < AVX0+2) || (h > AVX1-2) || (v < MY0+2) || (v > MY0+MH-2));
    wire by_border = in_by && ((h < BYX0+2) || (h > BYX1-2) || (v < MY0+2) || (v > MY0+MH-2));

    // 避障 / 绕障：2 个 16x16，水平居中（92-32)/2=30，纵向 (40-16)/2=12
    wire [11:0] av_tx = h - (AVX0 + 12'd30);
    wire [11:0] av_ty = v - (MY0 + 12'd12);
    wire in_av_text = in_av && (av_tx < 12'd32) && (av_ty < 12'd16);
    wire [15:0] av_row = av_tx[4] ? hanzi(4'd7, av_ty[3:0]) : hanzi(4'd6, av_ty[3:0]);
    wire av_pix = av_row[15 - av_tx[3:0]];

    wire [11:0] by_tx = h - (BYX0 + 12'd30);
    wire [11:0] by_ty = v - (MY0 + 12'd12);
    wire in_by_text = in_by && (by_tx < 12'd32) && (by_ty < 12'd16);
    wire [15:0] by_row = by_tx[4] ? hanzi(4'd9, by_ty[3:0]) : hanzi(4'd8, by_ty[3:0]);   // 追踪
    wire by_pix = by_row[15 - by_tx[3:0]];

    // ---------------- 九宫格 ----------------
    wire in_x = (h >= X0) && (h < X0 + SX*3);
    wire in_y = (v >= Y0) && (v < Y0 + SY*3);
    wire in_grid = de && in_x && in_y;

    wire [11:0] dx = h - X0;
    wire [11:0] dy = v - Y0;

    reg [1:0]  col;
    reg [1:0]  row;
    reg [11:0] cx;
    reg [11:0] cy;
    always @(*) begin
        if (dx < SX)         begin col = 2'd0; cx = 12'd0;   end
        else if (dx < SX*2)  begin col = 2'd1; cx = SX;      end
        else                 begin col = 2'd2; cx = SX*2;    end

        if (dy < SY)         begin row = 2'd0; cy = 12'd0;   end
        else if (dy < SY*2)  begin row = 2'd1; cy = SY;      end
        else                 begin row = 2'd2; cy = SY*2;    end
    end

    wire [11:0] ox = dx - cx;
    wire [11:0] oy = dy - cy;

    wire in_cell = in_grid && (ox < CW) && (oy < CH);
    wire border  = in_cell && ((ox < 2) || (ox >= CW-2) || (oy < 2) || (oy >= CH-2));

    wire [3:0] cell_idx = {row, 1'b0} + row + col;
    wire sel = pressed && (cell_idx == cell_sel);

    reg [3:0] ch_a, ch_b;
    reg       nc2;
    always @(*) begin
        case ({row, col})
        {2'd0,2'd0}: begin ch_a = 4'd1; ch_b = 4'd0; nc2 = 1'b1; end // 左前
        {2'd0,2'd1}: begin ch_a = 4'd0; ch_b = 4'd0; nc2 = 1'b0; end // 前
        {2'd0,2'd2}: begin ch_a = 4'd2; ch_b = 4'd0; nc2 = 1'b1; end // 右前
        {2'd1,2'd0}: begin ch_a = 4'd1; ch_b = 4'd0; nc2 = 1'b0; end // 左
        {2'd1,2'd1}: begin ch_a = 4'd4; ch_b = 4'd5; nc2 = 1'b1; end // 停止
        {2'd1,2'd2}: begin ch_a = 4'd2; ch_b = 4'd0; nc2 = 1'b0; end // 右
        {2'd2,2'd0}: begin ch_a = 4'd1; ch_b = 4'd3; nc2 = 1'b1; end // 左后
        {2'd2,2'd1}: begin ch_a = 4'd3; ch_b = 4'd0; nc2 = 1'b0; end // 后
        default:     begin ch_a = 4'd2; ch_b = 4'd3; nc2 = 1'b1; end // 右后
        endcase
    end

    wire [11:0] t2x = ox - 12'd14;
    wire [11:0] t1x = ox - 12'd22;
    wire [11:0] txc = nc2 ? t2x : t1x;
    wire [3:0]  chsel = nc2 ? (t2x[4] ? ch_b : ch_a) : ch_a;
    wire [11:0] ty = oy - 12'd32;
    wire in_text = in_cell && (ty < 12'd16) && (txc < (nc2 ? 12'd32 : 12'd16));
    wire [15:0] cell_row = hanzi(chsel, ty[3:0]);
    wire cell_pix = cell_row[15 - txc[3:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            en  <= 1'b0;
            rgb <= 16'd0;
        end else begin
            en <= in_av || in_by || in_cell;
            if (in_av) begin
                if (av_border)                   rgb <= 16'hFFFF;
                else if (in_av_text && av_pix)   rgb <= 16'hFFFF;
                else                             rgb <= (mode_sel == 2'd1) ? 16'hF800 : 16'h0010;
            end else if (in_by) begin
                if (by_border)                   rgb <= 16'hFFFF;
                else if (in_by_text && by_pix)   rgb <= 16'hFFFF;
                else                             rgb <= (mode_sel == 2'd2) ? 16'hF800 : 16'h0010;
            end else if (in_cell) begin
                if (border)                      rgb <= 16'hFFFF;
                else if (in_text && cell_pix)    rgb <= 16'hFFFF;
                else                             rgb <= sel ? 16'hF800 : 16'h0010;
            end else begin
                rgb <= 16'd0;
            end
        end
    end

endmodule
