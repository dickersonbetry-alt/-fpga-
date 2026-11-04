// eth_stat.v : GMII 层旁路解析（纯诊断，不参与收包判决）
//
// 以 SFD(0xD5) 为基准计数，直接解出关键字段，避免"dv 与数据差一个字节"的偏移问题：
//   SFD 之后 pos 对照：0..5 目的MAC | 6..11 源MAC | 12..13 EtherType
//                      14..33 IP头 | 34..41 UDP头 | 42.. 负载
//   因此  pos 12,13  -> EtherType
//         pos 30..33 -> 目的 IP
//         pos 34,35  -> UDP 源端口
//         pos 36,37  -> UDP 目的端口
module eth_stat(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        dv,
    input  wire [7:0]  d,
    output reg  [15:0] frames     = 16'd0,
    output reg  [15:0] last_len   = 16'd0,
    output reg  [15:0] last_etype = 16'd0,
    output reg  [15:0] last_dip_hi = 16'd0,
    output reg  [15:0] last_dip_lo = 16'd0,
    output reg  [15:0] last_sport = 16'd0,
    output reg  [15:0] last_dport = 16'd0
);
    reg        pdv = 1'b0;
    reg        sfd = 1'b0;
    reg [9:0]  pos = 10'd0;
    reg [15:0] len = 16'd0;
    reg [7:0]  ehi  = 8'd0;
    reg [7:0]  dph  = 8'd0;
    reg [7:0]  dpl  = 8'd0;
    reg [7:0]  sph  = 8'd0;
    reg [7:0]  dth  = 8'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pdv <= 1'b0; sfd <= 1'b0; pos <= 10'd0; len <= 16'd0;
            ehi <= 8'd0; dph <= 8'd0; dpl <= 8'd0; sph <= 8'd0; dth <= 8'd0;
            frames <= 16'd0; last_len <= 16'd0; last_etype <= 16'd0;
            last_dip_hi <= 16'd0; last_dip_lo <= 16'd0;
            last_sport <= 16'd0; last_dport <= 16'd0;
        end else begin
            pdv <= dv;
            if (dv && !pdv) begin
                sfd      <= 1'b0;        // 新帧：重新找 SFD
                pos      <= 10'd0;
                len      <= 16'd1;
                frames   <= frames + 16'd1;
                last_len <= len;
            end else if (dv) begin
                len <= len + 16'd1;
                if (!sfd) begin
                    if (d == 8'hD5) begin
                        sfd <= 1'b1;     // 找到 SFD，下一个字节才是目的 MAC
                        pos <= 10'd0;
                        last_etype <= 16'd0;
                    end
                end else begin
                    case (pos)
                    10'd12: ehi <= d;
                    10'd13: last_etype  <= {ehi, d};
                    10'd30: dph <= d;
                    10'd31: last_dip_hi <= {dph, d};
                    10'd32: dpl <= d;
                    10'd33: last_dip_lo <= {dpl, d};
                    10'd34: sph <= d;
                    10'd35: last_sport  <= {sph, d};
                    10'd36: dth <= d;
                    10'd37: last_dport  <= {dth, d};
                    default: ;
                    endcase
                    if (pos != 10'd63) pos <= pos + 10'd1;
                end
            end
        end
    end
endmodule
