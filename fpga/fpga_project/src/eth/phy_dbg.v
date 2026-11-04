// phy_dbg.v : PHY 寄存器只读诊断（不写任何寄存器）
module phy_dbg(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        init_done,
    output wire        mdc,
    inout  wire        mdio,
    output reg  [15:0] phyid_hi = 16'd0,
    output reg  [15:0] phyid_lo = 16'd0,
    output reg  [15:0] bmcr     = 16'd0,
    output reg  [15:0] bmsr     = 16'd0
);
    reg        go    = 1'b0;
    reg        mode  = 1'b1;      // 1=读
    reg [4:0]  raddr = 5'd0;
    wire [15:0] rdata;
    wire        done;

    mdio_bit_shift u_mdio (
        .Clk        (clk),
        .Rst_n      (rst_n),
        .Mode       (mode),
        .Go         (go),
        .Phy_addr   (5'b00101),
        .Reg_addr   (raddr),
        .Tx_Data    (16'd0),
        .Rx_Data    (rdata),
        .Trans_Done (done),
        .mdc        (mdc),
        .mdio       (mdio)
    );

    reg [3:0] st = 4'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= 4'd0;
            go <= 1'b0;
        end else begin
            go <= 1'b0;
            case (st)
            4'd0: if (init_done) begin raddr <= 5'd2; go <= 1'b1; st <= 4'd1; end
            4'd1: if (done) begin phyid_hi <= rdata; raddr <= 5'd3; go <= 1'b1; st <= 4'd2; end
            4'd2: if (done) begin phyid_lo <= rdata; raddr <= 5'd0; go <= 1'b1; st <= 4'd3; end
            4'd3: if (done) begin bmcr <= rdata; raddr <= 5'd1; go <= 1'b1; st <= 4'd4; end
            4'd4: if (done) begin bmsr <= rdata; st <= 4'd5; end
            default: st <= 4'd5;
            endcase
        end
    end
endmodule
