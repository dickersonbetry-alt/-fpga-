// ============================================================
// box_sync.v : 把 RK3588 回传的识别框从 gmii_rx_clk(125MHz) 域
//              安全地搬到运动控制用的 clk(50MHz) 域
//
// 为什么需要：box_rx_parse 里的 box_cnt/box_x1.. 都在 125MHz 域更新，
// 直接拿去驱动电机（50MHz 域）会有亚稳态和多位不一致风险。
// 这里用 box_rx_parse 输出的 box_toggle 做翻转握手：
//   每收到一整包（或超时清空）toggle 翻转一次，
//   同步到 clk 域后做异或边沿检测，得到一拍 ce 脉冲，
//   此时把整组框数据一次性锁存进 clk 域寄存器。
//
// 只取第 0 个框：RK3588 按置信度从高到低发送，第 0 个即最优目标。
// ============================================================
module box_sync(
    input  wire        clk,        // 50MHz 控制域
    input  wire        rst_n,

    input  wire        src_toggle, // 125MHz 域：每包翻转一次
    input  wire [4:0]  src_cnt,
    input  wire [15:0] src_x1,
    input  wire [15:0] src_y1,
    input  wire [15:0] src_x2,
    input  wire [15:0] src_y2,
    input  wire [7:0]  src_score,

    output reg  [4:0]  cnt,
    output reg  [15:0] x1,
    output reg  [15:0] y1,
    output reg  [15:0] x2,
    output reg  [15:0] y2,
    output reg  [7:0]  score,
    output reg         ce          // 锁存完成，一个 clk 周期
);

    // 两级同步 + 边沿检测
    reg [1:0] tgl_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tgl_sync <= 2'b00;
        else        tgl_sync <= {tgl_sync[0], src_toggle};
    end
    wire upd = tgl_sync[1] ^ tgl_sync[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= 5'd0;
            x1    <= 16'd0;
            y1    <= 16'd0;
            x2    <= 16'd0;
            y2    <= 16'd0;
            score <= 8'd0;
            ce    <= 1'b0;
        end else begin
            ce <= 1'b0;
            if (upd) begin
                cnt   <= src_cnt;
                x1    <= src_x1;
                y1    <= src_y1;
                x2    <= src_x2;
                y2    <= src_y2;
                score <= src_score;
                ce    <= 1'b1;
            end
        end
    end

endmodule
