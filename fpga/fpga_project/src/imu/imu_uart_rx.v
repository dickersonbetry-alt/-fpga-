// ============================================================
// imu_uart_rx.v : IMU 串口接收（115200 8N1）
//
// 波特率 115200（亚博九轴 IMU 驱动库 YbImuSerialLib 中固定 115200）
// 50MHz 下 1 bit ≈ 434 个时钟
//
// 抗干扰处理（电机 PWM 噪声容易耦合到 IMU 的 TX 线上）：
//   1. 起始位中点复核：中点仍为低才认为是真正的起始位，
//      避免毛刺触发假帧；
//   2. 每个数据位在中心附近采样 3 次（±12 个时钟）做多数表决，
//      单点噪声不会翻转数据位；
//   3. 停止位为 1 才输出 valid。
// ============================================================
module imu_uart_rx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,        // IMU TX -> FPGA RX
    output reg  [7:0] data,      // 收到的字节
    output reg        valid      // 一个时钟周期有效
);

    localparam integer BIT_PERIOD = CLK_FREQ / BAUD;   // 434
    localparam integer HALF       = BIT_PERIOD / 2;    // 217

    // 2 级同步
    reg [1:0] rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_sync <= 2'b11;
        else        rx_sync <= {rx_sync[0], rx};
    end
    wire rx_f = rx_sync[1];

    localparam [1:0] S_IDLE = 2'd0,
                     S_HALF = 2'd1,
                     S_DATA = 2'd2,
                     S_STOP = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift;
    reg        s0, s1;      // 数据位中心前两次采样

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 16'd0;
            bit_idx <= 3'd0;
            shift   <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
            s0      <= 1'b0;
            s1      <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)
            S_IDLE: begin
                if (!rx_f) begin          // 起始位
                    cnt   <= 16'd0;
                    state <= S_HALF;
                end
            end

            // 等半个位，到起始位中点
            S_HALF: begin
                if (cnt >= HALF - 1) begin
                    if (!rx_f) begin      // 中点仍为低，起始位有效
                        cnt     <= 16'd0;
                        bit_idx <= 3'd0;
                        state   <= S_DATA;
                    end else begin
                        state   <= S_IDLE;  // 毛刺，丢弃
                    end
                end else
                    cnt <= cnt + 16'd1;
            end

            // 每个数据位：中心前 25/13 个时钟采两次，中心再采一次，多数表决
            S_DATA: begin
                if (cnt == BIT_PERIOD - 25) begin
                    s0  <= rx_f;
                    cnt <= cnt + 16'd1;
                end else if (cnt == BIT_PERIOD - 13) begin
                    s1  <= rx_f;
                    cnt <= cnt + 16'd1;
                end else if (cnt >= BIT_PERIOD - 1) begin
                    shift <= {(s0 & s1) | (s1 & rx_f) | (s0 & rx_f), shift[7:1]}; // LSB 先到
                    cnt   <= 16'd0;
                    if (bit_idx == 3'd7)
                        state <= S_STOP;
                    else
                        bit_idx <= bit_idx + 3'd1;
                end else
                    cnt <= cnt + 16'd1;
            end

            S_STOP: begin
                if (cnt >= BIT_PERIOD - 1) begin
                    cnt   <= 16'd0;
                    state <= S_IDLE;
                    data  <= shift;
                    valid <= rx_f;                 // 停止位为 1 才算有效
                end else
                    cnt <= cnt + 16'd1;
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
