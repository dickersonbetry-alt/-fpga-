// ============================================================
// xpt2046.v : 电阻触摸屏 XPT2046 SPI 读取（轮询 X/Y）
//
// 时序：CS 拉低 → 8bit 命令 → 16bit 读（取高 12bit）
// 命令 0x90 = 读 X（通道1，12 位差分），0xD0 = 读 Y（通道5）
// SPI 时钟 500kHz（杜邦线较长时更稳），模式 0
// 每次转换后 CS 拉高保持 GAP 个时钟再开始下一次，保证串口复位
// 每 POLL_MS 毫秒轮询一次 X/Y，坐标范围 0~4095。
// ============================================================
module xpt2046 #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer SCLK_HZ  = 500_000,
    parameter integer POLL_MS  = 20
)(
    input  wire        clk,
    input  wire        rst_n,

    // 触摸屏 SPI
    output reg         cs_n,
    output reg         sck,
    output reg         mosi,
    input  wire        miso,

    output reg  [11:0] x_val,
    output reg  [11:0] y_val,
    output reg         pressed,

    // 调试：原始 16bit 数据和采样有效脉冲
    output reg  [15:0] raw_x,
    output reg  [15:0] raw_y,
    output reg         sample_valid
);

    localparam integer HALF = CLK_FREQ / (SCLK_HZ * 2);   // 半周期时钟数
    localparam integer GAP  = CLK_FREQ / 100000;          // CS 高保持 10us
    localparam integer POLL = (CLK_FREQ / 1000) * POLL_MS;

    localparam [3:0] S_IDLE    = 4'd0,
                     S_CMD_LO  = 4'd1,
                     S_CMD_HI  = 4'd2,
                     S_READ_LO = 4'd3,
                     S_READ_HI = 4'd4,
                     S_DONE    = 4'd5,
                     S_GAP     = 4'd6;

    reg [3:0]  state    = S_IDLE;
    reg [31:0] wait_cnt = 32'd0;
    reg [31:0] gap_cnt  = 32'd0;
    reg [15:0] half_cnt = 16'd0;
    reg        tick     = 1'b0;
    reg [7:0]  cmd      = 8'h90;
    reg [3:0]  bitcnt   = 4'd0;
    reg [15:0] rx       = 16'd0;
    reg        phase    = 1'b0;

    // 半位定时
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            half_cnt <= 16'd0;
            tick     <= 1'b0;
        end else if (half_cnt >= HALF - 1) begin
            half_cnt <= 16'd0;
            tick     <= 1'b1;
        end else begin
            half_cnt <= half_cnt + 16'd1;
            tick     <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            cs_n     <= 1'b1;
            sck      <= 1'b0;
            mosi     <= 1'b0;
            x_val    <= 12'd0;
            y_val    <= 12'd0;
            pressed  <= 1'b0;
            raw_x    <= 16'd0;
            raw_y    <= 16'd0;
            sample_valid <= 1'b0;
            wait_cnt <= 32'd0;
            gap_cnt  <= 32'd0;
            bitcnt   <= 4'd0;
            rx       <= 16'd0;
            phase    <= 1'b0;
        end else begin
            sample_valid <= 1'b0;
            case (state)
                S_IDLE: begin
                    cs_n <= 1'b1;
                    sck  <= 1'b0;
                    if (wait_cnt >= POLL - 1) begin
                        wait_cnt <= 32'd0;
                        cs_n     <= 1'b0;
                        phase    <= 1'b0;
                        cmd      <= 8'h90;
                        bitcnt   <= 4'd7;
                        mosi     <= 1'b1;
                        state    <= S_CMD_LO;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                S_CMD_LO: if (tick) begin
                    sck   <= 1'b1;
                    state <= S_CMD_HI;
                end

                S_CMD_HI: if (tick) begin
                    sck <= 1'b0;
                    if (bitcnt == 4'd0) begin
                        bitcnt <= 4'd15;
                        state  <= S_READ_LO;
                    end else begin
                        bitcnt <= bitcnt - 4'd1;
                        mosi   <= cmd[bitcnt - 4'd1];
                        state  <= S_CMD_LO;
                    end
                end

                S_READ_LO: if (tick) begin
                    sck   <= 1'b1;
                    rx    <= {rx[14:0], miso};
                    state <= S_READ_HI;
                end

                S_READ_HI: if (tick) begin
                    sck <= 1'b0;
                    if (bitcnt == 4'd0) begin
                        state <= S_DONE;
                    end else begin
                        bitcnt <= bitcnt - 4'd1;
                        state  <= S_READ_LO;
                    end
                end

                S_DONE: begin
                    cs_n <= 1'b1;
                    if (!phase) begin
                        raw_x   <= rx;
                        x_val   <= rx[14:3];
                        phase   <= 1'b1;
                        cmd     <= 8'hD0;
                        bitcnt  <= 4'd7;
                        mosi    <= 1'b1;
                        gap_cnt <= 32'd0;
                        state   <= S_GAP;
                    end else begin
                        raw_y   <= rx;
                        y_val   <= rx[14:3];
                        sample_valid <= 1'b1;
                        pressed <= (x_val > 12'd100) && (x_val < 12'd4000) &&
                                   (rx[14:3] > 12'd100) && (rx[14:3] < 12'd4000);
                        state   <= S_IDLE;
                    end
                end

                // CS 高保持，给 XPT2046 复位串口
                S_GAP: begin
                    cs_n <= 1'b1;
                    sck  <= 1'b0;
                    if (gap_cnt >= GAP - 1) begin
                        gap_cnt <= 32'd0;
                        cs_n    <= 1'b0;
                        state   <= S_CMD_LO;
                    end else begin
                        gap_cnt <= gap_cnt + 32'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
