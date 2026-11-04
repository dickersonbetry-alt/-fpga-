// ============================================================
// ads8681_ctrl.v : ADS8681 16-bit 1MSPS ADC 驱动 + 电压换算
//
//   硬件连接（ACG720 P7，见《引脚配置与接线说明》）：
//     ads_cs_n  A13   -> ADS8681 CS
//     ads_sclk  A14   -> ADS8681 SCLK
//     ads_sdi   A15   -> ADS8681 SDI
//     ads_sdo0  A16   <- ADS8681 SDO0
//     ads_rst_n A18   -> ADS8681 RST   (低有效)
//     ads_rvs   A19   <- ADS8681 RVS   (模块输出，低=空闲)
//
//   协议（与厂家 STM32 例程一致）：
//     SPI mode 0，MSB first，16 位为一帧
//     读数据：CS 拉低 -> 等 RVS 低 -> 发 16'h0000 -> 同帧收回 16 位转换结果 -> CS 拉高
//     写寄存器：CS 拉低 -> 等 RVS 低 -> 发 32 位 {命令,地址,数据高,数据低} -> CS 拉高
//     量程寄存器 RANGE_SEL(0x14) 写 0x0008 = 0~12.288V 单极性
//
//   电压换算：Vin = code * 12.288 / 65535
//             输出 voltage_x100 = code * 1229 >> 16（单位 0.01V，即 0~1228）
// ============================================================
module ads8681_ctrl #(
    parameter CLK_FREQ = 50_000_000,
    parameter SCLK_HZ  = 1_000_000
)(
    input  wire        clk,
    input  wire        rst_n,

    output reg         ads_cs_n   = 1'b1,
    output reg         ads_sclk   = 1'b0,
    output reg         ads_sdi    = 1'b0,
    input  wire        ads_sdo0,
    input  wire        ads_rvs,
    output reg         ads_rst_n  = 1'b0,

    output reg  [15:0] adc_code     = 16'd0,  // 原始 16 位转换结果
    output reg  [15:0] voltage_x100 = 16'd0,  // 电压 ×100（0.01V 单位）
    output reg         sample_valid = 1'b0    // 每次更新一个时钟脉冲
);

    // ---------------- 时间常数 ----------------
    localparam HALF       = (CLK_FREQ / SCLK_HZ) / 2;  // 半个 SCLK 周期
    localparam D_RST_LOW  = CLK_FREQ / 1000;           // 1 ms 复位低电平
    localparam D_RST_HIGH = CLK_FREQ / 200;            // 5 ms 复位释放等待
    localparam D_PERIOD   = CLK_FREQ / 500;            // 2 ms 采样周期 (500Hz)

    // ---------------- SPI 引擎 ----------------
    localparam S_IDLE  = 3'd0,
               S_RVS   = 3'd1,
               S_LOW   = 3'd2,
               S_LOWW  = 3'd3,
               S_HIGHW = 3'd4,
               S_END   = 3'd5;

    reg [2:0]  spi_state = S_IDLE;
    reg [5:0]  bit_cnt   = 6'd0;
    reg [15:0] half_cnt  = 16'd0;
    reg [15:0] rvs_cnt   = 16'd0;
    reg [31:0] tx_sr     = 32'd0;
    reg [31:0] rx_sr     = 32'd0;
    reg        spi_start = 1'b0;
    reg        spi_done  = 1'b0;
    reg [5:0]  spi_nbits = 6'd16;
    reg [31:0] spi_tx    = 32'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_state <= S_IDLE;
            ads_cs_n  <= 1'b1;
            ads_sclk  <= 1'b0;
            ads_sdi   <= 1'b0;
            bit_cnt   <= 6'd0;
            half_cnt  <= 16'd0;
            rvs_cnt   <= 16'd0;
            tx_sr     <= 32'd0;
            rx_sr     <= 32'd0;
            spi_done  <= 1'b0;
        end else begin
            spi_done <= 1'b0;

            case (spi_state)
            S_IDLE: begin
                ads_cs_n <= 1'b1;
                ads_sclk <= 1'b0;
                ads_sdi  <= 1'b0;
                if (spi_start) begin
                    ads_cs_n  <= 1'b0;
                    tx_sr     <= spi_tx;
                    bit_cnt   <= spi_nbits;
                    rvs_cnt   <= 16'd0;
                    spi_state <= S_RVS;
                end
            end

            // 等 RVS 变低（模块内部基准/忙信号），最多等 20us 防止卡死
            S_RVS: begin
                if (!ads_rvs || rvs_cnt >= 16'd1000)
                    spi_state <= S_LOW;
                else
                    rvs_cnt <= rvs_cnt + 16'd1;
            end

            // SCLK 低：更新 MOSI
            S_LOW: begin
                ads_sclk <= 1'b0;
                ads_sdi  <= tx_sr[bit_cnt - 6'd1];
                half_cnt <= HALF - 1;
                spi_state <= S_LOWW;
            end

            S_LOWW: begin
                if (half_cnt == 16'd0) begin
                    ads_sclk <= 1'b1;      // 上升沿，ADC 采样 MOSI
                    half_cnt <= HALF - 1;
                    spi_state <= S_HIGHW;
                end else begin
                    half_cnt <= half_cnt - 16'd1;
                end
            end

            S_HIGHW: begin
                if (half_cnt == 16'd0) begin
                    rx_sr    <= {rx_sr[30:0], ads_sdo0};
                    ads_sclk <= 1'b0;
                    if (bit_cnt == 6'd1) begin
                        bit_cnt   <= 6'd0;
                        spi_state <= S_END;
                    end else begin
                        bit_cnt   <= bit_cnt - 6'd1;
                        spi_state <= S_LOW;
                    end
                end else begin
                    half_cnt <= half_cnt - 16'd1;
                end
            end

            S_END: begin
                ads_cs_n  <= 1'b1;
                ads_sclk  <= 1'b0;
                ads_sdi   <= 1'b0;
                spi_done  <= 1'b1;
                spi_state <= S_IDLE;
            end

            default: spi_state <= S_IDLE;
            endcase
        end
    end

    // ---------------- 主流程 ----------------
    localparam M_RST      = 3'd0,
               M_RST_WAIT = 3'd1,
               M_WR       = 3'd2,
               M_WR_WAIT  = 3'd3,
               M_RD       = 3'd4,
               M_RD_WAIT  = 3'd5,
               M_RD_GAP   = 3'd6;

    reg [2:0]  m_state   = M_RST;
    reg [31:0] delay_cnt = 32'd0;
    reg [26:0] sec_cnt   = 27'd0;   // 1 秒量程重写计数器（50MHz）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_state      <= M_RST;
            ads_rst_n    <= 1'b0;
            delay_cnt    <= 32'd0;
            sec_cnt      <= 27'd0;
            spi_start    <= 1'b0;
            spi_nbits    <= 6'd16;
            spi_tx       <= 32'd0;
            adc_code     <= 16'd0;
            voltage_x100 <= 16'd0;
            sample_valid <= 1'b0;
        end else begin
            sample_valid <= 1'b0;

            case (m_state)
            M_RST: begin
                ads_rst_n <= 1'b0;
                if (delay_cnt >= D_RST_LOW) begin
                    delay_cnt <= 32'd0;
                    ads_rst_n <= 1'b1;
                    m_state   <= M_RST_WAIT;
                end else begin
                    delay_cnt <= delay_cnt + 32'd1;
                end
            end

            M_RST_WAIT: begin
                if (delay_cnt >= D_RST_HIGH) begin
                    delay_cnt <= 32'd0;
                    // 0~12.288V 单极性量程: 写 RANGE_SEL(0x14) = 0x0008
                    spi_nbits <= 6'd32;
                    spi_tx    <= {8'hD0, 8'h14, 8'h00, 8'h08};
                    spi_start <= 1'b1;
                    m_state   <= M_WR;
                end else begin
                    delay_cnt <= delay_cnt + 32'd1;
                end
            end

            M_WR: begin
                spi_start <= 1'b0;
                if (spi_done) begin
                    delay_cnt <= 32'd0;
                    m_state   <= M_WR_WAIT;
                end
            end

            M_WR_WAIT: begin
                if (delay_cnt >= CLK_FREQ / 1000) begin   // 写完后等 1ms
                    m_state <= M_RD;
                end else begin
                    delay_cnt <= delay_cnt + 32'd1;
                end
            end

            M_RD: begin
                spi_nbits <= 6'd16;
                spi_tx    <= 32'h0000_0000;   // NOP，同帧返回转换结果
                spi_start <= 1'b1;
                m_state   <= M_RD_WAIT;
            end

            M_RD_WAIT: begin
                spi_start <= 1'b0;
                if (spi_done) begin
                    adc_code     <= rx_sr[15:0];
                    // code * 12.288/65535 * 100 ≈ code * 1229 >> 16
                    voltage_x100 <= (rx_sr[15:0] * 32'd1229) >> 16;
                    sample_valid <= 1'b1;
                    delay_cnt    <= 32'd0;
                    m_state      <= M_RD_GAP;
                end
            end

            // 上电只初始化一次，之后连续采样；不再每秒复位，避免数据卡死
            M_RD_GAP: begin
                sec_cnt <= 27'd0;
                if (delay_cnt >= D_PERIOD) begin
                    delay_cnt <= 32'd0;
                    m_state   <= M_RD;
                end else begin
                    delay_cnt <= delay_cnt + 32'd1;
                end
            end

            default: m_state <= M_RST;
            endcase
        end
    end

endmodule
