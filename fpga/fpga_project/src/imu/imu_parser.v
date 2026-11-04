// ============================================================
// imu_parser.v : 亚博九轴 IMU（YbImu）帧解析
//
// 协议见《高精度IMU姿态传感器\6.附件\通信协议\通信协议.xlsx》：
//   帧格式： 0x7E 0x23 | len | func | data... | checksum
//   len      = 整帧字节数（= 校验位下标 + 1，含包头和校验）
//   data 长度 = len - 5
//   checksum = (0x7E + 0x23 + len + func + Σdata) & 0xFF   （不含校验位本身）
//
// 厂家 C 驱动（imu_uart_driver.c）的等价写法是 data_length = len - 4，
// 其中 frame_buffer 的最后一个字节就是校验位，两者完全等价。
//
// 关心的功能字：
//   0x26 欧拉角  len=0x11=17，data = roll/pitch/yaw 三个 float32（弧度，小端）
//   0x04 原始数据 len=0x17=23，data = ax/ay/az/gx/gy/gz/mx/my/mz 九个 int16（小端）
//   0x16 四元数  len=0x15=21（本工程未使用）
//   0x32 气压计  len=0x15=21（十轴模块才有，本工程未使用）
//   0x01 版本号  len=0x08=8
//   0x81 校准状态 len=0x07=7
// ============================================================
module imu_parser(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  rx_byte,
    input  wire        rx_valid,

    output reg  [31:0] roll,      // float32，弧度
    output reg  [31:0] pitch,
    output reg  [31:0] yaw,
    output reg         euler_valid,   // 收到一帧有效欧拉角时拉高一个时钟

    output reg  signed [15:0] ax, ay, az,
    output reg  signed [15:0] gx, gy, gz,
    output reg  signed [15:0] mx, my, mz,
    output reg         raw_valid,

    output reg  [15:0] frame_cnt,     // 校验通过的有效帧总数
    output reg  [15:0] euler_cnt,     // 0x26 帧数
    output reg  [15:0] raw_cnt,       // 0x04 帧数
    output reg  [15:0] cs_err_cnt,    // 校验失败次数
    output reg  [7:0]  last_func,     // 最近一次解析到的功能字
    output reg  [7:0]  last_len       // 最近一次解析到的长度字节
);

    localparam [7:0] H1 = 8'h7E, H2 = 8'h23;
    localparam [7:0] FUNC_RAW   = 8'h04;
    localparam [7:0] FUNC_EULER = 8'h26;

    localparam [2:0] S_IDLE  = 3'd0,
                     S_H2    = 3'd1,
                     S_LEN   = 3'd2,
                     S_FUNC  = 3'd3,
                     S_DATA  = 3'd4,
                     S_CHK   = 3'd5;

    reg [2:0]  state;
    reg [7:0]  len;
    reg [7:0]  func;
    reg [7:0]  sum;
    reg [4:0]  dcnt;
    reg [7:0]  rxbuf [0:23];
    reg [17:0] tmo;            // 半帧超时计数（5ms @50MHz）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            len         <= 8'd0;
            func        <= 8'd0;
            sum         <= 8'd0;
            dcnt        <= 5'd0;
            roll        <= 32'd0;
            pitch       <= 32'd0;
            yaw         <= 32'd0;
            euler_valid <= 1'b0;
            ax <= 16'sd0; ay <= 16'sd0; az <= 16'sd0;
            gx <= 16'sd0; gy <= 16'sd0; gz <= 16'sd0;
            mx <= 16'sd0; my <= 16'sd0; mz <= 16'sd0;
            raw_valid   <= 1'b0;
            frame_cnt   <= 16'd0;
            euler_cnt   <= 16'd0;
            raw_cnt     <= 16'd0;
            cs_err_cnt  <= 16'd0;
            last_func   <= 8'd0;
            last_len    <= 8'd0;
            tmo         <= 18'd0;
        end else begin
            euler_valid <= 1'b0;
            raw_valid   <= 1'b0;

            // 帧内超过 5ms 没有新字节，说明丢字节/卡住，强制回到找包头
            if (rx_valid)
                tmo <= 18'd0;
            else if (tmo < 18'd250000)
                tmo <= tmo + 18'd1;

            if (!rx_valid && state != S_IDLE && tmo >= 18'd250000)
                state <= S_IDLE;

            if (rx_valid) begin
                case (state)
                S_IDLE: begin
                    if (rx_byte == H1) state <= S_H2;
                end
                S_H2: begin
                    if (rx_byte == H2)      state <= S_LEN;
                    else if (rx_byte == H1) state <= S_H2;
                    else                    state <= S_IDLE;
                end
                S_LEN: begin
                    // len 是整帧字节数；最小 7（7E 23 07 xx .. CS），本工程用到的最大 23
                    if (rx_byte >= 8'd7 && rx_byte <= 8'd28) begin
                        len      <= rx_byte;
                        last_len <= rx_byte;
                        sum      <= H1 + H2 + rx_byte;
                        state    <= S_FUNC;
                    end else
                        state <= S_IDLE;
                end
                S_FUNC: begin
                    func      <= rx_byte;
                    last_func <= rx_byte;
                    sum       <= sum + rx_byte;
                    dcnt      <= 5'd0;
                    state     <= S_DATA;
                end
                S_DATA: begin
                    sum <= sum + rx_byte;
                    if (dcnt < 5'd24) rxbuf[dcnt] <= rx_byte;
                    if (dcnt + 5'd1 >= len - 5'd5)
                        state <= S_CHK;
                    else
                        dcnt <= dcnt + 5'd1;
                end
                S_CHK: begin
                    if (rx_byte == sum) begin
                        state     <= S_IDLE;
                        frame_cnt <= frame_cnt + 16'd1;
                        if (func == FUNC_EULER) begin
                            // 3 个 float32，小端
                            roll        <= {rxbuf[3], rxbuf[2], rxbuf[1], rxbuf[0]};
                            pitch       <= {rxbuf[7], rxbuf[6], rxbuf[5], rxbuf[4]};
                            yaw         <= {rxbuf[11], rxbuf[10], rxbuf[9], rxbuf[8]};
                            euler_valid <= 1'b1;
                            euler_cnt   <= euler_cnt + 16'd1;
                        end else if (func == FUNC_RAW) begin
                            ax <= {rxbuf[1],  rxbuf[0]};
                            ay <= {rxbuf[3],  rxbuf[2]};
                            az <= {rxbuf[5],  rxbuf[4]};
                            gx <= {rxbuf[7],  rxbuf[6]};
                            gy <= {rxbuf[9],  rxbuf[8]};
                            gz <= {rxbuf[11], rxbuf[10]};
                            mx <= {rxbuf[13], rxbuf[12]};
                            my <= {rxbuf[15], rxbuf[14]};
                            mz <= {rxbuf[17], rxbuf[16]};
                            raw_valid <= 1'b1;
                            raw_cnt   <= raw_cnt + 16'd1;
                        end
                    end else begin
                        // 校验失败：若校验字节本身是包头，直接当作新帧头
                        cs_err_cnt <= cs_err_cnt + 16'd1;
                        if (rx_byte == H1)
                            state <= S_H2;
                        else
                            state <= S_IDLE;
                    end
                end
                default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
