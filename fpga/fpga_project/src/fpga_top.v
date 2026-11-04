// ============================================================
// fpga_top.v : 顶层（四路闭环电机 + OV5640 摄像头 TFT 显示）
// 引脚定义见《ACG720机器人外设引脚配置与接线说明.docx》
// ============================================================
module fpga_top(
    input  clk,
    input  rst_n,

    // ---------------- IMU（UART） ----------------
    input  imu_rx,

    // ---------------- 电阻触摸 XPT2046（SPI） ----------------
    output       tp_cs,
    output       tp_sck,
    output       tp_mosi,
    input        tp_miso,


    // ---------------- ADS8681（激光测距 0-10V 模拟量采集） ----------------
    output       ads_cs_n,
    output       ads_sclk,
    output       ads_sdi,
    input        ads_sdo0,
    output       ads_rst_n,
    input        ads_rvs,

    // ---------------- 四路闭环电机 ----------------
    input  encoder0_a,
    input  encoder0_b,
    input  encoder1_a,
    input  encoder1_b,
    input  encoder2_a,
    input  encoder2_b,
    input  encoder3_a,
    input  encoder3_b,
    output motor_stby,
    output motor0_A,
    output motor0_B,
    output motor1_A,
    output motor1_B,
    output motor2_A,
    output motor2_B,
    output motor3_A,
    output motor3_B,
    output motor0_pwm,
    output motor1_pwm,
    output motor2_pwm,
    output motor3_pwm,

    // ---------------- OV5640 摄像头（P4 接口） ----------------
    output       camera_sclk,
    inout        camera_sdat,
    input        camera_vsync,
    input        camera_href,
    input        camera_pclk,
    output       camera_xclk,
    input  [7:0] camera_data,
    output       camera_rst_n,
    output       camera_pwdn,

    // ---------------- TFT 显示屏（40 针 FPC） ----------------
    output [15:0] TFT_rgb,
    output        TFT_hs,
    output        TFT_vs,
    output        TFT_clk,
    output        TFT_de,
    output        TFT_pwm,

    // ---------------- 千兆以太网（RTL8211E，RGMII） ----------------
    output        eth_rst_n,
    output        eth_mdc,
    inout         eth_mdio,
    output        rgmii_gtxc,
    output [3:0]  rgmii_txd,
    output        rgmii_txen,
    input         rgmii_rxc,
    input         rgmii_rxdv,
    input  [3:0]  rgmii_rxd,
    output        uart_tx_pin,
    input         uart_rx_pin,

    // ---------------- DDR3 帧缓存 ----------------
    output [13:0] O_ddr_addr,
    output [2:0]  O_ddr_ba,
    output        O_ddr_cs_n,
    output        O_ddr_ras_n,
    output        O_ddr_cas_n,
    output        O_ddr_we_n,
    output        O_ddr_clk,
    output        O_ddr_clk_n,
    output        O_ddr_cke,
    output        O_ddr_odt,
    output        O_ddr_reset_n,
    output [1:0]  O_ddr_dqm,
    inout  [15:0] IO_ddr_dq,
    inout  [1:0]  IO_ddr_dqs,
    inout  [1:0]  IO_ddr_dqs_n
);

    // ==================== 四路闭环电机 ====================
    reg [31:0] move_data = 555050;

    // 电机侧暂不复位（此前怀疑 B21 复位脚被拉低导致轮子不动，已用初值修好；
    // 这里保持原行为，避免改动已验证过的电机部分）
    wire rst_motor = 1'b1;

    assign motor_stby = rst_motor;

    wire signed [31:0] enc0, enc1, enc2, enc3;

    // 摄像头像素流（以太网发送）
    wire        cam_pclk;
    wire [15:0] cam_pixel;
    wire        cam_valid;
    wire        cam_hs;
    wire        cam_vs;
    wire [11:0] cam_x;
    wire [11:0] cam_y;
    wire signed [31:0] pos_sum = enc0 + enc1 + enc2 + enc3;
    wire signed [31:0] pos_avg = pos_sum >>> 2;   // 四轮平均位置
    reg  signed [31:0] by_start = 32'sd0;
    wire signed [31:0] disp_cnt = pos_avg - by_start;  // 本次移动距离(counts)
    wire               pos_mode;
    wire               pid_clr;
    wire signed [31:0] pos_delta = 32'sd4300;   // 1 米 = 4300 counts (43.0 counts/cm)

    // 追踪用：RK3588 识别框（由 box_sync 同步到本 clk 域）
    // trk_move 在后面的「以太网接收」段算出，这里先声明，Verilog 里连续赋值与引用顺序无关
    wire [4:0]  trk_cnt;
    wire [15:0] trk_x1, trk_y1, trk_x2, trk_y2;
    wire [7:0]  trk_score;
    wire        trk_ce;
    wire [31:0] trk_move;

    move move1(
        .clk        (clk),
        .rst_n      (rst_motor),
        .move       (move_data),
        .enc0_a     (encoder0_a),
        .enc0_b     (encoder0_b),
        .enc1_a     (encoder1_a),
        .enc1_b     (encoder1_b),
        .enc2_a     (encoder2_a),
        .enc2_b     (encoder2_b),
        .enc3_a     (encoder3_a),
        .enc3_b     (encoder3_b),
        .motor0_A   (motor0_A),
        .motor0_B   (motor0_B),
        .motor1_A   (motor1_A),
        .motor1_B   (motor1_B),
        .motor2_A   (motor2_A),
        .motor2_B   (motor2_B),
        .motor3_A   (motor3_A),
        .motor3_B   (motor3_B),
        .motor0_pwm (motor0_pwm),
        .motor1_pwm (motor1_pwm),
        .motor2_pwm (motor2_pwm),
        .motor3_pwm (motor3_pwm),
        .pos0       (enc0),
        .pos1       (enc1),
        .pos2       (enc2),
        .pos3       (enc3),
        .pos_mode   (pos_mode),
        .pos_delta  (pos_delta),
        .pos_fb     (pos_avg),
        .rot_en     (rot_en),
        .rot_v10    (rot_v10)
    );



    // ==================== IMU（UART 115200，F13） ====================
    wire [7:0] imu_byte;
    wire       imu_byte_valid;

    imu_uart_rx #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200)
    ) u_imu_uart (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (imu_rx),
        .data  (imu_byte),
        .valid (imu_byte_valid)
    );

    wire [31:0] imu_roll_f, imu_pitch_f, imu_yaw_f;
    wire        imu_euler_valid;
    wire signed [15:0] imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz, imu_mx, imu_my, imu_mz;
    wire        imu_raw_valid;
    wire [15:0] imu_frame_cnt;
    wire [15:0] imu_euler_cnt, imu_raw_cnt, imu_cs_err;
    wire [7:0]  imu_last_func, imu_last_len;

    // IMU 串口收到的字节总数（诊断：用来判断 TX 线上到底有没有数据）
    reg [15:0] imu_byte_cnt = 16'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               imu_byte_cnt <= 16'd0;
        else if (imu_byte_valid)  imu_byte_cnt <= imu_byte_cnt + 16'd1;
    end

    imu_parser u_imu_parser (
        .clk         (clk),
        .rst_n       (rst_n),
        .rx_byte     (imu_byte),
        .rx_valid    (imu_byte_valid),
        .roll        (imu_roll_f),
        .pitch       (imu_pitch_f),
        .yaw         (imu_yaw_f),
        .euler_valid (imu_euler_valid),
        .ax (imu_ax), .ay (imu_ay), .az (imu_az),
        .gx (imu_gx), .gy (imu_gy), .gz (imu_gz),
        .mx (imu_mx), .my (imu_my), .mz (imu_mz),
        .raw_valid   (imu_raw_valid),
        .frame_cnt   (imu_frame_cnt),
        .euler_cnt   (imu_euler_cnt),
        .raw_cnt     (imu_raw_cnt),
        .cs_err_cnt  (imu_cs_err),
        .last_func   (imu_last_func),
        .last_len    (imu_last_len)
    );

    wire signed [15:0] imu_roll_deg, imu_pitch_deg, imu_yaw_deg;

    float_to_deg u_conv_roll (
        .clk (clk), .rst_n (rst_n),
        .in_valid (imu_euler_valid), .fin (imu_roll_f),
        .out_valid (), .deg_x10 (imu_roll_deg)
    );

    float_to_deg u_conv_pitch (
        .clk (clk), .rst_n (rst_n),
        .in_valid (imu_euler_valid), .fin (imu_pitch_f),
        .out_valid (), .deg_x10 (imu_pitch_deg)
    );

    float_to_deg u_conv_yaw (
        .clk (clk), .rst_n (rst_n),
        .in_valid (imu_euler_valid), .fin (imu_yaw_f),
        .out_valid (), .deg_x10 (imu_yaw_deg)
    );

    // ==================== IMU 上电自动对零 ====================
    // IMU 返回的是绝对姿态，而且上电后要过几秒才开始输出数据。
    // 收到第一帧欧拉角后：先丢弃 16 帧（约 0.6 s），
    // 再累计 64 帧（约 2.6 s）取平均作为零点，之后显示相对角度。
    // 对零期间屏幕显示 "ZERO..."，不显示角度数据。
    reg signed [31:0] acc_roll  = 32'sd0;
    reg signed [31:0] acc_pitch = 32'sd0;
    reg signed [31:0] acc_yaw   = 32'sd0;
    reg [6:0]         cal_cnt   = 7'd0;      // 0..79
    reg               cal_done  = 1'b0;
    reg signed [15:0] off_roll  = 16'sd0;
    reg signed [15:0] off_pitch = 16'sd0;
    reg signed [15:0] off_yaw   = 16'sd0;
    reg [2:0]         euler_vld_pipe = 3'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_roll       <= 32'sd0;
            acc_pitch      <= 32'sd0;
            acc_yaw        <= 32'sd0;
            cal_cnt        <= 7'd0;
            cal_done       <= 1'b0;
            off_roll       <= 16'sd0;
            off_pitch      <= 16'sd0;
            off_yaw        <= 16'sd0;
            euler_vld_pipe <= 3'd0;
        end else begin
            // imu_euler_valid 到 deg_x10 更新有 3 级流水延迟
            euler_vld_pipe <= {euler_vld_pipe[1:0], imu_euler_valid};

            if (euler_vld_pipe[2] && !cal_done) begin
                if (cal_cnt < 7'd16) begin
                    cal_cnt <= cal_cnt + 7'd1;            // 丢弃前 16 帧
                end else if (cal_cnt == 7'd79) begin
                    // 累计满 64 帧，取平均作为零点
                    off_roll  <= (acc_roll  + imu_roll_deg ) >>> 6;
                    off_pitch <= (acc_pitch + imu_pitch_deg) >>> 6;
                    off_yaw   <= (acc_yaw   + imu_yaw_deg  ) >>> 6;
                    cal_done  <= 1'b1;
                end else begin
                    acc_roll  <= acc_roll  + imu_roll_deg;
                    acc_pitch <= acc_pitch + imu_pitch_deg;
                    acc_yaw   <= acc_yaw   + imu_yaw_deg;
                    cal_cnt   <= cal_cnt + 7'd1;
                end
            end
        end
    end

    wire signed [15:0] roll_disp  = cal_done ? (imu_roll_deg  - off_roll ) : 16'sd0;
    wire signed [15:0] pitch_disp = cal_done ? (imu_pitch_deg - off_pitch) : 16'sd0;
    wire signed [15:0] yaw_diff   = imu_yaw_deg - off_yaw;
    wire signed [15:0] yaw_wrap   = (yaw_diff > 16'sd1800)  ? (yaw_diff - 16'sd3600) :
                                    (yaw_diff < -16'sd1800) ? (yaw_diff + 16'sd3600) : yaw_diff;
    wire signed [15:0] yaw_disp   = cal_done ? yaw_wrap : 16'sd0;

    // 姿态角直接用 IMU 返回的欧拉角（0x26 帧，内部已融合），不在 FPGA 里积分陀螺。
    // YAW_SIGN：实测左转时 yaw_disp 变大就是 +1；如果转向反了，改成 -1。
    localparam         YAW_SIGN = 1'b1;
    wire signed [15:0] yaw_ctrl = YAW_SIGN ? yaw_disp : -yaw_disp;

    wire imu_zeroing = ~cal_done;

    // ==================== ADS8681 激光测距模拟量 ====================
    wire [15:0] laser_volt_x100;

    ads8681_ctrl #(
        .CLK_FREQ (50_000_000),
        .SCLK_HZ  (1_000_000)
    ) u_ads8681 (
        .clk          (clk),
        .rst_n        (rst_n),
        .ads_cs_n     (ads_cs_n),
        .ads_sclk     (ads_sclk),
        .ads_sdi      (ads_sdi),
        .ads_sdo0     (ads_sdo0),
        .ads_rvs      (ads_rvs),
        .ads_rst_n    (ads_rst_n),
        .adc_code     (),
        .voltage_x100 (laser_volt_x100),
        .sample_valid ()
    );

    // ==================== 电阻触摸 XPT2046（调试：先显示原始坐标） ====================
    wire [11:0] tp_x;
    wire [11:0] tp_y;
    wire        tp_pressed;
    wire        tp_valid;


    xpt2046 #(
        .CLK_FREQ (50_000_000),
        .SCLK_HZ  (500_000),
        .POLL_MS  (20)
    ) u_touch (
        .clk     (clk),
        .rst_n   (rst_n),
        .cs_n    (tp_cs),
        .sck     (tp_sck),
        .mosi    (tp_mosi),
        .miso    (tp_miso),
        .x_val   (tp_x),
        .y_val   (tp_y),
        .pressed (tp_pressed),
        .sample_valid (tp_valid)
    );


    // ==================== 九宫格 -> 电机命令 ====================
    // 九宫格外框（实测四角）：
    //   上边 tp_x≈3283，下边 tp_x≈1373；左边 tp_y≈1072，右边 tp_y≈277
    // 把触摸原始坐标映射到屏幕像素（九宫格画在 x608~795，y60~323）
    wire [11:0] gdx = (tp_y >= 12'd1072) ? 12'd0 : (12'd1072 - tp_y);
    wire [11:0] gdy = (tp_x >= 12'd3283) ? 12'd0 : (12'd3283 - tp_x);
    wire [19:0] gsx = gdx * 12'd242;   // 188/795 ~= 242/1024
    wire [19:0] gsy = gdy * 12'd142;   // 264/1910 ~= 142/1024
    wire [11:0] gx = 12'd608 + gsx[19:10];
    wire [11:0] gy = 12'd60  + gsy[19:10];

    wire [10:0] ox = gx - 12'd608;
    wire [10:0] oy = gy - 12'd60;
    wire [1:0]  col_now = (ox < 11'd64) ? 2'd0 : (ox < 11'd128) ? 2'd1 : 2'd2;
    wire [1:0]  row_now = (oy < 11'd92) ? 2'd0 : (oy < 11'd184) ? 2'd1 : 2'd2;
    wire [10:0] cxo = (col_now == 2'd0) ? ox : (col_now == 2'd1) ? (ox - 11'd64) : (ox - 11'd128);
    wire [10:0] cyo = (row_now == 2'd0) ? oy : (row_now == 2'd1) ? (oy - 11'd92) : (oy - 11'd184);

    // 先判断是否在九宫格外框内（实测外角：tp_x 1365~3310，tp_y 258~1082）
    // 框外直接不响应；框内再判断是否落在实际格子（60x80）内，间隙也不响应
    wire in_grid = tp_pressed &&
                   (tp_x <= 12'd3310) && (tp_x >= 12'd1365) &&
                   (tp_y <= 12'd1082) && (tp_y >= 12'd258);
    wire in_btn_area = in_grid &&
                       (ox <= 11'd187) && (oy <= 11'd263) &&
                       (cxo < 11'd60)  && (cyo < 11'd80);

    wire [3:0] cell_now = {2'd0, row_now} * 3 + {2'd0, col_now};

    // 0.5 秒稳定判定：连续 25 次触摸采样（20ms/次）都在同一格才有效
    // 抬手瞬间的坐标抖动不会再触发错误按键
    reg [4:0] stable_cnt  = 5'd0;
    reg [3:0] stable_cell = 4'hF;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stable_cnt  <= 5'd0;
            stable_cell <= 4'hF;
        end else if (tp_valid) begin
            if (!in_btn_area) begin
                stable_cnt  <= 5'd0;
                stable_cell <= 4'hF;
            end else if (cell_now == stable_cell) begin
                if (stable_cnt < 5'd25)
                    stable_cnt <= stable_cnt + 5'd1;
            end else begin
                stable_cell <= cell_now;
                stable_cnt  <= 5'd0;
            end
        end
    end

    wire cell_valid = in_btn_area && (stable_cnt >= 5'd25);

    // 按一下走 3 秒：方向键稳定按下后启动计时，3 秒后自动停车/熄灭
    reg [3:0]  dir_sel     = 4'd4;
    reg        dir_run     = 1'b0;
    reg [27:0] dir_cnt     = 28'd0;
    reg        cell_valid_d = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cell_valid_d <= 1'b0;
        else        cell_valid_d <= cell_valid;
    end

    wire dir_press  = cell_valid && !cell_valid_d && (stable_cell != 4'd4);
    wire stop_press = cell_valid && !cell_valid_d && (stable_cell == 4'd4);

    // 快速停止：停止格连续 3 次采样（约 60ms）就生效，不等 0.5 秒
    reg [2:0] stop_cnt = 3'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) stop_cnt <= 3'd0;
        else if (tp_valid) begin
            if (in_btn_area && (cell_now == 4'd4)) begin
                if (stop_cnt < 3'd3) stop_cnt <= stop_cnt + 3'd1;
            end else begin
                stop_cnt <= 3'd0;
            end
        end
    end
    wire stop_fast = (stop_cnt >= 3'd3);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dir_sel <= 4'd4;
            dir_run <= 1'b0;
            dir_cnt <= 28'd0;
        end else if (stop_press || stop_fast || mode_valid) begin
            dir_sel <= 4'd4;
            dir_run <= 1'b0;
            dir_cnt <= 28'd0;
        end else if (dir_press) begin
            dir_sel <= stable_cell;
            dir_run <= 1'b1;
            dir_cnt <= 28'd0;
        end else if (dir_run) begin
            if (dir_cnt >= 28'd149_999_999) begin
                dir_run <= 1'b0;
                dir_cnt <= 28'd0;
            end else begin
                dir_cnt <= dir_cnt + 28'd1;
            end
        end
    end

    // 按钮互斥：方向键 / 避障 / 绕障 / 停止 只能有一个亮
    reg [1:0] mode_sel = 2'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            mode_sel <= 2'd0;
        else if (stop_press || stop_fast || dir_press)
            mode_sel <= 2'd0;
        else if ((mode_sel == 2'd1) && (laser_volt_x100 < 16'd270))
            mode_sel <= 2'd0;          // 电压低于 2.7V 退出避障
        else if (mode_valid && ((mode_cand != 2'd1) || (laser_volt_x100 > 16'd270)))
            mode_sel <= mode_cand;
    end

    wire [3:0] tp_cell = dir_run ? dir_sel : 4'd4;


    // ==================== 避障逻辑 ====================
    // 选避障：先静止 1 秒 -> 以 10% 前进 -> 5V 开始减速 -> 2.6V 停车
    reg [1:0]  avoid_state = 2'd0;   // 0=准备 1=静止等待 2=前进
    reg [26:0] avoid_cnt   = 27'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            avoid_state <= 2'd0;
            avoid_cnt   <= 27'd0;
        end else if (mode_sel != 2'd1) begin
            avoid_state <= 2'd0;
            avoid_cnt   <= 27'd0;
        end else if (avoid_state == 2'd0) begin
            avoid_state <= 2'd1;          // 先静止 1 秒
            avoid_cnt   <= 27'd0;
        end else if (avoid_state == 2'd1) begin
            if (avoid_cnt >= 27'd49_999_999) begin
                avoid_state <= 2'd2;
                avoid_cnt   <= 27'd0;
            end else begin
                avoid_cnt <= avoid_cnt + 27'd1;
            end
        end
    end

    // ==================== 动作序列 ====================
    // 按绕障：前进 1 米 -> 停 1 秒（停稳）-> 左转 90 度 -> 停 0.5 秒（停稳）
    localparam [2:0] SEQ_IDLE = 3'd0,
                     SEQ_RCL0 = 3'd1,
                     SEQ_FWD  = 3'd2,
                     SEQ_STOP = 3'd3,
                     SEQ_RCL1 = 3'd4,
                     SEQ_TURN = 3'd5,
                     SEQ_DONE = 3'd6;

    reg [2:0]  seq      = SEQ_IDLE;
    reg [29:0] seq_cnt  = 30'd0;
    reg        mode2_d  = 1'b0;
    reg signed [31:0] yaw_target = 32'sd0;
    reg [26:0] turn_settle = 27'd0;

    // 原「绕障」动作序列（前进 1 米 → 停 → 左转 90°）已停用：
    // 模式 2 现在改成「追踪」，直接由 RK3588 识别框生成 move_data，不再走这个序列机。
    wire mode2_start = 1'b0;

    wire signed [31:0] by_target = by_start - 32'sd4300;   // 前进 = 位置递减
    wire signed [31:0] by_err2   = by_target - pos_avg;
    reg [26:0] settle_cnt = 27'd0;
    wire by_near = (by_err2 <= 32'sd10) && (by_err2 >= -32'sd10);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            settle_cnt <= 27'd0;
        else if (!by_near)
            settle_cnt <= 27'd0;
        else if (settle_cnt < 27'd49_999_999)
            settle_cnt <= settle_cnt + 27'd1;
    end
    wire by_done = (settle_cnt >= 27'd49_999_999);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mode2_d <= 1'b0;
        else        mode2_d <= (mode_sel == 2'd2);
    end

    // ==================== 航向闭环：直接用 IMU 欧拉角 ====================
    // yaw_ctrl 来自 0x26 帧（IMU 内部融合好的偏航角，相对上电零点，单位 0.1°）
    localparam signed [31:0] TURN_DBAND = 32'sd15;   // 到位判定：±1.5°
    localparam        [4:0]  TURN_SHIFT = 5'd3;      // 每 0.8° 误差加 1 档
    localparam signed [31:0] TURN_MAX   = 32'sd5;    // 最大 5 档（y 轴 ±5 => 10% 转速）
    localparam signed [31:0] TURN_MIN   = 32'sd1;    // 最小 1 档（2%）：比例档位落到 0 时兜底

    wire [15:0] yaw_abs = yaw_ctrl[15] ? -yaw_ctrl : yaw_ctrl;   // 调试行显示绝对值

    wire signed [31:0] yaw_err = $signed(yaw_ctrl) - yaw_target;
    wire signed [31:0] err_abs = yaw_err[31] ? -yaw_err : yaw_err;   // 误差绝对值（0.1°）

    // 线性比例 + 0.1% 直通：1° 误差 = 1% 转向速度，限幅 ±10%，0.5° 死区
    wire signed [31:0] rot_lin = (yaw_err > 32'sd900) ? 32'sd100 :
                                 (yaw_err < -32'sd900) ? -32'sd100 : (yaw_err / 32'sd9);
    wire signed [31:0] rot_v10 = (err_abs <= 32'sd5) ? 32'sd0 : rot_lin;
    wire               rot_en  = (seq == SEQ_TURN);

    // 兼容九宫格 y 轴通路（rot_en=1 时由直通覆盖，此处仅占位）
    wire [31:0] turn_move = 32'd505050;

    // 只在 SEQ_TURN 里累计"到位"计时。若不加这个门控，
    // 待机时 yaw_err≈0 就把 turn_settle 攒满，SEQ_TURN 一进去立刻判完成、根本不转。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            turn_settle <= 27'd0;
        else if (seq != SEQ_TURN)
            turn_settle <= 27'd0;
        else if (err_abs <= TURN_DBAND) begin
            if (turn_settle < 27'd14_999_999) turn_settle <= turn_settle + 27'd1;
        end else
            turn_settle <= 27'd0;
    end
    wire turn_done = (turn_settle >= 27'd14_999_999);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq      <= SEQ_IDLE;
            seq_cnt  <= 30'd0;
            by_start <= 32'sd0;
        end else if (stop_fast || stop_press) begin
            seq     <= SEQ_IDLE;
            seq_cnt <= 30'd0;
        end else begin
            case (seq)
            SEQ_IDLE: begin
                if (mode2_start) begin
                    seq      <= SEQ_RCL0;
                    seq_cnt  <= 30'd0;
                end
            end
            SEQ_RCL0: begin
                // 等 IMU 对零完成，再保证至少 0.5s 静止（原来靠陀螺零偏标定顺带等 1.3s）
                if ((cal_done && (seq_cnt >= 30'd24_999_999)) || (seq_cnt >= 30'd149_999_999)) begin
                    seq      <= SEQ_FWD;
                    by_start <= pos_avg;
                    seq_cnt  <= 30'd0;
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            SEQ_FWD: begin
                if (by_done || (seq_cnt >= 30'd499_999_999)) begin
                    seq     <= SEQ_STOP;
                    seq_cnt <= 30'd0;
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            SEQ_STOP: begin
                if (seq_cnt >= 30'd49_999_999) begin
                    seq     <= SEQ_RCL1;
                    seq_cnt <= 30'd0;
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            SEQ_RCL1: begin
                if (cal_done || (seq_cnt >= 30'd149_999_999)) begin
                    seq        <= SEQ_TURN;
                    seq_cnt    <= 30'd0;
                    yaw_target <= $signed(yaw_ctrl) + 32'sd900;   // 左转 90 度（IMU 角度 +90.0°）
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            SEQ_TURN: begin
                if (turn_done || (seq_cnt >= 30'd249_999_999)) begin
                    seq     <= SEQ_DONE;
                    seq_cnt <= 30'd0;
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            SEQ_DONE: begin
                if (seq_cnt >= 30'd24_999_999) begin
                    seq     <= SEQ_IDLE;
                    seq_cnt <= 30'd0;
                end else
                    seq_cnt <= seq_cnt + 30'd1;
            end
            default: seq <= SEQ_IDLE;
            endcase
        end
    end

    assign pos_mode = (seq == SEQ_FWD);
    assign pid_clr  = (seq == SEQ_STOP) || (seq == SEQ_DONE) || (seq == SEQ_RCL0) || (seq == SEQ_RCL1);

    // 速度：3.0V 以下 0，5V 以上 10，中间线性（(V-3.0)*10/2.0）
    // 2.7V 停止迟滞：低于 2.7V 锁定停止，高于 3.1V 才允许再走
    reg av_block = 1'b0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) av_block <= 1'b0;
        else if (mode_sel != 2'd1) av_block <= 1'b0;
        else if (laser_volt_x100 < 16'd270) av_block <= 1'b1;
        else if (laser_volt_x100 > 16'd310) av_block <= 1'b0;
    end

    wire [15:0] av_num = (laser_volt_x100 > 16'd270) ? (laser_volt_x100 - 16'd270) : 16'd0;
    wire [31:0] av_prod = av_num * 16'd1139;          // 4/230 ~= 1139/65536
    // 速度档位 0/2/4/6/8/10%：<=2.7V 停，>=5V 10%，中间按电压线性
    wire [3:0]  av_idx0 = (laser_volt_x100 >= 16'd500) ? 4'd5 :
                          (laser_volt_x100 <= 16'd270) ? 4'd0 :
                          (4'd1 + av_prod[17:16]);
    wire [3:0]  av_idx  = (av_idx0 > 4'd5) ? 4'd5 : av_idx0;   // 硬限幅 10%
    wire [31:0] av_x    = 32'd50 + {28'd0, av_idx};
    wire [31:0] avoid_move = av_x * 32'd10000 + 32'd5050;

    // ==================== 避障 / 追踪 模式按钮（长亮锁定，S 清除） ====================
    // 九宫格上方两个按钮（原始坐标估算范围，等实测后修正）
    // 实测四角：避障 x3391~3615 y670~1107；追踪（原绕障）x3375~3631 y249~660
    wire mode_av_hit = tp_pressed &&
                       (tp_x >= 12'd3385) && (tp_x <= 12'd3625) &&
                       (tp_y >= 12'd665)  && (tp_y <= 12'd1115);
    wire mode_by_hit = tp_pressed &&
                       (tp_x >= 12'd3368) && (tp_x <= 12'd3640) &&
                       (tp_y >= 12'd240)  && (tp_y <= 12'd664);
    wire [1:0] mode_hit = mode_av_hit ? 2'd1 : (mode_by_hit ? 2'd2 : 2'd0);

    reg [4:0] mode_cnt  = 5'd0;
    reg [1:0] mode_cand = 2'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_cnt  <= 5'd0;
            mode_cand <= 2'd0;
        end else if (tp_valid) begin
            if (mode_hit == 2'd0) begin
                mode_cnt  <= 5'd0;
                mode_cand <= 2'd0;
            end else if (mode_hit == mode_cand) begin
                if (mode_cnt < 5'd25)
                    mode_cnt <= mode_cnt + 5'd1;
            end else begin
                mode_cand <= mode_hit;
                mode_cnt  <= 5'd0;
            end
        end
    end

    wire mode_valid = (mode_cnt >= 5'd25);



    // 光标屏幕坐标：用九宫格实测四角标定
    //   tp_y 1072 -> x608，tp_y 277 -> x796
    //   tp_x 3283 -> y60， tp_x 1373 -> y324
    // 光标屏幕坐标：用九宫格实测四角标定（绝对值+符号，避免有符号乘法）
    wire [12:0] dxp = (tp_y <= 12'd1072) ? (12'd1072 - tp_y) : (tp_y - 12'd1072);
    wire        dxn = (tp_y > 12'd1072);
    wire [24:0] dxm = dxp * 13'd242;
    wire [13:0] dxo = dxm[23:10];
    wire [13:0] cx_pos = 14'd608 + dxo;
    wire [13:0] cx_neg = (14'd608 > dxo) ? (14'd608 - dxo) : 14'd0;
    wire [11:0] cur_x = dxn ? cx_neg[11:0] : ((cx_pos > 14'd799) ? 12'd799 : cx_pos[11:0]);

    wire [12:0] dyp = (tp_x <= 12'd3283) ? (12'd3283 - tp_x) : (tp_x - 12'd3283);
    wire        dyn = (tp_x > 12'd3283);
    wire [24:0] dym = dyp * 13'd142;
    wire [13:0] dyo = dym[23:10];
    wire [13:0] cy_pos = 14'd60 + dyo;
    wire [13:0] cy_neg = (14'd60 > dyo) ? (14'd60 - dyo) : 14'd0;
    wire [11:0] cur_y = dyn ? cy_neg[11:0] : ((cy_pos > 14'd479) ? 12'd479 : cy_pos[11:0]);

    // 方向键：按下走 3 秒自动停；编码按 move.v 的新 y 方向定义
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            move_data <= 32'd505050;
        else if (mode_sel == 2'd1) begin
            // 避障：静止 1 秒后按电压调速前进
            if ((avoid_state == 2'd2) && !av_block) begin
                // 纯字面量查表，避免动态乘加综合出偏差
                case (av_idx)
                4'd1: move_data <= 32'd515050; // 2%
                4'd2: move_data <= 32'd525050; // 4%
                4'd3: move_data <= 32'd535050; // 6%
                4'd4: move_data <= 32'd545050; // 8%
                4'd5: move_data <= 32'd555050; // 10%
                default: move_data <= 32'd505050; // 0%
                endcase
            end else
                move_data <= 32'd505050;
        end else if (mode_sel == 2'd2) begin
            // 追踪：跟随 RK3588 的识别框（左右对中 + 分档前进），速度全部在 10% 以内
            move_data <= trk_move;
        end else if (dir_run) begin
            case (dir_sel)
            4'd0: move_data <= 32'd534850; // 左前
            4'd1: move_data <= 32'd555050; // 前
            4'd2: move_data <= 32'd535250; // 右前
            4'd3: move_data <= 32'd504550; // 左
            4'd4: move_data <= 32'd505050; // 停止
            4'd5: move_data <= 32'd505550; // 右
            4'd6: move_data <= 32'd475250; // 左后
            4'd7: move_data <= 32'd455050; // 后
            default: move_data <= 32'd474850; // 右后
            endcase
        end else begin
            move_data <= 32'd505050;   // 3 秒到 / 停止 / 模式切换
        end
    end

    // ==================== 以太网接收：YOLO 目标框回传 ====================
    wire        gmii_rx_clk;
    wire [7:0]  gmii_rxd;
    wire        gmii_rxdv;
    wire        gmii_rxer;


    wire rgmii_rxc_buf;
    wire rx_pll_lock;
    rx_pll u_rx_pll (
        .lock    (rx_pll_lock),
        .clkout0 (rgmii_rxc_buf),
        .clkin   (rgmii_rxc),
        .reset   (~rst_n)
    );

    rgmii_to_gmii u_rgmii_rx (
        .reset        (~rst_n),
        .rgmii_rx_clk (rgmii_rxc_buf),
        .rgmii_rxd    (rgmii_rxd),
        .rgmii_rxdv   (rgmii_rxdv),
        .gmii_rx_clk  (gmii_rx_clk),
        .gmii_rxd     (gmii_rxd),
        .gmii_rxdv    (gmii_rxdv),
        .gmii_rxer    (gmii_rxer)
    );

    wire        rx_payload_valid;
    wire [7:0]  rx_payload_data;
    wire        rx_done;
    wire        rx_err;
    wire [15:0] rx_len;

    // 接收过滤：本机 MAC + 广播 MAC 都放行（eth_udp_rx_gmii 内部已或上
    // FF:FF:FF:FF:FF:FF），目的 IP 允许 192.168.1.255 / 192.168.1.10 /
    // 255.255.255.255 三个，UDP 目的端口 6103。
    // 注意：local_mac 必须写本机真实 MAC，否则 RK3588 用单播（静态 ARP）
    //       回传时会被 MAC 过滤丢掉；原来写成全 F 就只能收广播。
    eth_udp_rx_gmii u_udp_rx (
        .reset_p          (~rst_n),
        .local_mac        (48'h00_0A_35_01_FE_C1),
        .local_ip         (32'hC0_A8_01_FF),      // 192.168.1.255 子网广播
        .local_ip_alt     (32'hC0_A8_01_0A),      // 192.168.1.10  本机单播
        .local_port       (16'd6103),
        .clk125m_o        (),
        .exter_mac        (),
        .exter_ip         (),
        .exter_port       (),
        .rx_data_length   (rx_len),
        .data_overflow_i  (1'b0),
        .payload_valid_o  (rx_payload_valid),
        .payload_dat_o    (rx_payload_data),
        .one_pkt_done     (rx_done),
        .pkt_error        (rx_err),
        .debug_crc_check  (),
        .gmii_rx_clk      (gmii_rx_clk),
        .gmii_rxdv        (gmii_rxdv),
        .gmii_rxd         (gmii_rxd)
    );

    wire [4:0]   box_cnt;
    wire         box_toggle;
    wire [255:0] box_x1, box_y1, box_x2, box_y2;
    wire [127:0] box_score, box_cls;

    reg [15:0] rx_pkt_cnt = 16'd0;
    reg [15:0] rx_byte_cnt = 16'd0;
    reg [15:0] crc_err_cnt = 16'd0;
    reg [15:0] rx_last_len = 16'd0;
    always @(posedge gmii_rx_clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte_cnt <= 16'd0;
            crc_err_cnt <= 16'd0;
            rx_last_len <= 16'd0;
        end else begin
            if (gmii_rxdv) rx_byte_cnt <= rx_byte_cnt + 16'd1;
            if (rx_err) crc_err_cnt <= crc_err_cnt + 16'd1;
            if (rx_done) rx_last_len <= rx_len;
        end
    end

    wire [15:0] st_frames, st_len, st_etype, st_dip_hi, st_dip_lo, st_sport, st_dport;
    eth_stat u_eth_stat (
        .clk         (gmii_rx_clk),
        .rst_n       (rst_n),
        .dv          (gmii_rxdv),
        .d           (gmii_rxd),
        .frames      (st_frames),
        .last_len    (st_len),
        .last_etype  (st_etype),
        .last_dip_hi (st_dip_hi),
        .last_dip_lo (st_dip_lo),
        .last_sport  (st_sport),
        .last_dport  (st_dport)
    );

    uart_dbg u_uart_dbg (
        .clk   (clk),
        .rst_n (rst_n),
        .v0    (st_frames),      // 以太网 GMII 总帧数
        .v1    (rx_pkt_cnt),     // 以太网通过过滤的帧数（= 屏上 E）
        .v2    (imu_byte_cnt),   // IMU 串口字节数
        .v3    (imu_frame_cnt),  // IMU 校验通过的帧数
        .v4    (imu_euler_cnt),  // IMU 0x26 欧拉角帧数
        .v5    (imu_raw_cnt),    // IMU 0x04 原始数据帧数
        .v6    (imu_cs_err),     // IMU 校验失败次数
        .v7    ({imu_last_func, imu_last_len}),
        .v8    (imu_roll_deg),   // roll  0.1°
        .v9    (imu_pitch_deg),  // pitch 0.1°
        .v10   (imu_yaw_deg),    // yaw   0.1°
        .v11   ({cal_done, 8'd0, cal_cnt}),   // bit15 = cal_done, bit6:0 = cal_cnt
        .tx    (uart_tx_pin)
    );
    always @(posedge gmii_rx_clk or negedge rst_n) begin
        if (!rst_n) rx_pkt_cnt <= 16'd0;
        else if (rx_done) rx_pkt_cnt <= rx_pkt_cnt + 16'd1;
    end

    box_rx_parse u_box_rx (
        .clk        (gmii_rx_clk),
        .rst_n      (rst_n),
        .data       (rx_payload_data),
        .valid      (rx_payload_valid),
        .box_count  (box_cnt),
        .box_toggle (box_toggle),
        .box_x1     (box_x1),
        .box_y1     (box_y1),
        .box_x2     (box_x2),
        .box_y2     (box_y2),
        .box_score  (box_score),
        .box_cls    (box_cls)
    );

    // ==================== 追踪控制（跟随 RK3588 识别框） ====================
    box_sync u_box_sync (
        .clk        (clk),
        .rst_n      (rst_n),
        .src_toggle (box_toggle),
        .src_cnt    (box_cnt),
        .src_x1     (box_x1[15:0]),
        .src_y1     (box_y1[15:0]),
        .src_x2     (box_x2[15:0]),
        .src_y2     (box_y2[15:0]),
        .src_score  (box_score[7:0]),
        .cnt        (trk_cnt),
        .x1         (trk_x1),
        .y1         (trk_y1),
        .x2         (trk_x2),
        .y2         (trk_y2),
        .score      (trk_score),
        .ce         (trk_ce)
    );

    // 取第 0 个框：RK3588 按置信度从高到低发送，第 0 个就是最优目标
    wire [15:0] trk_cx = (trk_x1 + trk_x2) >> 1;                         // 框中心 X（画面宽 800）
    wire [15:0] trk_h  = (trk_y2 > trk_y1) ? (trk_y2 - trk_y1) : 16'd0;  // 框高 ≈ 距离（越大越近）
    wire [15:0] trk_w  = (trk_x2 > trk_x1) ? (trk_x2 - trk_x1) : 16'd0;

    wire signed [17:0] trk_ex    = $signed({2'b00, trk_cx}) - 18'sd400;  // >0：目标在画面右侧
    wire               trk_right = (trk_ex >  18'sd40);                  // 死区 ±40 像素
    wire               trk_left  = (trk_ex < -18'sd40);
    wire               trk_ok    = (trk_cnt != 5'd0) && (trk_score >= 8'd50) &&
                                   (trk_w >= 16'd8) && (trk_h >= 16'd8);

    // move 编码：6 位十进制 XX_YY_ZZ，每轴 50 = 静止，映射 (v-50)*2 %
    //   XX = 前进后退，YY = 左右（>50 右转），ZZ = 原地旋转（不用）
    // 各档都把 |XX偏置| + |YY偏置| 控制在 10 以内（即每轮 ≤10%），
    // 避免 move.v 的 10% 硬限幅把 x/y 合成量削掉造成转向失真
    assign trk_move =
        (!trk_ok)          ? 32'd505050 :                                // 没目标：停
        (trk_h >= 16'd180) ? (trk_right ? 32'd505550 :                   // 近：只转向，不再前冲
                              trk_left  ? 32'd504550 : 32'd505050) :
        (trk_h >= 16'd90)  ? (trk_right ? 32'd525250 :                   // 中：前 4% + 转 4%
                              trk_left  ? 32'd524850 : 32'd525050) :
                             (trk_right ? 32'd535250 :                   // 远：前 6% + 转 4%
                              trk_left  ? 32'd534850 : 32'd555050);      // 远且居中：前 10%


    // ==================== OV5640 + DDR3 帧缓存 + TFT ====================
    ov5640_ddr3_tft u_cam_tft(
        .clk50m        (clk),
        .reset_n       (rst_n),

        // 校准：显示电机0编码器累计计数（4 位十进制）
        .imu_roll      (roll_disp),
        .imu_pitch     (pitch_disp),
        .imu_yaw       (yaw_disp),
        .imu_zeroing   (imu_zeroing),
        .volt_x100     (laser_volt_x100),      // 恢复显示激光测距电压（之前调试以太网时被改成框数）
        .enc_cnt       ({11'd0, box_cnt}),     // E 行改成显示 RK3588 识别到的目标框数
        .tp_pressed    (dir_run),
        .tp_cell       (tp_cell),
        .mode_sel      (mode_sel),
        .cur_pressed   (tp_pressed),
        .cur_x         (cur_x),
        .cur_y         (cur_y),

        .camera_sclk   (camera_sclk),
        .camera_sdat   (camera_sdat),
        .camera_vsync  (camera_vsync),
        .camera_href   (camera_href),
        .camera_pclk   (camera_pclk),
        .camera_xclk   (camera_xclk),
        .camera_data   (camera_data),
        .camera_rst_n  (camera_rst_n),
        .camera_pwdn   (camera_pwdn),

        .box_cnt       (box_cnt),
        .box_x1        (box_x1),
        .box_y1        (box_y1),
        .box_x2        (box_x2),
        .box_y2        (box_y2),
        .box_score     (box_score),
        .box_cls       (box_cls),

        .cam_pclk      (cam_pclk),
        .cam_pixel     (cam_pixel),
        .cam_valid     (cam_valid),
        .cam_hs        (cam_hs),
        .cam_vs        (cam_vs),
        .cam_x         (cam_x),
        .cam_y         (cam_y),

        .TFT_rgb       (TFT_rgb),
        .TFT_hs        (TFT_hs),
        .TFT_vs        (TFT_vs),
        .TFT_clk       (TFT_clk),
        .TFT_de        (TFT_de),
        .TFT_pwm       (TFT_pwm),

        .O_ddr_addr    (O_ddr_addr),
        .O_ddr_ba      (O_ddr_ba),
        .O_ddr_cs_n    (O_ddr_cs_n),
        .O_ddr_ras_n   (O_ddr_ras_n),
        .O_ddr_cas_n   (O_ddr_cas_n),
        .O_ddr_we_n    (O_ddr_we_n),
        .O_ddr_clk     (O_ddr_clk),
        .O_ddr_clk_n   (O_ddr_clk_n),
        .O_ddr_cke     (O_ddr_cke),
        .O_ddr_odt     (O_ddr_odt),
        .O_ddr_reset_n (O_ddr_reset_n),
        .O_ddr_dqm     (O_ddr_dqm),
        .IO_ddr_dq     (IO_ddr_dq),
        .IO_ddr_dqs    (IO_ddr_dqs),
        .IO_ddr_dqs_n  (IO_ddr_dqs_n)
    );

    // ==================== 千兆以太网图像发送（RGMII + UDP 广播） ====================
    // 每行一包：行号(2B) + 800 像素(1600B)，共 1602B 负载，发往 192.168.1.255:6102
    localparam [47:0] ETH_SRC_MAC = 48'h00_0a_35_01_fe_c1;
    localparam [31:0] ETH_SRC_IP  = 32'hC0_A8_01_0A;   // 192.168.1.10
    localparam [31:0] ETH_DST_IP  = 32'hC0_A8_01_FF;   // 192.168.1.255（广播，免 ARP）
    localparam [15:0] ETH_DST_PORT = 16'd6102;
    localparam [15:0] ETH_SRC_PORT = 16'd5000;

    wire        eth_clk125m;
    wire        eth_pll_lock;

    eth_pll u_eth_pll(
        .lock    (eth_pll_lock),
        .clkout0 (),
        .clkout1 (eth_clk125m),
        .clkout2 (),
        .clkin   (clk),
        .reset   (~rst_n)
    );

    wire        phy_init_done;
    wire        cfg_mdc;
    wire        dbg_mdc;
    wire        dbg_mdio;
    wire [15:0] phyid_hi, phyid_lo, phy_bmcr, phy_bmsr;

    assign eth_mdc = phy_init_done ? dbg_mdc : cfg_mdc;

    phy_dbg u_phy_dbg (
        .clk       (clk),
        .rst_n     (rst_n),
        .init_done (phy_init_done),
        .mdc       (dbg_mdc),
        .mdio      (eth_mdio),
        .phyid_hi  (phyid_hi),
        .phyid_lo  (phyid_lo),
        .bmcr      (phy_bmcr),
        .bmsr      (phy_bmsr)
    );
    wire [15:0] phy_rd_data;

    phy_reg_config u_phy_cfg(
        .Clk           (clk),
        .Rst_n         (rst_n),
        .Phy_rst_n     (eth_rst_n),
        .Rd_Data       (phy_rd_data),
        .Phy_init_done (phy_init_done),
        .mdc           (cfg_mdc),
        .mdio          (eth_mdio)
    );

    // 摄像头行数据打包：行首插入 2 字节行号
    reg        cam_hs_d1;
    reg [15:0] cam_pix_d1;
    reg        cam_vld_d1;
    reg [15:0] eth_pix;
    reg        eth_pix_vld;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            cam_hs_d1  <= 1'b0;
            cam_pix_d1 <= 16'd0;
            cam_vld_d1 <= 1'b0;
        end else begin
            cam_hs_d1  <= cam_hs;
            cam_pix_d1 <= cam_pixel;
            cam_vld_d1 <= cam_valid;
        end
    end

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            eth_pix <= 16'd0;
        else if (~cam_hs_d1 && cam_hs)
            eth_pix <= cam_y - 12'd1;      // 行号
        else if (cam_vld_d1)
            eth_pix <= cam_pix_d1;
    end

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n)
            eth_pix_vld <= 1'b0;
        else if (~cam_hs_d1 && cam_hs)
            eth_pix_vld <= 1'b1;
        else if (cam_vld_d1)
            eth_pix_vld <= 1'b1;
        else
            eth_pix_vld <= 1'b0;
    end

    wire [7:0] eth_fifo_dout;
    wire       eth_fifo_rd;
    wire       eth_tx_en;
    wire       eth_tx_done;

    eth_tx_ctrl #(
        .PAYLOAD_DATA_BYTE (2),
        .PAYLOAD_LENGTH    (801)          // 800 像素 + 行号
    ) u_eth_tx_ctrl (
        .reset_p           (~rst_n),
        .clk               (cam_pclk),
        .data_i            ({eth_pix[7:0], eth_pix[15:8]}),
        .data_valid_i      (eth_pix_vld),
        .eth_txfifo_rd_clk (eth_clk125m),
        .tx_en_pulse       (eth_tx_en),
        .tx_done           (eth_tx_done),
        .eth_txfifo_rden   (eth_fifo_rd),
        .eth_txfifo_dout   (eth_fifo_dout)
    );

    wire [7:0] gmii_txd;
    wire       gmii_tx_clk;
    wire       gmii_txen;

    eth_udp_tx_gmii u_eth_udp (
        .clk125m      (eth_clk125m),
        .reset_p      (~rst_n),
        .tx_en_pulse  (eth_tx_en),
        .tx_done      (eth_tx_done),
        .dst_mac      (48'hFF_FF_FF_FF_FF_FF),
        .src_mac      (ETH_SRC_MAC),
        .dst_ip       (ETH_DST_IP),
        .src_ip       (ETH_SRC_IP),
        .dst_port     (ETH_DST_PORT),
        .src_port     (ETH_SRC_PORT),
        .data_length  (16'd1602),
        .payload_req_o(eth_fifo_rd),
        .payload_dat_i(eth_fifo_dout),
        .gmii_tx_clk  (gmii_tx_clk),
        .gmii_txen    (gmii_txen),
        .gmii_txd     (gmii_txd)
    );

    gmii_to_rgmii u_gmii_to_rgmii (
        .reset_n       (rst_n),
        .gmii_tx_clk   (gmii_tx_clk),
        .gmii_txd      (gmii_txd),
        .gmii_txen     (gmii_txen),
        .gmii_txer     (1'b0),
        .rgmii_tx_clk  (rgmii_gtxc),
        .rgmii_txd     (rgmii_txd),
        .rgmii_txen    (rgmii_txen)
    );

endmodule
