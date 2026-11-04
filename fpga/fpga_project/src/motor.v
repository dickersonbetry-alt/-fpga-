// ============================================================
// motor.v : 单路带正交编码器反馈的闭环直流电机驱动 (TB6612)
//
// 输入目标速度 v (-100 ~ +100，单位 %)，通过正交编码器测量实际转速，
// 用 PID 速度环调节 PWM 占空比，并输出方向信号 A/B。
//
// TB6612 接线：
//   A   -> IN1
//   B   -> IN2
//   pwm -> PWM
//
// 关键参数：
//   PWM_PERIOD     : PWM 周期(时钟数)，50MHz 下 10000 => 5kHz
//   CTRL_PERIOD    : 速度环控制周期(时钟数)，50MHz 下 5_000_000 => 10Hz (100ms)
//   FULL_SPEED_CNT : 100% 占空比(最高速)时，一个控制周期内的编码器计数增量。
//                    需按实际电机标定：
//                    FULL_SPEED_CNT = 4 * PPR * 减速比 * (最高输出转速rpm/60)
//                                      * CTRL_PERIOD / f_clk
//   KP/KI/KD       : PID 增益(定点)，实际增益 = 参数值 / 2^GAIN_SHIFT
//   MIN_DUTY       : 最小占空比(死区)，低于此值短刹车，防止低速方向高频抖动
//   SPEED_INVERT   : 1 = 反转编码器测速符号（电机一上电就全速乱跑/正反馈时置1）
// ============================================================
module motor #(
    parameter signed [31:0] KP             = 32'sd320, // 比例增益(定点, 实际 512/256 = 2.0，加快收敛)
    parameter signed [31:0] KI             = 32'sd128, // 积分增益(定点, 实际 128/256=0.5，加快负载恢复)
    parameter signed [31:0] KD             = 32'sd10,   // 微分增益(阻尼)
    parameter        [31:0] GAIN_SHIFT     = 8,          // 增益右移位数
    parameter        [31:0] PWM_PERIOD     = 32'd10_000,  // PWM 周期
    parameter        [31:0] CTRL_PERIOD    = 32'd5_000_000,  // 控制周期
    parameter signed [31:0] FULL_SPEED_CNT = 32'sd1000,  // 100% 速度对应计数 (100ms 内)
    parameter signed [31:0] SPEED_LIMIT    = 32'sd2000, // 测速限幅(0.1%单位)，防毛刺打爆积分
    parameter        [31:0] MIN_DUTY       = 32'd200,   // 最小占空比(死区)
    parameter        [31:0] SPEED_INVERT   = 32'd0      // 1 = 反转编码器测速符号
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] v,      // 目标速度 -100 ~ +100
    input  wire               pos_mode,   // 1 = 位置闭环
    input  wire signed [31:0] pos_delta,  // 位置环目标距离(counts)
    input  wire signed [31:0] pos_fb,     // 四轮平均位置（位置环反馈）
    input  wire               pid_clr,    // 清 PID 积分
    input  wire               rot_en,     // 转向直通使能
    input  wire signed [31:0] rot_v10,   // 转向速度（0.1% 单位）
    input  wire               enc_a,  // 编码器 A 相
    input  wire               enc_b,  // 编码器 B 相
    output reg                A,      // TB6612 IN1
    output reg                B,      // TB6612 IN2
    output wire               pwm,    // TB6612 PWM
    output wire signed [31:0] enc_pos // 编码器累计位置（校准用）
);

    wire signed [31:0] pos;

    reg  [31:0]        ctrl_cnt       = 32'd0;
    reg                tick           = 1'b0;
    reg  signed [31:0] pos_prev       = 32'sd0;
    reg  signed [31:0] speed_pct      = 32'sd0;
    reg  signed [31:0] spd            = 32'sd0;
    reg                first_tick = 1'b1;   // 上电默认 1（rst_n 被强制为常 1，靠初值生效）
    wire signed [31:0] pid_out;
    reg  signed [31:0] pid_out_reg    = 32'sd0;
    reg  [31:0]        duty           = 32'd0;
    reg  [31:0]        duty_raw       = 32'd0;

    // 1. 正交编码器 4 倍频解码
    quadrature u_quad (
        .clk   (clk),
        .rst_n (rst_n),
        .a     (enc_a),
        .b     (enc_b),
        .pos   (pos)
    );

    // 2. 控制周期 tick 生成
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_cnt <= 32'd0;
            tick     <= 1'b0;
        end else begin
            if (ctrl_cnt >= CTRL_PERIOD - 32'd1) begin
                ctrl_cnt <= 32'd0;
                tick     <= 1'b1;
            end else begin
                ctrl_cnt <= ctrl_cnt + 32'd1;
                tick     <= 1'b0;
            end
        end
    end

    // PID 目标：v 是整数%，setpoint 用 0.1% 单位（×10），正方向取反
    // ---------------- 位置环（与速度环级联） ----------------
    // pos_delta 是相对起点的目标距离；P 控制：1 count 误差 = 0.1% 速度，限幅 ±10%
    reg               pos_mode_d = 1'b0;
    reg signed [31:0] pos_start  = 32'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_mode_d <= 1'b0;
            pos_start  <= 32'sd0;
        end else begin
            pos_mode_d <= pos_mode;
            if (pos_mode && !pos_mode_d)
                pos_start <= pos_fb;
        end
    end

    wire signed [31:0] pos_err = pos_fb - pos_start + pos_delta;   // 前进=位置递减
    wire signed [31:0] v_pos;
    assign v_pos = (pos_err > 32'sd2)  ? ((pos_err > 32'sd100)  ? 32'sd100  : pos_err) :
                   (pos_err < -32'sd2) ? ((pos_err < -32'sd100) ? -32'sd100 : pos_err) : 32'sd0;

    wire signed [31:0] sp_target;
    assign sp_target = rot_en  ? (-rot_v10) : (pos_mode ? (-v_pos) : (-v * 32'sd10));

    // 速度换算常数：spd = delta * 1000 / FULL_SPEED_CNT
    // 改成“乘常数 + 右移 16”，避免综合出 64 位除法器（63ns 长路径，PnR 会卡很久）
    //   SPEED_SCALE = round(1000 * 2^16 / FULL_SPEED_CNT)
    localparam signed [31:0] SPEED_SCALE = (32'sd1000 * 32'sd65536) / FULL_SPEED_CNT;

    // 3. 实测速度折算为百分比（与目标 v 同量纲）
    reg signed [31:0] last_spd = 32'sd0;   // 上一拍速度，用于斜率限制
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_prev   <= 32'sd0;
            speed_pct  <= 32'sd0;
            first_tick <= 1'b1;
            last_spd   <= 32'sd0;
        end else if (tick) begin
            if (first_tick) begin
                // 第一个控制周期只对齐位置、不测速：
                // 丢弃开机瞬间（FPGA 配置期间 IO 浮空、电机残余转动）累积的计数，
                // 否则第一次测速会得到一个巨大的“速度”，把 PID 推向反向刹车。
                pos_prev   <= pos;
                speed_pct  <= 32'sd0;
                first_tick <= 1'b0;
                last_spd   <= 32'sd0;
            end else begin
                // 乘常数后右移 16 位，代替 64 位除法；再限幅防毛刺
                if (SPEED_INVERT)
                    spd = -((((pos - pos_prev) * SPEED_SCALE) >>> 16));
                else
                    spd = (((pos - pos_prev) * SPEED_SCALE) >>> 16);
                if (spd > SPEED_LIMIT)
                    spd = SPEED_LIMIT;
                else if (spd < -SPEED_LIMIT)
                    spd = -SPEED_LIMIT;
                // 斜率限制：100ms 内最多变化 300（30%），
                // 编码器偶发毛刺不会让 PID 瞬间反向
                if (spd > last_spd + 32'sd300)
                    spd = last_spd + 32'sd300;
                else if (spd < last_spd - 32'sd300)
                    spd = last_spd - 32'sd300;
                speed_pct <= spd;
                last_spd  <= spd;
                pos_prev  <= pos;
            end
        end
    end

    // 4. PID 速度闭环
    pid #(
        .KP         (KP),
        .KI         (KI),
        .KD         (KD),
        .GAIN_SHIFT (GAIN_SHIFT),
        .OUT_MAX    (32'sd1000),
        .OUT_MIN    (-32'sd1000)
    ) u_pid (
        .clk      (clk),
        .rst_n    (rst_n),
        .tick     (tick),
        .clr      (pid_clr),
        // 电机方向反转：目标速度取反。
        // 若将来只有个别电机反了，只需对调该路的 enc_a/enc_b 或对调该电机 A/B 两根线。
        // 目标 v 是整数%，setpoint 也用 0.1% 单位（×10），与 speed_pct 同量纲
        .setpoint (sp_target),
        .feedback (speed_pct),
        .out      (pid_out)
    );

    // 5. PID 输出寄存
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pid_out_reg <= 32'sd0;
        else
            pid_out_reg <= pid_out;
    end

    // 6. 占空比 + 方向（带死区，防止低速时 A/B 高频抖动）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            A    <= 1'b0;
            B    <= 1'b0;
            duty <= 32'd0;
        end else begin
            // 目标占空比幅值 = |pid_out| * 10  （pid_out 是 0.1% 单位，1000 → 100% → duty 10000）
            if (pid_out_reg >= 32'sd0)
                duty_raw = pid_out_reg * 32'd10;
            else
                duty_raw = (-pid_out_reg) * 32'd10;
            if (duty_raw > PWM_PERIOD)
                duty_raw = PWM_PERIOD;

            // 死区：占空比过小则滑行(自由停)，避免方向在零附近来回翻转。
            // 用滑行(A=B=0)而非短刹车(A=B=1)：短刹车会硬刹停电机，
            // 造成“刹停→积分累积→猛冲→超调→反向”的极限环振荡。
            if (duty_raw < MIN_DUTY) begin
                duty <= 32'd0;
                A    <= 1'b0;   // 滑行 (IN1=IN2=0)
                B    <= 1'b0;
            end else begin
                duty <= duty_raw;
                A    <= (pid_out_reg >= 32'sd0);
                B    <= (pid_out_reg <= 32'sd0);
            end
        end
    end

    // 7. PWM 输出
    pwm #(
        .T (PWM_PERIOD)
    ) u_pwm (
        .clk        (clk),
        .rst_n      (rst_n),
        .duty_count (duty),
        .pwm        (pwm)
    );

    assign enc_pos = pos;

endmodule
