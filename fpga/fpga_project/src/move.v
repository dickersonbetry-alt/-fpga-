// ============================================================
// move.v : 四轮运动学解算 + 四路闭环电机驱动
//
// move 输入格式：6 位十进制，每两位代表一个轴，50 为 0 基准：
//   万/千位 (move/10000) = 前进后退 (x)
//   百/十位   (move/100%100) = 左右转向 (y)   —— 本车 y 轴是旋转
//   个位      (move%100)     = 原地旋转 (z)   —— 本车未使用
// 每轴取值 0~99，50 为静止，映射到 -100 ~ +100。
//
// 附加功能：
//   pos_mode + pos_delta + pos_fb : 位置闭环（走固定距离）
//   rot_en   + rot_v10            : 转向直通（0.1% 分辨率，角度环用）
//   pid_clr                       : 清 PID 积分
//
// 编码器反馈 enc0~enc3 分别对应电机 0~3 (A左前/B左后/C右前/D右后)。
// ============================================================
module move(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [31:0] move,
    // 编码器反馈 (0=A左前, 1=B左后, 2=C右前, 3=D右后)
    input  wire        enc0_a,
    input  wire        enc0_b,
    input  wire        enc1_a,
    input  wire        enc1_b,
    input  wire        enc2_a,
    input  wire        enc2_b,
    input  wire        enc3_a,
    input  wire        enc3_b,
    output wire        motor0_A,
    output wire        motor0_B,
    output wire        motor1_A,
    output wire        motor1_B,
    output wire        motor2_A,
    output wire        motor2_B,
    output wire        motor3_A,
    output wire        motor3_B,
    output wire        motor0_pwm,
    output wire        motor1_pwm,
    output wire        motor2_pwm,
    output wire        motor3_pwm,
    output wire signed [31:0] pos0,
    output wire signed [31:0] pos1,
    output wire signed [31:0] pos2,
    output wire signed [31:0] pos3,
    input  wire               pos_mode,
    input  wire signed [31:0] pos_delta,
    input  wire signed [31:0] pos_fb,
    input  wire               pid_clr,
    input  wire               rot_en,
    input  wire signed [31:0] rot_v10
);

    reg signed [31:0] x;
    reg signed [31:0] y;
    reg signed [31:0] z;
    reg signed [31:0] v [3:0];
    reg [31:0] max;
    integer n;

    always @(*) begin
        x = ($signed(move)/10000-50)*2;
        y = ($signed(move)/100%100-50)*2;
        z = ($signed(move)%100-50)*2;
        v[0] = x-y-z;
        v[1] = x+y-z;
        v[2] = x+y+z;
        v[3] = x-y+z;
        max = 0;
        for(n=0;n<4;n=n+1) begin
            if((v[n]>=0?v[n]:-v[n])>max)
                max = v[n]>=0?v[n]:-v[n];
        end
        if(max>100)
            for(n=0;n<4;n=n+1)
                v[n]=v[n]*100/$signed(max);
        // 硬限幅：任何模式每个电机最多 10%
        for(n=0;n<4;n=n+1) begin
            if(v[n] >  10) v[n] =  10;
            if(v[n] < -10) v[n] = -10;
        end
    end

    // 转向直通：与 y 轴旋转同向（左转 = rot_v10 负）
    wire signed [31:0] rot0 = -rot_v10;
    wire signed [31:0] rot1 =  rot_v10;
    wire signed [31:0] rot2 =  rot_v10;
    wire signed [31:0] rot3 = -rot_v10;

    motor motor0(
        .clk      (clk),
        .rst_n    (rst_n),
        .v        (v[0]),
        .enc_a    (enc0_a),
        .enc_b    (enc0_b),
        .A        (motor0_A),
        .B        (motor0_B),
        .pwm      (motor0_pwm),
        .enc_pos  (pos0),
        .pos_mode (pos_mode),
        .pos_delta(pos_delta),
        .pos_fb   (pos_fb),
        .pid_clr  (pid_clr),
        .rot_en   (rot_en),
        .rot_v10  (rot0)
    );
    motor motor1(
        .clk      (clk),
        .rst_n    (rst_n),
        .v        (v[1]),
        .enc_a    (enc1_a),
        .enc_b    (enc1_b),
        .A        (motor1_A),
        .B        (motor1_B),
        .pwm      (motor1_pwm),
        .enc_pos  (pos1),
        .pos_mode (pos_mode),
        .pos_delta(pos_delta),
        .pos_fb   (pos_fb),
        .pid_clr  (pid_clr),
        .rot_en   (rot_en),
        .rot_v10  (rot1)
    );
    motor motor2(
        .clk      (clk),
        .rst_n    (rst_n),
        .v        (v[2]),
        .enc_a    (enc2_a),
        .enc_b    (enc2_b),
        .A        (motor2_A),
        .B        (motor2_B),
        .pwm      (motor2_pwm),
        .enc_pos  (pos2),
        .pos_mode (pos_mode),
        .pos_delta(pos_delta),
        .pos_fb   (pos_fb),
        .pid_clr  (pid_clr),
        .rot_en   (rot_en),
        .rot_v10  (rot2)
    );
    motor motor3(
        .clk      (clk),
        .rst_n    (rst_n),
        .v        (v[3]),
        .enc_a    (enc3_a),
        .enc_b    (enc3_b),
        .A        (motor3_A),
        .B        (motor3_B),
        .pwm      (motor3_pwm),
        .enc_pos  (pos3),
        .pos_mode (pos_mode),
        .pos_delta(pos_delta),
        .pos_fb   (pos_fb),
        .pid_clr  (pid_clr),
        .rot_en   (rot_en),
        .rot_v10  (rot3)
    );
endmodule
