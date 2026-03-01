#include <AnglePID.h>
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>
#include <SpeedPID.h>
#include <cmath>

// ピン配置
constexpr int8_t PIN_DIR_1      = 33;
constexpr int8_t PIN_PWM_1      = 32;
constexpr int8_t PIN_ROTARY_A_1 = 13;
constexpr int8_t PIN_ROTARY_B_1 = 2;

constexpr int8_t PIN_DIR_2      = 26;
constexpr int8_t PIN_PWM_2      = 25;
constexpr int8_t PIN_ROTARY_A_2 = 21;
constexpr int8_t PIN_ROTARY_B_2 = 15;

constexpr int8_t PIN_DIR_3      = 14;
constexpr int8_t PIN_PWM_3      = 27;
constexpr int8_t PIN_ROTARY_A_3 = 23;
constexpr int8_t PIN_ROTARY_B_3 = 22;

constexpr int8_t PIN_PWM_ARM        = 19;
constexpr int8_t PIN_DIR_ARM        = 18;
constexpr int8_t PIN_LIMIT_SWITCH_1 = 17;
constexpr int8_t PIN_LIMIT_SWITCH_2 = 16;

ESP32Encoder enc1, enc2, enc3;

const int    CONTROL_CYCLE = 5000;
const double RESOLUTION    = 4096.;

// ロボットの情報
constexpr double ROBOT_RADIUS = 150.;
constexpr double WHEEL_RADIUS = 30.;

static unsigned long last;

constexpr int8_t MOTOR1_CH = 0;
constexpr int8_t MOTOR2_CH = 1;
constexpr int8_t MOTOR3_CH = 2;

// radに変換
constexpr double WHEEL_ANGLE1 = 0. * PI / 180.;
constexpr double WHEEL_ANGLE2 = 120. * PI / 180.;
constexpr double WHEEL_ANGLE3 = 240. * PI / 180.;

// 位置推定
double x     = 0.0; // mm
double y     = 0.0; // mm
double theta = 0.0; // rad

// 目標
double targetX     = 0.0;
double targetY     = 0.0;
double targetTheta = 0.0;

double Kp_pos   = 0.02;
double Kp_theta = 3.0;

long prev1 = 0, prev2 = 0, prev3 = 0;

SpeedPID pid1(5.0, 0.0, 0.05, -255, 255);
SpeedPID pid2(5.0, 0.0, 0.05, -255, 255);
SpeedPID pid3(5.0, 0.0, 0.05, -255, 255);

AnglePID thetaPID(5., 0., 0.2, -5., 5, 2 * PI, 1., -1.);

double pulseToRad(long deltaCount) {
    return (double(deltaCount) / RESOLUTION) * 2.0 * PI;
}

void setMotor(int8_t ch, int8_t dirPin, double pwm) {
    bool dir = (pwm >= 0) ? HIGH : LOW;
    digitalWrite(dirPin, dir);
    pwm = constrain(abs(pwm), 0., 255.);
    ledcWrite(ch, pwm);
}

// 逆運動学
void inverseKinematics(double vx, double vy, double omega, double& w1, double& w2, double& w3) {
    w1 = (-sin(WHEEL_ANGLE1) * vx + cos(WHEEL_ANGLE1) * vy + ROBOT_RADIUS * omega) / WHEEL_RADIUS;
    w2 = (-sin(WHEEL_ANGLE2) * vx + cos(WHEEL_ANGLE2) * vy + ROBOT_RADIUS * omega) / WHEEL_RADIUS;
    w3 = (-sin(WHEEL_ANGLE3) * vx + cos(WHEEL_ANGLE3) * vy + ROBOT_RADIUS * omega) / WHEEL_RADIUS;
}

void forwardKinematics(double w1, double w2, double w3, double& vx, double& vy, double& omega) {
    vx = (2.0 * WHEEL_RADIUS / 3.0) * (cos(WHEEL_ANGLE1) * w1 + cos(WHEEL_ANGLE2) * w2 + cos(WHEEL_ANGLE3) * w3);

    vy = (2.0 * WHEEL_RADIUS / 3.0) * (sin(WHEEL_ANGLE1) * w1 + sin(WHEEL_ANGLE2) * w2 + sin(WHEEL_ANGLE3) * w3);

    omega = (WHEEL_RADIUS / (3.0 * ROBOT_RADIUS)) * (w1 + w2 + w3);
}

bool risingEdge(bool currentState, bool& prevState) {
    bool triggered = (currentState && !prevState);
    prevState      = currentState;
    return triggered;
}

void setup() {
    Serial.begin(115200);
    PS4.begin("08:D1:F9:37:41:F2");
    ESP32Encoder::useInternalWeakPullResistors = puType::none;
    enc1.attachHalfQuad(PIN_ROTARY_A_1, PIN_ROTARY_B_1);
    enc2.attachHalfQuad(PIN_ROTARY_A_2, PIN_ROTARY_B_2);
    enc3.attachHalfQuad(PIN_ROTARY_A_3, PIN_ROTARY_B_3);

    enc1.clearCount();
    enc2.clearCount();
    enc3.clearCount();

    ledcSetup(MOTOR1_CH, 20000, 8);
    ledcSetup(MOTOR2_CH, 20000, 8);
    ledcSetup(MOTOR3_CH, 20000, 8);

    ledcAttachPin(PIN_PWM_1, MOTOR1_CH);
    ledcAttachPin(PIN_PWM_2, MOTOR2_CH);
    ledcAttachPin(PIN_PWM_3, MOTOR3_CH);

    pinMode(PIN_DIR_1, OUTPUT);
    pinMode(PIN_DIR_2, OUTPUT);
    pinMode(PIN_DIR_3, OUTPUT);

    last = micros();
}

void loop() {
    unsigned long now = micros();
    if (now - last < CONTROL_CYCLE) return;
    double dt = (now - last) * 1.e-6;
    last      = now;

    static bool prevRight = false;
    static bool prevLeft  = false;
    static bool prevUp    = false;
    static bool prevDown  = false;

    if (risingEdge(PS4.Right(), prevRight)) targetX += 100;
    if (risingEdge(PS4.Left(), prevLeft)) targetX -= 100;
    if (risingEdge(PS4.Up(), prevUp)) targetY += 100;
    if (risingEdge(PS4.Down(), prevDown)) targetY -= 100;

    long c1 = enc1.getCount();
    long c2 = enc2.getCount();
    long c3 = enc3.getCount();

    long d1 = c1 - prev1;
    long d2 = c2 - prev2;
    long d3 = c3 - prev3;

    prev1 = c1;
    prev2 = c2;
    prev3 = c3;

    double w1_meas = pulseToRad(d1) / dt;
    double w2_meas = pulseToRad(d2) / dt;
    double w3_meas = pulseToRad(d3) / dt;

    double vx, vy, omega;
    forwardKinematics(w1_meas, w2_meas, w3_meas, vx, vy, omega);

    x += vx * dt;
    y += vy * dt;
    theta += omega * dt;

    double errX = targetX - x;
    double errY = targetY - y;

    double vx_cmd    = Kp_pos * errX;
    double vy_cmd    = Kp_pos * errY;
    double omega_cmd = thetaPID.update(targetTheta, theta, dt);

    double w1_cmd, w2_cmd, w3_cmd;
    inverseKinematics(vx_cmd, vy_cmd, omega_cmd, w1_cmd, w2_cmd, w3_cmd);

    double pwm1 = pid1.update(w1_cmd, w1_meas, dt);
    double pwm2 = pid2.update(w2_cmd, w2_meas, dt);
    double pwm3 = pid3.update(w3_cmd, w3_meas, dt);

    setMotor(MOTOR1_CH, PIN_DIR_1, pwm1);
    setMotor(MOTOR2_CH, PIN_DIR_2, pwm2);
    setMotor(MOTOR3_CH, PIN_DIR_3, pwm3);

    if (fabs(errX) < 2.0 && fabs(errY) < 2.0) {
        setMotor(MOTOR1_CH, PIN_DIR_1, 0);
        setMotor(MOTOR2_CH, PIN_DIR_2, 0);
        setMotor(MOTOR3_CH, PIN_DIR_3, 0);
    }
}
