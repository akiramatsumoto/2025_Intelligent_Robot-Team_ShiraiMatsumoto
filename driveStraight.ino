// Encoderライブラリ(PCINT)を使った左車輪P制御サンプル
#include <Arduino.h>
#include <Encoder.h>  // PCINTを用いたエンコーダ処理

// エンコーダピン定義
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相
#define ENC_RIGHT_A    18  // 右車輪 A 相（開放）
#define ENC_RIGHT_B    38  // 右車輪 B 相（開放）

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_LEFT_FORWARD  7  // 左車輪前進 (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪後退 (PWM)
#define WHEEL_MD_RIGHT_FORWARD 4  // 右車輪前進 (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪後退 (PWM)

// Encoderオブジェクト生成 (PCINT対応)
Encoder encLeft(ENC_LEFT_A, ENC_LEFT_B);

// 制御目標・ゲイン
const long targetPulses = 3200;  // 左車輪：1秒あたりの目標パルス
const float Kp = 0.005;           // P制御ゲイン（要調整）

// 制御用変数
long prevCountL = 0;
int pwmLeft = 0;                 // 左PWM値
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);

  // モータドライバ出力設定
  pinMode(WHEEL_MD_LEFT_FORWARD,  OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK,     OUTPUT);
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK,    OUTPUT);

  // エンコーダ初期リセット
  encLeft.write(0);
  prevCountL = encLeft.read();
  prevTime = millis();

  // 初期PWM値（右車輪は固定）
  pwmLeft = 55;  // 適当なスタート値
}

void loop() {
  unsigned long now = millis();
  // 常に左車輪を駆動
  driveLeft(pwmLeft);

  // 1秒周期で制御
  if (now - prevTime >= 1000) {
    long currL = encLeft.read();
    // エンコーダ取り付け向き反転対応
    long deltaL = prevCountL - currL;  // 逆方向なので符号反転
    long error = targetPulses - deltaL;
    int adjust = int(Kp * error);
    pwmLeft += adjust;
    pwmLeft = constrain(pwmLeft, 0, 255);

    // デバッグ出力
    Serial.print("DeltaL: "); Serial.print(deltaL);
    Serial.print("  Error: "); Serial.print(error);
    Serial.print("  PWM_L: "); Serial.println(pwmLeft);

    prevCountL = currL;
    prevTime += 1000;
  }
}

// 左車輪のみ駆動: pwm (0～255)
void driveLeft(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_LEFT_FORWARD, pwm);
    analogWrite(WHEEL_MD_LEFT_BACK,    0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_LEFT_FORWARD, 0);
    analogWrite(WHEEL_MD_LEFT_BACK,    pwm);
  }
}

// 全停止関数
void stopAll() {
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
}

