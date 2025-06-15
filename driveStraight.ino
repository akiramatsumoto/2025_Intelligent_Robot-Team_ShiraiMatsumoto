// Encoderライブラリ(PCINT)を使った右車輪P制御サンプル
#include <Arduino.h>
#include <Encoder.h>  // PCINTを用いたエンコーダ処理

// エンコーダピン定義
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相（開放）
#define ENC_LEFT_B     32  // 左車輪 B 相（開放）

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_RIGHT_FORWARD 4  // 右車輪前進 (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪後退 (PWM)
#define WHEEL_MD_LEFT_FORWARD  7  // 左車輪前進 (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪後退 (PWM)

// Encoderオブジェクト生成 (PCINT対応)
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);

// 制御目標・ゲイン
const long targetPulses = 3200;  // 右車輪：1秒あたりの目標パルス
const float Kp = 0.01;            // P制御ゲイン（要調整）

// 制御用変数
long prevCountR = 0;
int pwmRight = 0;                // 右PWM値
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);

  // モータドライバ出力設定
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK,    OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWARD,  OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK,     OUTPUT);

  // エンコーダ初期リセット
  encRight.write(0);
  prevCountR = encRight.read();
  prevTime = millis();

  // 初期PWM値（左車輪は固定）
  pwmRight = 77;  // 適当なスタート値
}

void loop() {
  unsigned long now = millis();
  // 常に右車輪を駆動
  driveRight(pwmRight);

  // 1秒周期で制御
  if (now - prevTime >= 1000) {
    long currR = encRight.read();
    long deltaR = currR - prevCountR;
    long error = targetPulses - deltaR;
    int adjust = int(Kp * error);
    pwmRight += adjust;
    pwmRight = constrain(pwmRight, 0, 255);

    // デバッグ出力
    Serial.print("DeltaR: "); Serial.print(deltaR);
    Serial.print("  Error: "); Serial.print(error);
    Serial.print("  PWM_R: "); Serial.println(pwmRight);

    prevCountR = currR;
    prevTime += 1000;
  }
}

// 右車輪のみ駆動: pwm (0～255)
void driveRight(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_RIGHT_FORWARD, pwm);
    analogWrite(WHEEL_MD_RIGHT_BACK,    0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
    analogWrite(WHEEL_MD_RIGHT_BACK,    pwm);
  }
}

// 左車輪停止 or 固定駆動は要件に合わせて
// 全停止関数も併せて提供可能
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
}
