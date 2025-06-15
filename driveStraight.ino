// Encoderライブラリ(PCINT)を使った左右車輪P制御サンプル
#include <Arduino.h>
#include <Encoder.h>  // PCINTを用いたエンコーダ処理

// エンコーダピン定義
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_LEFT_FORWARD  7  // 左車輪前進 (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪後退 (PWM)
#define WHEEL_MD_RIGHT_FORWARD 4  // 右車輪前進 (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪後退 (PWM)

// Encoderオブジェクト生成 (PCINT対応)
Encoder encLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);

// 制御目標・ゲイン
const long targetPulses = 3200;      // 1秒あたりの目標パルス
const float KpRight = 0.01;          // 右車輪P制御ゲイン
const float KpLeft  = 0.005;         // 左車輪P制御ゲイン

// 制御用変数
long prevCountR = 0;
long prevCountL = 0;
int pwmRight    = 77;  // 初期PWM値（右）
int pwmLeft     = 55;  // 初期PWM値（左）
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
  encRight.write(0);

  prevCountL = encLeft.read();
  prevCountR = encRight.read();
  prevTime   = millis();
}

void loop() {
  unsigned long now = millis();

  // 駆動
  driveLeft(pwmLeft);
  driveRight(pwmRight);

  // 1秒周期でP制御
  if (now - prevTime >= 1000) {
    long currL = encLeft.read();
    long currR = encRight.read();

    long deltaL = prevCountL - currL;  // 逆向き取り付け対応
    long deltaR = currR - prevCountR;

    long errorL = targetPulses - deltaL;
    long errorR = targetPulses - deltaR;

    pwmLeft  += int(KpLeft  * errorL);
    pwmRight += int(KpRight * errorR);

    pwmLeft  = constrain(pwmLeft,  0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    // デバッグ出力
    Serial.print("DeltaL:"); Serial.print(deltaL);
    Serial.print(" ErrorL:"); Serial.print(errorL);
    Serial.print(" PWM_L:"); Serial.println(pwmLeft);
    Serial.print("DeltaR:"); Serial.print(deltaR);
    Serial.print(" ErrorR:"); Serial.print(errorR);
    Serial.print(" PWM_R:"); Serial.println(pwmRight);

    prevCountL = currL;
    prevCountR = currR;
    prevTime   += 1000;
  }
}

// 左車輪のみ駆動: pwm 0～255
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

// 右車輪のみ駆動: pwm 0～255
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

// 全停止関数
void stopAll() {
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
}

