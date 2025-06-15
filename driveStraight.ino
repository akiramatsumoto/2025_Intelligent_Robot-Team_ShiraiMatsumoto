// Encoderライブラリ(PCINT)を使った累積パルスカウントサンプル
#include <Arduino.h>
#include <Encoder.h>  // PCINTを用いたエンコーダ処理

// エンコーダピン定義
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_RIGHT_FORWARD 4  // 右車輪前進 (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪後退 (PWM)
#define WHEEL_MD_LEFT_FORWARD  7  // 左車輪前進 (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪後退 (PWM)

// Encoderオブジェクト生成 (PCINT対応)
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encLeft(ENC_LEFT_A,  ENC_LEFT_B);

void setup() {
  Serial.begin(115200);

  // モータドライバ出力設定
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK,    OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWARD,  OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK,     OUTPUT);

  // 初期カウントをゼロにリセット（必要時）
  encRight.write(0);
  encLeft.write(0);
}

void loop() {
  // モータを1秒間駆動
  driveStraight(65);
  delay(1000);

  // 累積カウント値を取得（リセットしない）
  long countR = encRight.read();
  long countL = encLeft.read();

  Serial.print("[累積] Right Count: "); Serial.print(countR);
  Serial.print("  Left Count: ");  Serial.println(countL);

  delay(500);
}

// 直進: pwm (-255～255)
// 65以下ではトルクが小さすぎてホイールを回せない
void driveStraight(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_RIGHT_FORWARD, pwm);
    analogWrite(WHEEL_MD_RIGHT_BACK,    0);
    analogWrite(WHEEL_MD_LEFT_FORWARD,  pwm);
    analogWrite(WHEEL_MD_LEFT_BACK,     0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
    analogWrite(WHEEL_MD_RIGHT_BACK,    pwm);
    analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
    analogWrite(WHEEL_MD_LEFT_BACK,     pwm);
  }
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
}
