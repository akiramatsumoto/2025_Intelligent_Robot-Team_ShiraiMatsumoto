// Encoderライブラリ(PCINT)を使った1秒ごとのパルス数計測サンプル
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

long prevCountR = 0;
long prevCountL = 0;

void setup() {
  Serial.begin(115200);
  
  // モータドライバ出力設定
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK,    OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWARD,  OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK,     OUTPUT);

  // エンコーダ初期リセット
  encRight.write(0);
  encLeft.write(0);

  // 初回差分計測用
  prevCountR = encRight.read();
  prevCountL = encLeft.read();
}

void loop() {
  // 1秒間駆動
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 77);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);

  delay(1000);

  // 現在のカウント取得
  long currR = encRight.read();
  long currL = encLeft.read();

  // 1秒ごとのパルス数計算
  long deltaR = currR - prevCountR;
  long deltaL = currL - prevCountL;

  // 出力
  Serial.print("Pulses per 1s → Right: "); Serial.print(deltaR);
  Serial.print("  Left: ");               Serial.println(deltaL);

  // 次回用に値を保存
  prevCountR = currR;
  prevCountL = currL;

  // 停止または継続は不要（要件に記載なし）
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
}

