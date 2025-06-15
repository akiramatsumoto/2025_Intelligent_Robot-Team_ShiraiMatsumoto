#include <Arduino.h>
#include <Encoder.h>

// エンコーダ
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_RIGHT_FORWORD 4  // 右車輪入力 B (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪入力 A (PWM)
#define WHEEL_MD_LEFT_FORWORD  7  // 左車輪入力 A (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪入力 B (PWM)

#define rotateC 7050

// エンコーダライブラリ用インスタンス
Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);

void setup() {
  Serial.begin(115200);

  // モータドライバ出力ピン設定
  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);

  // エンコーダ初期化
  encoderRight.write(0);
  encoderLeft.write(0);

  // モーター回転開始
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 77);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 55);
}

void loop() {
  long posRight = encoderRight.read();
  long posLeft  = encoderLeft.read();

  Serial.print("Right Encoder: ");
  Serial.print(posRight);
  Serial.print("  Left Encoder: ");
  Serial.println(posLeft);
  // 7050
  if (posRight >= rotateC / 30 && posLeft >= rotateC / 30) {
    stopAll();
    while (1); // 停止後は無限ループで停止維持
  }

  delay(20);  // 100msごとに表示
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
}

