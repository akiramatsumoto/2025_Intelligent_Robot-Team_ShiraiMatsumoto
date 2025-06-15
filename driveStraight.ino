#include <Arduino.h>

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

void setup() {
  Serial.begin(115200);

  // エンコーダ
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);

  // モータドライバ
  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);
}

void loop() {
  driveStraight(255);
  delay(1000);
  stopAll();
  delay(1000);
}

// 前後移動: mm/s (正:前進, 負:後退)
void driveStraight(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_RIGHT_FORWORD, pwm);
    analogWrite(WHEEL_MD_RIGHT_BACK, 0);
    analogWrite(WHEEL_MD_LEFT_FORWORD, pwm);
    analogWrite(WHEEL_MD_LEFT_BACK, 0);
  } else {
    analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
    analogWrite(WHEEL_MD_RIGHT_BACK, pwm);
    analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
    analogWrite(WHEEL_MD_LEFT_BACK, pwm);
  }
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
  analogWrite(SUCTION_MD_A, 0);
  analogWrite(SUCTION_MD_B, 0);
}
