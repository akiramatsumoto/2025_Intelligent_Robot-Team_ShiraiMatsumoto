/* メインプログラム: FSM によるロボット動作制御 (Arduino Mega2560 Rev3 ピン接続) */

#include <Arduino.h>
#include <Wire.h> // I2C 用

/* ピン割り当て */
// ライントレースセンサ (ch1～ch8)
#define LINE_CH1_PIN A0
#define LINE_CH2_PIN A1
#define LINE_CH3_PIN A2
#define LINE_CH4_PIN A3
#define LINE_CH5_PIN A4
#define LINE_CH6_PIN A5
#define LINE_CH7_PIN A6
#define LINE_CH8_PIN A7

// エンコーダ1
#define ENC1_A_PIN 19  // A 相
#define ENC1_B_PIN 32  // B 相
// エンコーダ2
#define ENC2_A_PIN 18  // A 相
#define ENC2_B_PIN 38  // B 相

// モータドライバ1
#define M1_IN_A 7  // PWM
#define M1_IN_B 6  // PWM
// モータドライバ2
#define M2_IN_A 5  // PWM
#define M2_IN_B 4  // PWM
// モータドライバ3
#define M3_IN_A 3  // PWM
#define M3_IN_B 2  // PWM

// サーボ
#define SERVO_PIN 9  // PWM

// 測距センサ (I2C)
#define DIST_SDA_PIN 20
#define DIST_SCL_PIN 21

/* FSM 状態定義 */
#define STATE_WAIT           1  // 待機
#define STATE_FORWARD        2  // 直進
#define STATE_TO_BALL_AREA   3  // ライントレースしてボールエリアへ
#define STATE_DETECT_COLLECT 4  // ボール検出・回収
#define STATE_TO_RED_GOAL    5  // 赤色ゴールへライントレース
#define STATE_TO_YELLOW_GOAL 6  // 黄色ゴールへライントレース
#define STATE_TO_BLUE_GOAL   7  // 青色ゴールへライントレース
#define STATE_DROP_RED       8  // 赤色ゴールに投入
#define STATE_DROP_YELLOW    9  // 黄色ゴールに投入
#define STATE_DROP_BLUE     10  // 青色ゴールに投入

/* グローバル変数 */
int state = STATE_WAIT; // 現在の状態
bool psd = false;       // 測距センサ検出フラグ
int color = 0;          // 読み取ったボール色: 0=なし,1=赤,2=黄,3=青
int linePos = 0;        // ライン位置番号: 1〜8

void setup() {
  Serial.begin(115200);
  Wire.begin();
  // ラインセンサ
  pinMode(LINE_CH1_PIN, INPUT);
  pinMode(LINE_CH2_PIN, INPUT);
  pinMode(LINE_CH3_PIN, INPUT);
  pinMode(LINE_CH4_PIN, INPUT);
  pinMode(LINE_CH5_PIN, INPUT);
  pinMode(LINE_CH6_PIN, INPUT);
  pinMode(LINE_CH7_PIN, INPUT);
  pinMode(LINE_CH8_PIN, INPUT);
  // エンコーダ
  pinMode(ENC1_A_PIN, INPUT);
  pinMode(ENC1_B_PIN, INPUT);
  pinMode(ENC2_A_PIN, INPUT);
  pinMode(ENC2_B_PIN, INPUT);
  // モータドライバ
  pinMode(M1_IN_A, OUTPUT);
  pinMode(M1_IN_B, OUTPUT);
  pinMode(M2_IN_A, OUTPUT);
  pinMode(M2_IN_B, OUTPUT);
  pinMode(M3_IN_A, OUTPUT);
  pinMode(M3_IN_B, OUTPUT);
  // サーボ
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  // 測距センサ読み取り (I2C)
  psd = readDistanceSensor();
  // カラーセンサは別途実装
  color = readColor();
  // ライン位置を推定
  linePos = readLinePosition();

  switch (state) {
    case STATE_WAIT:
      stopMotors();
      if (!psd && color == 0 && linePos == 0) {
        state = STATE_FORWARD;
      }
      break;

    // 以下、前回実装通り
    // ...

    default:
      state = STATE_WAIT;
      break;
  }
  delay(50);
}

/* --- ヘルパー関数 --- */

bool readDistanceSensor() {
  // I2C 接続の測距センサ読み取り処理を実装
  return false;
}

int readColor() {
  // カラーセンサ読み取り処理を実装
  return 0;
}

int readLinePosition() {
  // ch1～ch8 を利用し現在ライン位置を推定
  // 例: center センサのみ反応時は 4 とするなど
  return 0;
}

void moveForward() {}
void lineTrace() {}
void stopMotors() {
  // 全モーターを停止
  digitalWrite(M1_IN_A, LOW);
  digitalWrite(M1_IN_B, LOW);
  digitalWrite(M2_IN_A, LOW);
  digitalWrite(M2_IN_B, LOW);
  digitalWrite(M3_IN_A, LOW);
  digitalWrite(M3_IN_B, LOW);
}
void collectBall() {}
void dropBall() {}

