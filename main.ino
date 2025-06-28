/* メインプログラム: FSM によるロボット動作制御 (Arduino Mega2560 Rev3 ピン接続) */

#include <Arduino.h>
#include <Wire.h> // I2C 用
#include <Servo.h>
Servo servo;

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

// エンコーダ
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_RIGHT_A 5  // 右車輪入力 A (PWM)
#define WHEEL_MD_RIGHT_B 4  // 右車輪入力 B (PWM)
#define WHEEL_MD_LEFT_A  7  // 左車輪入力 A (PWM)
#define WHEEL_MD_LEFT_B  6  // 左車輪入力 B (PWM)

// 吸引モータドライバ (AM2837)
#define SUCTION_MD_A   3  // 吸引用入力 A (PWM)
#define SUCTION_MD_B   2  // 吸引用入力 B (PWM)

// サーボ
#define SERVO_PIN      9  // PWM

// 測距センサ (I2C)
#define DIST_SDA_PIN  20
#define DIST_SCL_PIN  21

/* FSM 状態定義 */
#define STATE_WAIT           1
#define STATE_FORWARD        2
#define STATE_TO_BALL_AREA   3
#define STATE_DETECT_COLLECT 4
#define STATE_TO_RED_GOAL    5
#define STATE_TO_YELLOW_GOAL 6
#define STATE_TO_BLUE_GOAL   7
#define STATE_DROP_RED       8
#define STATE_DROP_YELLOW    9
#define STATE_DROP_BLUE     10
#define STATE_FUNCTION_TEST 11

/* グローバル変数 */
int state = STATE_WAIT; // 現在の状態
bool psd = false;       // 測距センサ検出フラグ
int color = 0;          // ボール色: 0=なし,1=赤,2=黄,3=青
int linePos = 0;        // ライン位置番号: 1～8

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
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  // モータドライバ
  pinMode(WHEEL_MD_RIGHT_A, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_B, OUTPUT);
  pinMode(WHEEL_MD_LEFT_A, OUTPUT);
  pinMode(WHEEL_MD_LEFT_B, OUTPUT);
  pinMode(SUCTION_MD_A, OUTPUT);
  pinMode(SUCTION_MD_B, OUTPUT);
  // サーボ
  pinMode(SERVO_PIN, OUTPUT);
  // 初期ヘルパー関数テスト用状態
  state = STATE_FUNCTION_TEST;
}

void loop() {
    analogWrite(SUCTION_MD_A, 150);
    analogWrite(SUCTION_MD_B, 0);
}

/* --- ヘルパー関数 --- */

// サーボ昇降 (true: 上げる, false: 下げる)
void controlServo(bool up) {
  servo.attach(SERVO_PIN);
  servo.write(up ? 0 : 90);
  delay(500);
  servo.detach();
}

// 吸引オンオフ (true: ON, false: OFF)
void controlSuction(bool on) {
  if (on) {

  } else {
    analogWrite(SUCTION_MD_A, 0);
    analogWrite(SUCTION_MD_B, 0);
  }
}

// 全モーター停止
void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_A, 0);
  analogWrite(WHEEL_MD_RIGHT_B, 0);
  analogWrite(WHEEL_MD_LEFT_A, 0);
  analogWrite(WHEEL_MD_LEFT_B, 0);
  analogWrite(SUCTION_MD_A, 0);
  analogWrite(SUCTION_MD_B, 0);
}
