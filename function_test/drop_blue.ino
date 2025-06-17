#include <Arduino.h>
#include <Encoder.h>

// === 定数定義 ===
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相

#define WHEEL_MD_RIGHT_FORWORD 4  // 右車輪入力 B (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪入力 A (PWM)
#define WHEEL_MD_LEFT_FORWORD  7  // 左車輪入力 A (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪入力 B (PWM)

#define ROTATE_C 5000  // 360度回転時のエンコーダカウント

// PWM設定
#define PWM_RIGHT_MAX 77
#define PWM_LEFT_MAX  55
#define PWM_RIGHT_MIN 65
#define PWM_LEFT_MIN  50

// 減速開始閾値・誤差
#define SLOWDOWN_THRESHOLD 400
#define TOLERANCE 10

// PI制御パラメータ（超簡易版）
#define KP 0.3
#define KI 0.02

// ラインからゴールまで直進を待機する時間
#define TO_RED_GOAL 1000
#define TO_YELLOW_GOAL 1000
#define TO_BLUE_GOAL 2000

Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);

void setup() {
  Serial.begin(115200);

  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);

  encoderRight.write(0);
  encoderLeft.write(0);
}

void loop() {
  // 青色ゴールのモーション
  stopAll();
  delay(1000);

  driveStraight();
  delay(TO_BLUE_GOAL);

  stopAll();
  delay(1000);

  // サーボモータを下げる

  rotateRobot(10, 18);

  stopAll();
  delay(1000);

  driveStraight();
  delay(TO_BLUE_GOAL);

  stopAll();
  delay(1000);

  while (1);
}

void driveStraight() {
  digitalWrite(WHEEL_MD_RIGHT_FORWORD, HIGH);
  digitalWrite(WHEEL_MD_LEFT_FORWORD, HIGH);
}

void rotateRobot(float degree, int repeat) {
  for (int i = 0; i < repeat; i++) {
    encoderRight.write(0);
    encoderLeft.write(0);

    long targetCount = (long)(ROTATE_C * (abs(degree) / 360.0));
    bool clockwise = (degree > 0);

    long errorSum = 0;
    
    while (true) {
      long posRight = encoderRight.read();
      long posLeft  = encoderLeft.read();
      long averagePos = (abs(posRight) + abs(posLeft)) / 2;
      long error = targetCount - averagePos;
      errorSum += error;

      Serial.print("Target:");
      Serial.print(targetCount);
      Serial.print(" Current:");
      Serial.print(averagePos);
      Serial.print(" Error:");
      Serial.println(error);

      // PI制御で速度決定
      float control = KP * error + KI * errorSum;
      control = constrain(control, 0.0, 1.0); // 正規化（0〜1）

      uint8_t pwmRight = max(PWM_RIGHT_MIN, (uint8_t)(PWM_RIGHT_MAX * control));
      uint8_t pwmLeft  = max(PWM_LEFT_MIN,  (uint8_t)(PWM_LEFT_MAX  * control));

      // 出力
      if (clockwise) {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, pwmRight);
        analogWrite(WHEEL_MD_RIGHT_BACK, 0);
        analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
        analogWrite(WHEEL_MD_LEFT_BACK, pwmLeft);
      } else {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
        analogWrite(WHEEL_MD_RIGHT_BACK, pwmRight);
        analogWrite(WHEEL_MD_LEFT_FORWORD, pwmLeft);
        analogWrite(WHEEL_MD_LEFT_BACK, 0);
      }

      if (error <= TOLERANCE) {
        stopAll();
        break;
      }

      delay(20);
    }

    delay(1000); // 各回転の間に少し待つ
  }
}

void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
}
