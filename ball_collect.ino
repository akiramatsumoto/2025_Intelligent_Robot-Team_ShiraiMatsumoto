#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// === 定数定義 ===
#define ENC_RIGHT_A    18
#define ENC_RIGHT_B    38
#define ENC_LEFT_A     19
#define ENC_LEFT_B     32

#define WHEEL_MD_RIGHT_FORWORD 4
#define WHEEL_MD_RIGHT_BACK    5
#define WHEEL_MD_LEFT_FORWORD  7
#define WHEEL_MD_LEFT_BACK     6

#define ROTATE_C 5000

#define PWM_RIGHT_MAX 77
#define PWM_LEFT_MAX  55
#define PWM_RIGHT_MIN 65
#define PWM_LEFT_MIN  50

#define TOLERANCE 10
#define KP 0.3
#define KI 0.02

Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);
  encoderRight.write(0);
  encoderLeft.write(0);

  Wire.begin();
  if (!lox.begin()) {
    Serial.println("VL53L0X 初期化失敗");
    while (1);
  }
  Serial.println("VL53L0X 初期化完了");
}

void loop() {
  encoderRight.write(0);
  encoderLeft.write(0);

  // ボール検出まで直進
  driveStraight();
  while (!isBallDetected()) {
    delay(10);
  }
  stopAll();

  // 走行距離を記録
  long distanceRight = abs(encoderRight.read());
  long distanceLeft  = abs(encoderLeft.read());
  long avgDistance = (distanceRight + distanceLeft) / 2;
  Serial.print("進んだ距離: "); Serial.println(avgDistance);

  delay(1000);

  rotateRobot(10, 18);
  delay(1000);

  stopAll();
  delay(1000);

  // 記録した距離だけ戻る
  encoderRight.write(0);
  encoderLeft.write(0);
  driveStraight();
  while (true) {
    long posR = abs(encoderRight.read());
    long posL = abs(encoderLeft.read());
    long avgPos = (posR + posL) / 2;

    if (avgPos >= avgDistance) {
      stopAll();
      break;
    }
    delay(10);
  }

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
      long posR = encoderRight.read();
      long posL = encoderLeft.read();
      long avgPos = (abs(posR) + abs(posL)) / 2;
      long error = targetCount - avgPos;
      errorSum += error;
      float control = KP * error + KI * errorSum;
      control = constrain(control, 0.0, 1.0);
      uint8_t pwmR = max(PWM_RIGHT_MIN, (uint8_t)(PWM_RIGHT_MAX * control));
      uint8_t pwmL = max(PWM_LEFT_MIN,  (uint8_t)(PWM_LEFT_MAX  * control));

      if (clockwise) {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, pwmR);
        analogWrite(WHEEL_MD_RIGHT_BACK, 0);
        analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
        analogWrite(WHEEL_MD_LEFT_BACK, pwmL);
      } else {
        analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
        analogWrite(WHEEL_MD_RIGHT_BACK, pwmR);
        analogWrite(WHEEL_MD_LEFT_FORWORD, pwmL);
        analogWrite(WHEEL_MD_LEFT_BACK, 0);
      }

      if (error <= TOLERANCE) {
        stopAll();
        break;
      }
      delay(20);
    }
    delay(300);
  }
}

void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
}

bool isBallDetected() {
  VL53L0X_RangingMeasurementData_t meas;
  lox.rangingTest(&meas, false);
  long distance = (meas.RangeStatus != 4) ? meas.RangeMilliMeter : -1;
  return (distance >= 0 && distance <= 80);
}

