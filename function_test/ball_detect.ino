#include <Arduino.h>
#include <Encoder.h>

// エンコーダピン定義
#define ENC_RIGHT_A    18
#define ENC_RIGHT_B    38
#define ENC_LEFT_A     19
#define ENC_LEFT_B     32

// モータドライバピン
#define WHEEL_MD_RIGHT_FORWORD 4
#define WHEEL_MD_RIGHT_BACK    5
#define WHEEL_MD_LEFT_FORWORD  7
#define WHEEL_MD_LEFT_BACK     6

#define ROTATE_C 5000  // 360度回転あたりのエンコーダカウント
#define PWM_RIGHT_MAX 77
#define PWM_LEFT_MAX  55
#define PWM_RIGHT_MIN 65
#define PWM_LEFT_MIN  50
#define TOLERANCE 10
#define KP 0.3
#define KI 0.02

Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  pinMode(WHEEL_MD_RIGHT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  pinMode(WHEEL_MD_LEFT_FORWORD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);
  encoderRight.write(0);
  encoderLeft.write(0);
  inputString.reserve(50);
}

void loop() {
  if (stringComplete) {
    processSerialData(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processSerialData(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  if (firstComma < 0 || secondComma < 0) return;  // 不正なデータ

  String angleStr = data.substring(secondComma + 1);
  float angle = angleStr.toFloat();

  // もし10度以内なら何もしない
  if (abs(angle) <= 20.0) {
    while(1) {
      stopAll();
      delay(100);
    }
  }

  // 10度だけ回転（方向は検出値に従う）
  int commandAngle = (angle > 0) ? 10 : -10;
  rotateRobot(-commandAngle);  // 向きを反転

  stopAll();
  delay(5000);  // 一時停止して次の測定を待つ
}

void rotateRobot(int degree) {
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

    float control = KP * error + KI * errorSum;
    control = constrain(control, 0.0, 1.0);

    uint8_t pwmRight = max(PWM_RIGHT_MIN, (uint8_t)(PWM_RIGHT_MAX * control));
    uint8_t pwmLeft  = max(PWM_LEFT_MIN,  (uint8_t)(PWM_LEFT_MAX  * control));

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
}

void stopAll() {
  analogWrite(WHEEL_MD_RIGHT_FORWORD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  analogWrite(WHEEL_MD_LEFT_FORWORD, 0);
  analogWrite(WHEEL_MD_LEFT_BACK, 0);
}
