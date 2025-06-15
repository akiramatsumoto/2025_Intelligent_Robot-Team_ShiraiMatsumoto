#include <Arduino.h>
#include <Encoder.h>

// エンコーダピン定義
#define ENC_LEFT_A     19
#define ENC_LEFT_B     32
#define ENC_RIGHT_A    18
#define ENC_RIGHT_B    38

// モータドライバピン定義
#define WHEEL_MD_LEFT_FORWARD  7
#define WHEEL_MD_LEFT_BACK     6
#define WHEEL_MD_RIGHT_FORWARD 4
#define WHEEL_MD_RIGHT_BACK    5

// 制御周期
const unsigned long CONTROL_PERIOD = 20;
const float CONTROL_PERIOD_SEC = CONTROL_PERIOD / 1000.0;
const long targetPulsesPerSec = 3200;

// エンコーダインスタンス
Encoder encLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);

// PIDゲイン
const float KpRight = 0.6 , KiRight = 0.02 * 0.5, KdRight = 0.1;
const float KpLeft  = 0.6, KiLeft  = 0.02 * 0.5, KdLeft  = 0.1;

// 制御変数
long prevCountR = 0, prevCountL = 0;
int pwmRight = 77, pwmLeft = 55;
float integralR = 0, integralL = 0;
long prevErrorR = 0, prevErrorL = 0;
unsigned long prevTime = 0;

void driveLeft(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_LEFT_FORWARD, pwm);
    analogWrite(WHEEL_MD_LEFT_BACK, 0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_LEFT_FORWARD, 0);
    analogWrite(WHEEL_MD_LEFT_BACK, pwm);
  }
}

void driveRight(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_RIGHT_FORWARD, pwm);
    analogWrite(WHEEL_MD_RIGHT_BACK, 0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
    analogWrite(WHEEL_MD_RIGHT_BACK, pwm);
  }
}

void updatePIDControl() {
  long currL = encLeft.read();
  long currR = encRight.read();

  long deltaL = prevCountL - currL;
  long deltaR = currR - prevCountR;

  long targetPulses = (targetPulsesPerSec * CONTROL_PERIOD) / 1000;

  long errorL = targetPulses - deltaL;
  long errorR = targetPulses - deltaR;

  integralL += errorL * CONTROL_PERIOD_SEC;
  integralR += errorR * CONTROL_PERIOD_SEC;

  long derivativeL = (errorL - prevErrorL) / CONTROL_PERIOD_SEC;
  long derivativeR = (errorR - prevErrorR) / CONTROL_PERIOD_SEC;

  pwmLeft  += int(KpLeft * errorL + KiLeft * integralL + KdLeft * derivativeL);
  pwmRight += int(KpRight * errorR + KiRight * integralR + KdRight * derivativeR);

  pwmLeft  = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  Serial.print(errorL);
  Serial.print('\t');
  Serial.println(errorR);

  prevCountL = currL;
  prevCountR = currR;
  prevErrorL = errorL;
  prevErrorR = errorR;
}

void controlLoop() {
  unsigned long now = millis();
  driveLeft(pwmLeft);
  driveRight(pwmRight);
  if (now - prevTime >= CONTROL_PERIOD) {
    updatePIDControl();
    prevTime += CONTROL_PERIOD;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(WHEEL_MD_LEFT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK, OUTPUT);
  encLeft.write(0);
  encRight.write(0);
  prevCountL = encLeft.read();
  prevCountR = encRight.read();
  prevTime = millis();
  Serial.println("ErrL\tErrR");
}

void stopAll() {
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
}

void loop() {
  controlLoop();
  delay(2767);
  while(1) {
    stopAll();
  }
}

