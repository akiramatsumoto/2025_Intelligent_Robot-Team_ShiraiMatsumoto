// Encoderライブラリ(PCINT)を使った左右車輪PID制御サンプル
#include <Arduino.h>
#include <Encoder.h>  // PCINTを用いたエンコーダ処理

// エンコーダピン定義
#define ENC_LEFT_A     19  // 左車輪 A 相
#define ENC_LEFT_B     32  // 左車輪 B 相
#define ENC_RIGHT_A    18  // 右車輪 A 相
#define ENC_RIGHT_B    38  // 右車輪 B 相

// 車輪モータドライバ (AM2837)
#define WHEEL_MD_LEFT_FORWARD  7  // 左車輪前進 (PWM)
#define WHEEL_MD_LEFT_BACK     6  // 左車輪後退 (PWM)
#define WHEEL_MD_RIGHT_FORWARD 4  // 右車輪前進 (PWM)
#define WHEEL_MD_RIGHT_BACK    5  // 右車輪後退 (PWM)

// 制御周期設定 (ミリ秒)
const unsigned long CONTROL_PERIOD = 20;  // 任意に変更可能
const float CONTROL_PERIOD_SEC = CONTROL_PERIOD / 1000.0;  // 秒単位

// 1秒あたりのエンコーダパルス数目標 (1回転=約3200パルス)
const long targetPulsesPerSec = 3200;

// Encoderオブジェクト生成 (PCINT対応)
Encoder encLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoder encRight(ENC_RIGHT_A, ENC_RIGHT_B);

// PIDゲイン (要調整)
const float KpRight = 0.6 , KiRight = 0.02 * 0.5, KdRight = 0.1;
const float KpLeft  = 0.6, KiLeft  = 0.02 * 0.5, KdLeft  = 0.1;

// 制御用変数
long prevCountR = 0;
long prevCountL = 0;
int pwmRight    = 77;  // 初期PWM値（右）
int pwmLeft     = 55;  // 初期PWM値（左）
unsigned long prevTime = 0;

float integralR = 0, integralL = 0;
long prevErrorR = 0, prevErrorL = 0;

void setup() {
  Serial.begin(115200);

  pinMode(WHEEL_MD_LEFT_FORWARD,  OUTPUT);
  pinMode(WHEEL_MD_LEFT_BACK,     OUTPUT);
  pinMode(WHEEL_MD_RIGHT_FORWARD, OUTPUT);
  pinMode(WHEEL_MD_RIGHT_BACK,    OUTPUT);

  encLeft.write(0);
  encRight.write(0);
  prevCountL = encLeft.read();
  prevCountR = encRight.read();
  prevTime   = millis();

  Serial.println("ErrL\tErrR");
}

void loop() {
  unsigned long now = millis();

  driveLeft(pwmLeft); 
  driveRight(pwmRight);

  if (now - prevTime >= CONTROL_PERIOD) {
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

    pwmLeft  = constrain(pwmLeft,  0, 255);
    pwmRight = constrain(pwmRight, 0, 255);

    Serial.print(errorL);
    Serial.print('\t');
    Serial.println(errorR);

    prevCountL = currL;
    prevCountR = currR;
    prevErrorL = errorL;
    prevErrorR = errorR;
    prevTime   += CONTROL_PERIOD;
  }
}

void driveLeft(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_LEFT_FORWARD, pwm);
    analogWrite(WHEEL_MD_LEFT_BACK,    0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_LEFT_FORWARD, 0);
    analogWrite(WHEEL_MD_LEFT_BACK,    pwm);
  }
}

void driveRight(int pwm) {
  if (pwm >= 0) {
    analogWrite(WHEEL_MD_RIGHT_FORWARD, pwm);
    analogWrite(WHEEL_MD_RIGHT_BACK,    0);
  } else {
    pwm = -pwm;
    analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
    analogWrite(WHEEL_MD_RIGHT_BACK,    pwm);
  }
}

void stopAll() {
  analogWrite(WHEEL_MD_LEFT_FORWARD,  0);
  analogWrite(WHEEL_MD_LEFT_BACK,     0);
  analogWrite(WHEEL_MD_RIGHT_FORWARD, 0);
  analogWrite(WHEEL_MD_RIGHT_BACK,    0);
}
