#include <Servo.h>

const int runStopPin = 8;
const int motorPwmPin = 9;

Servo spark;

const int pwmNeutral = 1500;
const int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_minMotorPwm = 40;

void setup() {
  pinMode(runStopPin, INPUT);
  spark.attach(motorPwmPin);
  spark.writeMicroseconds(pwmNeutral);

  Serial.begin(115200);
  Serial.println("PID Tuner:");
}

void loop() {
  // put your main code here, to run repeatedly:

  int runStop = digitalRead(runStopPin);
  if (runStop == LOW)
  {
    int motorPwm = 1600;
    spark.writeMicroseconds(motorPwm);
  }
  else
  {
    spark.writeMicroseconds(pwmNeutral);
  }
}
