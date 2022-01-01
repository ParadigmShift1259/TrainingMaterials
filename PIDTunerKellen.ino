#include <Servo.h>

int runStopPin = 8;
int motorPwmPin = 9;

Servo spark;


int pwmNeutral = 1500;
int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_minMotorPwm = 40;

void setup() {
  // put your setup code here, to run once:
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
    // Send the output to the motor
    spark.writeMicroseconds(motorPwm);
  }
  else {
    spark.writeMicroseconds(pwmNeutral);
  }
  

}
