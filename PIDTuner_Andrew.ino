#include <Servo.h>
#include <Encoder.h>
#include <LibPrintf.h>

// Pin assignments
int encInterruptPinA = 2;
int encInterruptPinB = 3;
int runStopPin = 8;
int motorPwmPin = 9;

Servo spark;
Encoder feedBackEnc(encInterruptPinA,encInterruptPinB);

int pwmNeutral = 1500;
int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_minMotorPwm = 40;

long encoderMax = 1024;

void setup() {
  pinMode(runStopPin, INPUT);
  spark.attach(motorPwmPin);
  spark.writeMicroseconds(pwmNeutral);
  feedBackEnc.write(512);
  
  Serial.begin(115200);
  printf("PID Tuner:");

  Serial.begin(9600);
}

void loop() {
  int runStop = digitalRead(runStopPin);

  if (runStop == LOW)
  {
    int motorPwm = 1570;
    
    // Send the output to the motor
    spark.writeMicroseconds(motorPwm);

    long measuredPosition = feedBackEnc.read() % encoderMax;

    printf("measuredPosition:%4ld motorPwm:%4d, pwmOffset:%4d\n"
      , measuredPosition
      , motorPwm
      , motorPwm - pwmNeutral);    
  }

  else
  {
    spark.writeMicroseconds(pwmNeutral);
  }
  
}
