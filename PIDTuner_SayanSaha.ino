// SAYAN IS BIG DUMB
#include<Servo.h>
#include<Encoder.h>
#include<LibPrintf.h>

//Pin Assignments
int encInterruptPinA = 2;
int encInterruptPinB = 3;
int runStopPin = 8;
int motorPwmPin = 9;

int pwmNeutral = 1500;
int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_MinMotorPwm = 40;
Servo spark;
Encoder feedBackEnc(encInterruptPinA, encInterruptPinB);


void setup() {
  // put your setup code here, to run once:
  pinMode(runStopPin, INPUT);
  spark.writeMicroseconds(pwmNeutral);
  feedBackEnc.write(512);
  Serial.begin(11520);
  Serial.println("PID Tuner: "); 
}

void loop() {
  int runStop = digitalRead(runStopPin); 
  if (runStop == LOW) {
    int motorPwm = 1600;
    spark.writeMicroseconds(motorPwm);
    long meas = (feedBackEnc.read() % 1024);
    printf("meas:%4ld motorPwm:%4d pwmOffset:%4d\n"
    , meas
    , motorPwm
    , motorPwm - pwmNeutral);
  }
  else {
    spark.writeMicroseconds(pwmNeutral);
  }

}
//
