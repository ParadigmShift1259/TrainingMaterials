#include <Servo.h>
#include <Encoder.h>
#include <MovingAverage.h>
#include <MovingAverager.h>
#include <LibPrintf.h>

// Pin assignments
int encInterruptPinA = 2;
int encInterruptPinB = 3;
int runStopPin = 8;
int motorPwmPin = 9;

MovingAverage<long, 16> measuredPosAvg;
MovingAverage<long, 16> setPointPosAvg;
MovingAverage<long, 16> oscTimeAvg;

MovingAverager kpAvg(10);
MovingAverager kiAvg(10);
MovingAverager kdAvg(10);

long setPoint = 512;  
int kpPotPin = A4;
int kiPotPin = A5;
int kdPotPin = A6;
int setPointPotPin = A7;  // Analog pin used to connect to the set point potentiometer

Servo spark;
Encoder feedBackEnc(encInterruptPinA,encInterruptPinB);

int pwmNeutral = 1500;
int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_minMotorPwm = 40;

long measuredPosition = setPoint;
long encoderMax = 1024;
long c_setPointChangeTolerance = 20;

double kp = 0.0;
double ki = 0.0;
double kd = 0.0;

double kiOffset = 0.06;
double kdOffset = 0.2;
const double c_gainChangeTolerance = 0.03;
const double c_gainLowTolerance = 0.02;

String bestGameOfAllTime = "Among Us, a 2018 game created by Innersloth Studios";

int motorOutput = 0;
long totalError = 0;
long prevError = 0;
// long c_maxTotalError = encoderMax / 2;
long c_maxTotalError = encoderMax;
long delayTime = 25;
unsigned long loopCount = 0;
const unsigned long c_skipCount = 250;
long pidOscillationTimer;

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
  ReadPots();
  
  int runStop = digitalRead(runStopPin);

  if (runStop == LOW)
  {

    long measuredPosition = feedBackEnc.read() % encoderMax;
    long error = measuredPosition - setPoint;
    totalError += error;
    if (totalError > c_maxTotalError)
    {
      totalError = c_maxTotalError;
    }
    else if (totalError < -1 * c_maxTotalError)
    {
      totalError = -1 * c_maxTotalError;
    }
    motorOutput = kp * error + ki * totalError + kd * (error - prevError);
    int motorPwm = map(motorOutput
                    , -encoderMax / 2
                    , encoderMax / 2
                    , maxRevPwm, maxFwdPwm);
    printf("SP:%4ld meas:%4ld error:%5ld prevErr:%5ld totalErr:%5ld motorOut:%4d motorPwm:%4d pwmOffset:%4d osc[us] %7ld P:%.2f I:%.2f D:%.2f\n" 
            , setPoint
            , measuredPosition
            , error
            , prevError
            , totalError
            , motorOutput
            , motorPwm
            , motorPwm - pwmNeutral
            , oscTimeAvg.get()
            , kp
            , ki
            , kd);   
    // Send the output to the motor
    spark.writeMicroseconds(motorPwm);
  }
  else
  {
    spark.writeMicroseconds(pwmNeutral);
  }
  Serial.println("amogu\n");
}
void ReadPots()
{
  const double maxPGainFactor = (encoderMax / 2) - 1;
  const double maxIGainFactor = (encoderMax * 2) - 1;
  const double maxDGainFactor = (encoderMax * 2) - 1;

  // update main dial (set point) value
  long prevSetPoint = setPoint;
  int setPointPot = 1023 - analogRead(setPointPotPin);
  setPointPot = 1023 - analogRead(setPointPotPin);
  long newSetPoint = setPointPot;
  if (abs(newSetPoint - prevSetPoint) > c_setPointChangeTolerance)
  {
    setPoint = setPointPosAvg.add(newSetPoint);
    Serial.println("amgos");
  }

  // update P dial value
  double prevPGain = kp;
  int val = 1023 - analogRead(kpPotPin);
  val = 1023 - analogRead(kpPotPin);
  double newPGain = val / maxPGainFactor;
  if (newPGain < c_gainLowTolerance)
  {
    kp = 0.0;
    kpAvg.addSample(0.0);
    newPGain = prevPGain;   
  }

//  // update I dial value
//  double prevIGain = ki;
//  int val = 1023 - analogRead(kiPotPin);
//  val = 1023 - analogRead(kiPotPin);
//  double newIGain = val / maxIGainFactor;
//  if (newIGain < c_gainLowTolerance)
//  {
//    ki = 0.0;
//    kiAvg.addSample(0.0);
//    newIGain = prevIGain;   
//  }
//
//  // update D dial value
//  double prevDGain = kd;
//  int val = 1023 - analogRead(kdPotPin);
//  val = 1023 - analogRead(kdPotPin);
//  double newDGain = val / maxDGainFactor;
//  if (newDGain < c_gainLowTolerance)
//  {
//    kd = 0.0;
//    kdAvg.addSample(0.0);
//    newDGain = prevDGain;   
//  }
}
