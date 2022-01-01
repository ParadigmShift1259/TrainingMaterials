#include <Servo.h>
#include <Encoder.h>
#include <LibPrintf.h>
#include <MovingAverage.h>
#include <MovingAverager.h>

// Pin assignments
int encInterruptPinA = 2;
int encInterruptPinB = 3;
int runStopPin = 8;
int motorPwmPin = 9;

int pwmNeutral = 1500;
int pwmSpan = 400;
int maxRevPwm = pwmNeutral - pwmSpan;
int maxFwdPwm = pwmNeutral + pwmSpan;
const int c_minMotorPwm = 40;

long setPoint = 512;
long measuredPosition = setPoint;
long encoderMax = 1024;
long c_setPointChangeTolerance = 20;

int motorOutput = 0;
long totalError = 0;
long prevError = 0;
long c_maxTotalError = encoderMax;

double kp = 0.0;
double ki = 0.0;
double kd = 0.0;

double kiOffset = 0.06;
double kdOffset = 0.2;
const double c_gainChangeTolerance = 0.03;
const double c_gainLowTolerance = 0.02;

int kpPotPin = A4;
int kiPotPin = A5; 
int kdPotPin = A6;
int setPointPotPin = A7;

Servo spark;
Encoder feedBackEnc(encInterruptPinA, encInterruptPinB);
MovingAverage<long, 16> mearuredPosAvg;
MovingAverage<long, 16> setPointPosAvg;
MovingAverager kpAvg(10);
MovingAverager kiAvg(10);
MovingAverager kdAvg(10);
MovingAverage<long, 16> oscTimeAvg;


void setup() {
  // put your setup code here, to run once:
  pinMode(runStopPin, INPUT);
  spark.attach(motorPwmPin); // attaches the servo on pin 9
  spark.writeMicroseconds(pwmNeutral);
  feedBackEnc.write(512);
  Serial.begin(115200);
  Serial.println("PID Tuner:");
}

void loop() {
  ReadPots();
  
  int runStop = digitalRead(runStopPin);
  if (runStop == LOW)
  {

    long measuredPosition = (feedBackEnc.read() % encoderMax);
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
    int motorPwm = map(motorOutput, -encoderMax / 2, encoderMax / 2, maxRevPwm, maxFwdPwm);
    spark.writeMicroseconds(motorPwm); 

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
  }
  else
  {
    spark.writeMicroseconds(pwmNeutral);

  }
}

void ReadPots()
{
    const double maxPGainFactor = (encoderMax / 2) - 1;
    const double maxIGainFactor = (encoderMax * 2) - 1;
    const double maxDGainFactor = (encoderMax * 2) - 1;

    long prevSetPoint = setPoint;
    int setPointPot = 1023 - analogRead(setPointPotPin);
    setPointPot = 1023 - analogRead(setPointPotPin);
    long newSetPoint = setPointPot;
    if (abs(newSetPoint - prevSetPoint) > c_setPointChangeTolerance)
    {
      setPoint = setPointPosAvg.add(newSetPoint);
    }
    
    double prevGain = kp;
    int val = 1023 - analogRead(kpPotPin);
    val = 1023 - analogRead(kpPotPin);
    double newGain = val / maxPGainFactor;
    if (newGain < c_gainLowTolerance)
    {
      kp = 0.0;
      kpAvg.addSample(0.0);
      newGain = prevGain;
    }
}
