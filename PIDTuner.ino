/* PIDTuner
 by Scott Coursin, FRC Team 1259 Paradigm Shift
 This example code is in the public domain.
*/

#include <Servo.h>
#include <Encoder.h>
#include <MovingAverage.h>  // For fixed point
#include <MovingAverager.h> // For floating point
#include <LibPrintf.h>

// Pin assignments
int encInterruptPinA = 2;
int encInterruptPinB = 3;
int runStopPin = 8;
int motorPwmPin = 9;

int kpPotPin = A4;        // Analog pin used to connect the proportional potentiometer
int kiPotPin = A5;        // Analog pin used to connect the integral potentiometer
int kdPotPin = A6;        // Analog pin used to connect the derivative potentiometer
int setPointPotPin = A7;  // Analog pin used to connect to the set point potentiometer

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
Encoder feedBackEnc(encInterruptPinA, encInterruptPinB);

Servo spark;  // create servo object to control a RevRobotics Spark motor controller

MovingAverage<long, 16> measuredPosAvg;
MovingAverage<long, 16> setPointPosAvg;
MovingAverage<long, 16> oscTimeAvg;

MovingAverager kpAvg(10);
MovingAverager kiAvg(10);
MovingAverager kdAvg(10);

int pwmNeutral = 1500;	      // The set point in the center of the range (1.5 ms pulse = neutral output)
int pwmSpan  = 400;	        // Limits the set point to +/- from setPointCenter
int maxRevPwm = pwmNeutral - pwmSpan;		// Max reverse set point
int maxFwdPwm = pwmNeutral + pwmSpan;		// Max forward set point
const int c_minMotorPwm = 40;

long setPoint = 512;          // The desired position, 0 to encoderMax
long measuredPosition = setPoint;	// Where does the encoder say the position is?
long encoderMax = 1024;		  // Maximum encoder pulses in one revolution
long c_setPointChangeTolerance = 20;

int motorOutput = 0;    	  // The commanded output (pulse width)
long totalError = 0;		    // Accumulated error
long prevError = 0;			    // The error calculated last time through the loop
//long c_maxTotalError = encoderMax / 2;
long c_maxTotalError = encoderMax;
long delayTime = 25;
unsigned long loopCount = 0;
const unsigned long c_skipCount = 250;
long pidOscillationTimer;

double kp = 0.0;
double ki = 0.0;
double kd = 0.0;
double kiOffset = 0.06;
double kdOffset = 0.2;
const double c_gainChangeTolerance = 0.03;
const double c_gainLowTolerance = 0.02;

void setup()
{
  pinMode(runStopPin, INPUT);
  spark.attach(motorPwmPin);  // attaches the servo on pin 9 to the servo object
  spark.writeMicroseconds(pwmNeutral);
  feedBackEnc.write(measuredPosition);
  Serial.begin(115200);
  Serial.println("PID Tuner:");
}

void loop()
{
  loopCount++;

  ReadPots();

  int runStop = digitalRead(runStopPin);
  if (runStop == LOW) // Inverted the 5V and ground on the switch
  {
    measuredPosition = measuredPosAvg.add(feedBackEnc.read() % encoderMax);
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
    motorOutput = kp * error  +  ki * totalError  +  kd * (error - prevError);
    // Scale motor output to pulse width
    int motorPwm = map(motorOutput, -encoderMax / 2, encoderMax / 2, maxRevPwm, maxFwdPwm);

    // Adjust for the dead zone when sending PWM; must be more than 60 from the center value
    //if (abs(error) > 1 && abs(motorPwm - pwmNeutral) < 60)
    if (abs(totalError) > (encoderMax / 2) && (abs(motorPwm - pwmNeutral) < c_minMotorPwm))
    {
      if (motorPwm < pwmNeutral)
      {
         motorPwm -= c_minMotorPwm;
      }
      else
      {
         motorPwm += c_minMotorPwm;
      }
    }

    if (error > 0 && prevError < 0)
    {
      // Error changed sign, crossed the x axis
      long now = micros();
      long pidOscTime = now - pidOscillationTimer;
      oscTimeAvg.add(pidOscTime);
      //printf(" oscillation time [us]: %10ld  avg time [us]: %10ld\n", pidOscTime, oscTimeAvg.get());
      pidOscillationTimer = now;
    }

    if (loopCount % c_skipCount == 0)
    {
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
      
    // Send the output to the motor
    spark.writeMicroseconds(motorPwm);
  
    prevError = error;
  }
  else
  {
    // Reset motor output to neutral
    spark.writeMicroseconds(pwmNeutral);
    measuredPosition = measuredPosAvg.add(feedBackEnc.read() % encoderMax);

    // Zero out the accumulated errors
    totalError = 0;
    prevError = 0;

    if (loopCount % c_skipCount == 0)
    {
      printf("SP: %4ld meas: %4ld P: %.2f I: %.2f D: %.2f\n" 
            , setPoint
            , measuredPosition
            , kp
            , ki
            , kd);
    }
  }
}

void ReadPots()
{
    const double maxPGainFactor = (encoderMax / 2) - 1;  // Max gain of 2.0
    const double maxIGainFactor = (encoderMax * 2) - 1;  // Max gain of 1.0
    const double maxDGainFactor = (encoderMax * 2) - 1;  // Max gain of 1.0

    long prevSetPoint = setPoint;
    int setPointPot = 1023 - analogRead(setPointPotPin);   // reads the value of the potentiometer (value between 0 and 1023)
    setPointPot = 1023 - analogRead(setPointPotPin);       // read twice for settling time
    long newSetPoint =  setPointPot; //map(setPointPot, 0, 1023, 1023, 0); // Invert
    if (abs(newSetPoint - prevSetPoint) > c_setPointChangeTolerance)
    {
      setPoint = setPointPosAvg.add(newSetPoint);
    }

    double prevGain = kp;
    int val = 1023 - analogRead(kpPotPin);        // Reads the value of the potentiometer (value between 0 and 1023)
    val = 1023 - analogRead(kpPotPin);                   // read twice for settling time
    double newGain = val / maxPGainFactor;
    if (newGain < c_gainLowTolerance)
    {
      kp = 0.0;
      kpAvg.addSample(0.0);
      newGain = prevGain;
    }
    if (fabs(newGain - prevGain) > c_gainChangeTolerance)
    {
      kp = kpAvg.addSample(newGain);
    }

    prevGain = ki;
    val = 1023 - analogRead(kiPotPin);
    val = 1023 - analogRead(kiPotPin);                   // read twice for settling time
    newGain = val / maxIGainFactor - kiOffset;
    if (newGain < c_gainLowTolerance)
    {
      ki = 0.0;
      totalError = 0.0;
      kiAvg.addSample(0.0);
      newGain = prevGain;
    }
    if (fabs(newGain - prevGain) > c_gainChangeTolerance)
    {
      ki = kiAvg.addSample(newGain);
    }
    
    prevGain = kd;
    val = 1023 - analogRead(kdPotPin);
    val = 1023 - analogRead(kdPotPin);                   // read twice for settling time
    newGain = val / maxDGainFactor - kdOffset;
    if (newGain < c_gainLowTolerance)
    {
      kd = 0.0;
      kdAvg.addSample(0.0);
      newGain = prevGain;
    }
    if (fabs(newGain - prevGain) > c_gainChangeTolerance)
    {
      kd = kdAvg.addSample(newGain);
    }
}
