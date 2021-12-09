/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int span  = 100;
int center = 1500;
int step  = 20;
int offset = 4;
int lo = center - span;
int hi = center + span;
int delayTime = 25;

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(center);
}

void loop() {
  for (pos = center; pos >= lo; pos -= step) { // goes from hi degrees to lo degrees
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                // waits delayTime ms for the servo to reach the position
  }
  delay(1000);

  for (pos = lo; pos <= center; pos += step) { // goes from lo degrees to hi degrees
    // in steps of 1 degree
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                // waits delayTime ms for the servo to reach the position
  }
  delay(1000);
  
  for (pos = center; pos <= hi; pos += step) { // goes from lo degrees to hi degrees
    // in steps of 1 degree
    myservo.writeMicroseconds(pos + offset);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                // waits delayTime ms for the servo to reach the position
  }
  delay(1000);

  for (pos = hi; pos >= center; pos -= step) { // goes from hi degrees to lo degrees
    myservo.write(pos + offset);              // tell servo to go to position in variable 'pos'
    delay(delayTime);                // waits delayTime ms for the servo to reach the position
  }
  delay(1000);
}
