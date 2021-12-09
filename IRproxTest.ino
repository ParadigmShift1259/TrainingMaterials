/*
  IRproxTest

  Detects an IR LED emitter with a IR receiver to be used as a proximity sensor/photoeye

*/

int val = 0;
int beamBrokenCount = 0;
int beamNotBrokenCount = 0;
int analogPin = A3;
int digitalPin = 7;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 7 as an input.
  pinMode(digitalPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  val = digitalRead(digitalPin);
  //val = analogRead(analogPin);
  Serial.print(val);
  Serial.print(" ");
  //if (val <= 200)
  if (val == 0)
  {
    beamBrokenCount++;
    beamNotBrokenCount = 0;
    digitalWrite(LED_BUILTIN, HIGH);   // Beam is broken
    Serial.print(beamBrokenCount);
    Serial.print(" ");
    Serial.println(" Beam is broken--------------------------------");
  }
  else
  {
    beamBrokenCount = 0;
    beamNotBrokenCount++;
    digitalWrite(LED_BUILTIN, LOW);   // No blockage
    Serial.print(beamNotBrokenCount);
    Serial.print(" ");
    Serial.println(" Beam is NOT broken");
  }
  delay(100);                       // wait for a second
}
