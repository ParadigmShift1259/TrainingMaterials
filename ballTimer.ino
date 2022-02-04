int ir_pin = 12;
unsigned long time;
bool broken = false;
const double diameter = 9.5 / 12; // ball diameter in feet

void setup() {
  // put your setup code here, to run once:
  pinMode(ir_pin, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("Ball timer: ");
}

void loop() {
  // put your main code here, to run repeatedly:
  int beamBroken = digitalRead(ir_pin);
//  Serial.print("input: ");
//  Serial.println(beamBroken);
    
  if (beamBroken == LOW && !broken) {
    broken = true;
    time = micros();
  } else if (beamBroken == HIGH && broken) {
    broken = false;
    double elapsed = micros() - time;
    Serial.print("Elapsed: ");
    Serial.print(elapsed);
    Serial.print(" ");
    Serial.print(micros());
    Serial.print(" ");
    Serial.print(time);
    double seconds = elapsed / 100000.0;
    double ballSpeed = diameter / seconds;
    Serial.print(" Speed (ft/s): ");
    Serial.println(ballSpeed);
  }
  
}
