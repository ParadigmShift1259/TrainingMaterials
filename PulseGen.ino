/**
 * A simple PWM/signal generator that permits setting of the frequency
 * in Herz and the pulse width (duty cycle). It uses a bit banging method
 * to generate the signal. This sketch can also be used to generate a PWM 
 * signal on any digital pin.
 * 
 * Author: Mario Gianota July 2020
 */
#define OUTPUT_PIN 2  // PWM/Signal output pin
#define PULSE_INPUT_PIN 3

float frequency;        // Frequency in Herz
float dutyCycle;      // Duty cycle (pulse width) percentage
unsigned long duration;

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(PULSE_INPUT_PIN, INPUT);
  Serial.begin(9600);

  // Set the frequency and the duty cycle. For most
  // purposes you will want to leave the duty cycle set
  // to 50%.
  frequency = 244;
  //dutyCycle = 98; // ~4096
  //dutyCycle = 50; // 2048
  dutyCycle = 25; // 1024
  //dutyCycle = 2; // 
  //dutyCycle = 1; //  should be 1 (1us), measured 44us
  //dutyCycle = 0.1; // 7us
  //dutyCycle = 0.05; // 5us
  //dutyCycle = 0.01; // 3.4us minimum at 244 Hz
}

void loop() {

  // Calculate the period and the amount of time the output is on for (HIGH) and 
  // off for (LOW).
  double period = 1000000.0 / frequency;
  double offFor = period - (period * (dutyCycle / 100.0));
  double onFor = period - offFor;

  if (period > 16383.0)
  {
    // If the period is greater than 16383 then use the millisecond timer delay,
    // otherwise use the microsecond timer delay. Currently, the largest value that
    // will produce an accurate delay for the microsecond timer is 16383.

    duration = pulseIn(PULSE_INPUT_PIN, HIGH);
    
    digitalWrite(OUTPUT_PIN, HIGH);
    delay((long)onFor/1000);
    
    digitalWrite(OUTPUT_PIN, LOW);
    delay((long)offFor/1000);
  } else {
    duration = pulseIn(PULSE_INPUT_PIN, HIGH);
    
    digitalWrite(OUTPUT_PIN, HIGH);
    delayMicroseconds((long)onFor);

    digitalWrite(OUTPUT_PIN, LOW);
    delayMicroseconds((long)offFor);
  }

  Serial.println(duration);

  //dutyCycle += 0.1;
  //if (dutyCycle > 98.0)
  //{
  //  dutyCycle = 0.1;
  //}
}
