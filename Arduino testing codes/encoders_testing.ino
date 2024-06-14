/// Pin Definitions
const int encoder1_pin_d0 = 2;  // Digital pin for Encoder 1
const int encoder1_pin_a0 = A0; // Analog pin for Encoder 1
const int encoder2_pin_d0 = 3;  // Digital pin for Encoder 2
const int encoder2_pin_a0 = A1; // Analog pin for Encoder 2

/// Constants
const unsigned int pulses_per_turn = 20;  // Number of notches on the encoder disk
const int wheel_diameter = 64;            // Diameter of the wheel [mm]
const int threshold = 10;                 // Threshold to prevent flickering in direction detection
const unsigned long debounce_time = 500;  // Debounce time in microseconds

/// Variables for Encoder 1
volatile int pulses1 = 0;
volatile bool direction1 = true; // true = CW, false = CCW
static int lastAnalogValue1 = 0;
static unsigned long lastDebounceTime1 = 0;

/// Variables for Encoder 2
volatile int pulses2 = 0;
volatile bool direction2 = true; // true = CW, false = CCW
static int lastAnalogValue2 = 0;
static unsigned long lastDebounceTime2 = 0;

/// Timer
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup for Encoder 1
  pinMode(encoder1_pin_d0, INPUT);
  pinMode(encoder1_pin_a0, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_pin_d0), counter1, RISING);
  
  // Setup for Encoder 2
  pinMode(encoder2_pin_d0, INPUT);
  pinMode(encoder2_pin_a0, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder2_pin_d0), counter2, RISING);

  // Print headers for Serial Monitor
  Serial.println("Seconds\tRPM1\tPulses1\tDirection1\tRPM2\tPulses2\tDirection2");
}

void loop() {
  if (millis() - lastUpdateTime >= 1000) { // Update every second
    noInterrupts();
    
    // Calculate RPM and print results for Encoder 1
    int rpm1 = calculateRPM(pulses1);
    Serial.print(millis() / 1000);
    Serial.print("\t");
    Serial.print(rpm1);
    Serial.print("\t");
    Serial.print(pulses1);
    Serial.print("\t");
    Serial.print(direction1 ? "CW" : "CCW");
    Serial.print("\t");

    // Calculate RPM and print results for Encoder 2
    int rpm2 = calculateRPM(pulses2);
    Serial.print(rpm2);
    Serial.print("\t");
    Serial.print(pulses2);
    Serial.print("\t");
    Serial.println(direction2 ? "CW" : "CCW");

    // Reset pulses for next calculation
    pulses1 = 0;
    pulses2 = 0;
    
    lastUpdateTime = millis();
    interrupts();
  }
}

/// Function to calculate RPM
int calculateRPM(int pulses) {
  return (60 * (1000 / pulses_per_turn)) / (millis() - lastUpdateTime) * pulses;
}

/// Interrupt Service Routine for Encoder 1
void counter1() {
  unsigned long currentTime = micros();
  if ((currentTime - lastDebounceTime1) > debounce_time) {
    lastDebounceTime1 = currentTime;

    int currentAnalogValue = analogRead(encoder1_pin_a0);
    if (currentAnalogValue > lastAnalogValue1 + threshold) {
      direction1 = true;
    } else if (currentAnalogValue < lastAnalogValue1 - threshold) {
      direction1 = false;
    }
    lastAnalogValue1 = currentAnalogValue;

    if (direction1) {
      pulses1++;
    } else {
      pulses1--;
    }
  }
}

/// Interrupt Service Routine for Encoder 2
void counter2() {
  unsigned long currentTime = micros();
  if ((currentTime - lastDebounceTime2) > debounce_time) {
    lastDebounceTime2 = currentTime;

    int currentAnalogValue = analogRead(encoder2_pin_a0);
    if (currentAnalogValue > lastAnalogValue2 + threshold) {
      direction2 = true;
    } else if (currentAnalogValue < lastAnalogValue2 - threshold) {
      direction2 = false;
    }
    lastAnalogValue2 = currentAnalogValue;

    if (direction2) {
      pulses2++;
    } else {
      pulses2--;
    }
  }
}
