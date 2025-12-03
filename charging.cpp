

enum ChargeState {
  STATE_IDLE = 0,
  STATE_CHARGING,
  STATE_DONE,
  STATE_FAULT
};

ChargeState state = STATE_IDLE;


const int PIN_BATTERY_V   = A0;  
const int PIN_CURRENT     = A1; 
const int PIN_TEMP        = A2;  
const int PIN_RELAY       = 8;   
const int PIN_LED_CHARGING = 9;
const int PIN_LED_FAULT    = 10;
const int PIN_LED_DONE     = 11;
const int PIN_START_BUTTON = 2; 

// UNO default analog reference
const int   ADC_MAX   = 1023;


const float VOLTAGE_DIVIDER_RATIO = 11.0;



const float ACS712_SENSITIVITY = 0.185; // V/A
const float ACS712_ZERO_V      = 2.5;   


const float LM35_MV_PER_C = 10.0;


float MAX_BATTERY_VOLTAGE = 12.6;
float MIN_BATTERY_VOLTAGE = 9.0;    
float MAX_CHARGE_CURRENT  = 5.0;    
float MAX_TEMP_C          = 60.0;   

unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL_MS = 500;


bool lastButtonState = HIGH;
unsigned long lastButtonChangeMs = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 50;


float readBatteryVoltage();
float readChargeCurrent();
float readTemperatureC();
void updateLeds();
bool startButtonPressed();

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);  // charging off

  pinMode(PIN_LED_CHARGING, OUTPUT);
  pinMode(PIN_LED_FAULT, OUTPUT);
  pinMode(PIN_LED_DONE, OUTPUT);

  pinMode(PIN_START_BUTTON, INPUT_PULLUP); // external button to GND

  Serial.begin(115200);
  Serial.println(F("Wireless EV Charging Controller - demo"));
}

void loop() {
  // Read sensors
  float vBat   = readBatteryVoltage();
  float iCharge = readChargeCurrent();
  float tempC  = readTemperatureC();

  // Handle state machine
  switch (state) {
    case STATE_IDLE:
      digitalWrite(PIN_RELAY, LOW);
      if (startButtonPressed()) {
        // basic sanity checks before starting charge
        if (vBat < MAX_BATTERY_VOLTAGE && vBat > MIN_BATTERY_VOLTAGE) {
          state = STATE_CHARGING;
        } else {
          state = STATE_FAULT; // battery voltage out of expected range
        }
      }
      break;

    case STATE_CHARGING:
      digitalWrite(PIN_RELAY, HIGH);  // enable wireless power path

      // Stop when full
      if (vBat >= MAX_BATTERY_VOLTAGE) {
        state = STATE_DONE;
      }

      // Safety cutoffs
      if (vBat < MIN_BATTERY_VOLTAGE ||
          iCharge > MAX_CHARGE_CURRENT ||
          tempC > MAX_TEMP_C) {
        state = STATE_FAULT;
      }
      break;

    case STATE_DONE:
      digitalWrite(PIN_RELAY, LOW); // turn off charging
      if (startButtonPressed()) {
   
        state = STATE_IDLE;
      }
      break;

    case STATE_FAULT:
      digitalWrite(PIN_RELAY, LOW); // hard off

      if (startButtonPressed()) {
        state = STATE_IDLE;
      }
      break;
  }

  // Update LEDs according to state
  updateLeds();


  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    Serial.print(F("Vbat="));
    Serial.print(vBat, 2);
    Serial.print(F(" V  I="));
    Serial.print(iCharge, 2);
    Serial.print(F(" A  T="));
    Serial.print(tempC, 1);
    Serial.print(F(" C  state="));
    Serial.println((int)state);
  }


  delay(10);
}



float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY_V);
  float vAdc = (raw * ADC_REF_V) / ADC_MAX;  // volts at Arduino pin
  float vBat = vAdc * VOLTAGE_DIVIDER_RATIO;
  return vBat;
}

float readChargeCurrent() {
  // Simple averaging for noise reduction
  const int samples = 20;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(PIN_CURRENT);
    delayMicroseconds(200);
  }
  float rawAvg = sum / (float)samples;
  float vAdc = (rawAvg * ADC_REF_V) / ADC_MAX;

  float current = (vAdc - ACS712_ZERO_V) / ACS712_SENSITIVITY; // amps
  if (current < 0) current = 0; // ignore small negative noise
  return current;
}

float readTemperatureC() {
  int raw = analogRead(PIN_TEMP);
  float vAdc = (raw * ADC_REF_V) / ADC_MAX; // volts
  float mV   = vAdc * 1000.0;               // millivolts
  float tempC = mV / LM35_MV_PER_C;         // °C
  return tempC;
}

bool startButtonPressed() {
  bool reading = (digitalRead(PIN_START_BUTTON) == LOW); // active LOW
  unsigned long now = millis();

  if (reading != lastButtonState) {
    lastButtonChangeMs = now;
    lastButtonState = reading;
  }

  // simple debounce
  if ((now - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS && reading) {
    // wait for release to avoid multiple triggers
    while (digitalRead(PIN_START_BUTTON) == LOW) {
      delay(5);
    }
    return true;
  }
  return false;
}

void updateLeds() {
  // Charging LED
  digitalWrite(PIN_LED_CHARGING, state == STATE_CHARGING);

  // Done LED
  digitalWrite(PIN_LED_DONE, state == STATE_DONE);

  // Fault LED (blinking)
  static bool faultOn = false;
  static unsigned long lastToggleMs = 0;

  if (state == STATE_FAULT) {
    unsigned long now = millis();
    if (now - lastToggleMs > 300) {
      lastToggleMs = now;
      faultOn = !faultOn;
      digitalWrite(PIN_LED_FAULT, faultOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(PIN_LED_FAULT, LOW);
  }
}
