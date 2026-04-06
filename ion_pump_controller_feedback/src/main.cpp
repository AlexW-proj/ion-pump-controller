#include <Arduino.h>
#include <Wire.h> // This is the library for I2C screens
#include <LiquidCrystal_I2C.h>

// screen stuff, the ERM1604FS-1
// this defines the screen object: backpack, cols, rows
LiquidCrystal_I2C lcd(0x27, 16, 4);

// PIN DEFINITIONS 
const int PWM_PIN         = A0; 
const int FEEDBACK_PIN    = 34; 
const int V_TARGET_PIN    = 39; 
const int RAMP_TIME_PIN   = 36; 

const int START_BTN_PIN   = 14;
const int DRAIN_BTN_PIN   = 32;
const int LED_PIN         = 27;

enum SystemState {
  state_idle,     // val of 0
  state_ramping,  // val of 1
  state_holding,  // val of 2
  state_draining, // val of 3
  state_error     // val of 4
};

// =================
// -- GLOBAL VARS --
// =================
// upon turning on, be in the idle state
SystemState currentState = state_idle;
int currentPWM = 0;
String screenStateText = "IDLE";
unsigned long previousRampTime = 0;
unsigned long previousScreenTime = 0;
unsigned long lastButtonTime = 0;
float smoothedADC = 0;
float smoothedTargetADC =0;

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Set the PWM hardware timer to 50kHz, and 8-bit resolution (0-255)
  analogWriteFrequency(50000); 
  analogWriteResolution(8);

  // Configure Output Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // configure Input Pins 
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(DRAIN_BTN_PIN, INPUT_PULLUP);

  // THE SAFETY LOCKS (Enforce these before anything else runs)
  analogWrite(PWM_PIN, 0);   // Force pump OFF
  digitalWrite(LED_PIN, LOW); // Force LED OFF

  // initialize the I2C Bus
  // SDA is pin 23, SCL is pin 22 by default on the Feather
  Wire.begin(); 

  // Initialize the LCD and turn on the backlight
  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.println("System Booted. High Voltage is OFF.");
  Serial.println("Entering STATE_IDLE.");

  // give buttons time to settle before entering the loop
  delay(200); 

};


void loop() {
  // before deciding which state we want to go in, we need to 
  // read all our data for any given moment

  // =========================================
  // ------------ data read ------------------
  // =========================================

  // read the start / drain buttons, LOW because theyre pull-up Rs
  bool startPressed = digitalRead(START_BTN_PIN) == LOW;
  bool drainPressed = digitalRead(DRAIN_BTN_PIN) == LOW;

  // read raw analog value from the 470k R after the V divider
  int rawADC = analogRead(FEEDBACK_PIN);

  // rolling average for the feedback pin, its noisy otherwise and 
  // can trigger safety mechanisms
  // 10% new reading, 90% old reading.
  // This absorbs sudden noise spikes but still tracks real changes
  smoothedADC = (rawADC * 0.1) + (smoothedADC * 0.9); 

  // this is reversing the voltage divider to get the real value
  // from the max 3.3V pin reading
  float pinVoltage = (smoothedADC / 4095.0) * 3.3;
  float liveVoltage = pinVoltage * 2128.6;

  // reading target voltage
  int rawVoltageKnob = analogRead(V_TARGET_PIN);

  // the reading is noisy, take a rolling average
  smoothedTargetADC = (rawVoltageKnob * 0.1) + (smoothedTargetADC * 0.9);

  long targetVoltage = 0;
  int targetPWM = 0;

  // small deadzone at the start for 0V
  if (smoothedTargetADC < 200) {
    targetVoltage = 0;
    targetPWM = 0;
  }
  else {
    // mapping voltage knob values to usable values
    targetVoltage = map(smoothedTargetADC, 200, 4095, 0, 7000);
    targetPWM = map(targetVoltage, 0, 8000, 0, 255);
    // note that voltages above 7k are inaccessible, safety measure
  }

  // reading target ramp time
  int rawTimeKnob = analogRead(RAMP_TIME_PIN);
  
  // if we use map() with the time knob, 60 min overflows the 32 bit int
  // that map()'s math uses will freak out, we will do the math ourselves
  // (rawTimeKnob / 4095.0) gives us a percentage from 0.0 to 1.0
  // multiply by the difference (3,600,000 - 60,000 = 3,540,000) and add the 60,000 floor.
  unsigned long totalRampTimeMs = 60000 + ((rawTimeKnob / 4095.0) * 3540000.0);

  // calculate step size for ramping, plus div by 0 safety
  unsigned long stepWaitTime = 0;
  if (targetPWM > 0) {
    stepWaitTime = totalRampTimeMs / targetPWM;
  }

  // data read all done, now we have a switch statement
  // to decide what state we will be in, and what to do 
  // in each of those states


  // =======================================================================
  // ------ STATE MACHINE --------------------------------------------------
  // =======================================================================
  switch (currentState) {
    case state_idle:

    // force hardware to safety
    currentPWM = 0;
    analogWrite(PWM_PIN, currentPWM);
    digitalWrite(LED_PIN, LOW);

    // update screen state
    if (liveVoltage >= 24) {
      screenStateText = "DRAINING";
    } else {
      screenStateText = "IDLE";
    }

    // wait for start
    // We check the button, and make sure it has been more than 500 ms since the last button hit (debouncing)
    if (startPressed && (millis() - lastButtonTime > 500)) {
      
      lastButtonTime = millis(); // reset the button stopwatch

      if (liveVoltage < 24) {
        currentState = state_ramping;
      }
    }

    
    break;
    // ---------------------------------------------------------------------------------
    case state_ramping:
    // most dangerous state, need to do safeties first
    digitalWrite(LED_PIN, HIGH); // warning light on
    screenStateText = "RAMPING";

    // escape to drain?
    // another debounced button check
    if (drainPressed && (millis() - lastButtonTime > 500)) {
      lastButtonTime = millis(); // Reset the button stopwatch
      currentState = state_draining;
      break;
    }

    // are we there yet?
    if (currentPWM >= targetPWM) {
      currentState = state_holding;
      break;
    }

    // is it time to increase the PWM?
    if (millis() - previousRampTime >= stepWaitTime) {
  
    // Reset the stopwatch
    previousRampTime = millis(); 
    
    // take one step up
    currentPWM++;                
    analogWrite(PWM_PIN, currentPWM);
    }

    // large voltage spike safety
    // We only check this if the target is above 500V, at very low voltages
    // background noise might accidentally look like a 15% spike
    if (liveVoltage > (targetVoltage * 1.15) && (targetVoltage > 500)) {
      currentState = state_error;
    }

    break;
    // ---------------------------------------------------------------------------------
    case state_holding:
    // keep the warning light going
    digitalWrite(LED_PIN, HIGH);
    screenStateText = "HOLDING";

    // debounced abort button
    if (drainPressed && (millis() - lastButtonTime > 500)) {
      lastButtonTime = millis();
      currentState = state_draining;
      break; 
    }

    // now we balance at the target V
    // We reuse the previousRampTime stopwatch to limit our adjustments
    // to 10 times a second (every 100ms).
    if (millis() - previousRampTime >= 100) {
      
      previousRampTime = millis(); // reset stopwatch

      // deadband: We only adjust if the voltage drifts by more than 50V.
      if (liveVoltage < (targetVoltage - 50)) {
        
        // Voltage is too low. Nudge UP (if we aren't already maxed out)
        if (currentPWM < 255) {
          currentPWM++;
          analogWrite(PWM_PIN, currentPWM);
        }
        
      } else if (liveVoltage > (targetVoltage + 50)) {
        
        // Voltage is too high. Nudge DOWN (if we aren't already at zero)
        if (currentPWM > 0) {
          currentPWM--;
          analogWrite(PWM_PIN, currentPWM);
        }
      }
    }

    // another high V safety
    // If the hardware fails and voltage spikes
    if (liveVoltage > (targetVoltage * 1.15) && (targetVoltage > 500)) {
      currentState = state_error;
    }

    break;
    // ---------------------------------------------------------------------------------
    case state_draining:

    // set target to safety
    currentPWM = 0;
    analogWrite(PWM_PIN, currentPWM);
    digitalWrite(LED_PIN, HIGH);
    screenStateText = "DRAINING";

    // leave draining if were near zero
    if (liveVoltage < 24) {
      currentState = state_idle;
    }

    break;
    // ---------------------------------------------------------------------------------
    case state_error:

    currentPWM = 0;
    analogWrite(PWM_PIN, currentPWM);

    // blink the LED 
    if ((millis() / 250) % 2 == 0) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }

    screenStateText = "VERY BAD";
    // code cannot exit this state, need to turn
    // machine on and off again to get back to idle 

    break;
  }

  // ================================================
  // 3. GLOBAL SCREEN UPDATE (Runs 4 times a second)
  // ================================================

  if ((millis() - previousScreenTime) >= 250) {
    previousScreenTime = millis();


  // line 1: system state
  lcd.setCursor(0, 0);
  lcd.print("STAT:           "); // 16 chars to clear old text (remember null terminator)
  lcd.setCursor(6, 0);
  lcd.print(screenStateText);

  // line 2: live Voltage reading
  lcd.setCursor(0, 1);
  lcd.print("LIVE-V:         ");
  lcd.setCursor(8, 1);
  lcd.print(liveVoltage, 0); // 0 says 0 decimal places
  lcd.print(" V");

  // line 3: voltage target
  lcd.setCursor(0, 2);
  lcd.print("TARG-V:         ");
  lcd.setCursor(8, 2);
  lcd.print(targetVoltage);
  lcd.print(" V");

  // line 4: time 
  int rampMinutes = totalRampTimeMs / 60000;

  lcd.setCursor(0, 3);
  lcd.print("TIME:           ");
  lcd.setCursor(6, 3);
  lcd.print(rampMinutes);
  lcd.print(" MIN");

  }

  // the OS in the background still needs to do stuff
  // the loop is hogging all processing power, OS will explode 
  // if it doesnt get any compute power, use yield to let it do 
  // its chores, shouldnt take long
  yield();

}
