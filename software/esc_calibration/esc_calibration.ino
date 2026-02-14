#include <ESP32Servo.h>

Servo esc;

// Define your PWM pin
const int ESC_PIN = 33;  // Change to your actual pin

// Standard PWM values (microseconds)
const int PWM_MIN = 1000;  // Minimum throttle
const int PWM_MAX = 2000;  // Maximum throttle

// Set this to true if you want to do calibration
const bool DO_CALIBRATION = true;  // Change to true for calibration

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  esc.setPeriodHertz(50);    // Standard 50 Hz servo signal
  esc.attach(ESC_PIN, PWM_MIN, PWM_MAX);
  
  Serial.println("=================================");
  Serial.println("ESP32 ESC CONTROL");
  Serial.println("=================================");
  Serial.println();
  
  if (DO_CALIBRATION) {
    Serial.println("⚠️  CALIBRATION MODE ENABLED");
    Serial.println();
    Serial.println("POWER SETUP:");
    Serial.println("1. ESP32 powered by PC USB (already done)");
    Serial.println("2. Connect ONLY signal + GND to ESC");
    Serial.println("3. Do NOT connect ESC 5V wire to ESP32!");
    Serial.println();
    calibrateESC();
  } else {
    Serial.println("NORMAL OPERATION MODE");
    Serial.println();
    Serial.println("POWER SETUP:");
    Serial.println("Option A: ESP32 from USB, ESC from battery");
    Serial.println("  - Connect: Signal + GND only");
    Serial.println("  - Do NOT connect ESC 5V to ESP32");
    Serial.println();
    Serial.println("Option B: Both from battery (via ESC BEC)");
    Serial.println("  - Connect all 3 wires: Signal + GND + 5V");
    Serial.println("  - Remove USB cable from ESP32");
    Serial.println();
    Serial.println("Starting ESC in 3 seconds...");
    delay(3000);
    
    // Arm the ESC (send minimum throttle)
    Serial.println("Arming ESC (sending minimum throttle)...");
    esc.writeMicroseconds(PWM_MIN);
    delay(2000);
    
    Serial.println("ESC should beep now (battery cell detection)");
    Serial.println("Ready for operation!");
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  0-100  = Set throttle percent");
    Serial.println("  t      = Run test sequence");
    Serial.println();
  }
}

void calibrateESC() {
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("CALIBRATION PROCEDURE");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
  
  Serial.println("STEP 1: Setting throttle to MAXIMUM");
  Serial.println("        PWM = " + String(PWM_MAX) + " μs");
  esc.writeMicroseconds(PWM_MAX);
  
  Serial.println();
  Serial.println(">>> NOW CONNECT BATTERY TO ESC <<<");
  Serial.println();
  Serial.println("Wait for: ●● ●● ●● ●● (4 double beeps)");
  Serial.println();
  Serial.println("Type 'ok' and press Enter when you hear beeps:");
  
  // Wait for user confirmation
  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("ok")) break;
    }
    delay(100);
  }
  
  Serial.println();
  Serial.println("STEP 2: Setting throttle to MINIMUM");
  Serial.println("        PWM = " + String(PWM_MIN) + " μs");
  esc.writeMicroseconds(PWM_MIN);
  
  Serial.println();
  Serial.println("Wait for: ●● (2 beeps = SUCCESS!)");
  Serial.println();
  
  delay(5000);
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("✓ CALIBRATION COMPLETE!");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
  Serial.println("ESC calibrated:");
  Serial.println("  MIN = " + String(PWM_MIN) + " μs (0% throttle)");
  Serial.println("  MAX = " + String(PWM_MAX) + " μs (100% throttle)");
  Serial.println();
  Serial.println("You can now:");
  Serial.println("1. Set DO_CALIBRATION = false");
  Serial.println("2. Re-upload code");
  Serial.println("3. Use normal operation mode");
  Serial.println();
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "t" || cmd == "test") {
      runTestSequence();
    } else {
      int throttle = cmd.toInt();
      if (throttle >= 0 && throttle <= 100) {
        setThrottlePercent(throttle);
      } else {
        Serial.println("Invalid command. Use 0-100 or 't' for test");
      }
    }
  }
}

void setThrottlePercent(float percent) {
  percent = constrain(percent, 0, 100);
  int pwm = PWM_MIN + (PWM_MAX - PWM_MIN) * (percent / 100.0);
  esc.writeMicroseconds(pwm);
  Serial.println("→ Throttle: " + String(percent, 1) + "% (" + String(pwm) + " μs)");
}

void runTestSequence() {
  Serial.println();
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("RUNNING TEST SEQUENCE");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  int steps[] = {0, 25, 50, 75, 100, 75, 50, 25, 0};
  int numSteps = sizeof(steps) / sizeof(steps[0]);
  
  for (int i = 0; i < numSteps; i++) {
    Serial.println();
    Serial.println("Step " + String(i+1) + "/" + String(numSteps) + ": " + String(steps[i]) + "% throttle");
    setThrottlePercent(steps[i]);
    delay(2000);
  }
  
  Serial.println();
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("TEST COMPLETE");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
}