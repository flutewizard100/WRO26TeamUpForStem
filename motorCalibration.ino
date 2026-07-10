#include <Servo.h>

Servo esc;

void setup() {
  // Open Serial Monitor at 9600 baud
  Serial.begin(9600);

  delay(5000);
  
  // Attach ESC signal to digital pin 9
  esc.attach(9, 1000, 2000);
  
  // 5-second initial safety delay. 
  Serial.println("===============================================");
  Serial.println("  HOBBYWING WP 1080 G2 CALIBRATION STARTING  ");
  Serial.println("===============================================");
  Serial.println("1. Turn ESC OFF.");
  Serial.println("2. Hold physical SET button.");
  Serial.println("3. Click Power button ON.");
  Serial.println("4. Release SET button when ESC rapidly flashes red/beeps.");
  Serial.println("\n[COUNTDOWN] You have 5 seconds to enter calibration mode...");
  
  delay(5000);
  // ==========================================
  // STEP 1: NEUTRAL SIGNAL (1500us)
  // ==========================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[STEP 1/3] Sending NEUTRAL Signal (1500us)...");
  Serial.println("--> CLICK THE PHYSICAL SET BUTTON *ONCE* NOW!");
  Serial.println("--> Listen for: 1 Beep / 1 Flash.");
  Serial.println("-----------------------------------------------");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    esc.writeMicroseconds(1500);
  }
  
  Serial.println(">> Step 1 Neutral Finished! Get ready for Step 2.");

  // ==========================================
  // STEP 2: FULL FORWARD SIGNAL (2000us)
  // ==========================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[STEP 2/3] Sending FULL FORWARD Signal (2000us)...");
  Serial.println("--> CLICK THE PHYSICAL SET BUTTON *ONCE* NOW!");
  Serial.println("--> Listen for: 2 Beeps / 2 Flashes.");
  Serial.println("-----------------------------------------------");
  
  startTime = millis();
  while (millis() - startTime < 8000) {
    esc.writeMicroseconds(2000);
  }
  
  Serial.println(">> Step 2 Forward Finished! Get ready for Step 3.");

  // ==========================================
  // STEP 3: FULL REVERSE SIGNAL (1000us)
  // ==========================================
  Serial.println("\n-----------------------------------------------");
  Serial.println("[STEP 3/3] Sending FULL REVERSE Signal (1000us)...");
  Serial.println("--> CLICK THE PHYSICAL SET BUTTON *ONCE* NOW!");
  Serial.println("--> Listen for: 3 Beeps / 3 Flashes.");
  Serial.println("-----------------------------------------------");
  
  startTime = millis();
  while (millis() - startTime < 8000) {
    esc.writeMicroseconds(1000);
  }
  
  // ==========================================
  // CALIBRATION COMPLETE
  // ==========================================
  Serial.println("\n===============================================");
  Serial.println("         CALIBRATION TIMELINE COMPLETE         ");
  Serial.println("===============================================");
  Serial.println("If you successfully heard 1, 2, then 3 beeps:");
  Serial.println("--> HOLD THE POWER BUTTON FOR 3 SECONDS TO TURN ESC OFF.");
  Serial.println("\nAfter powering off, upload your main driving code.");
}

void loop() {
  // Keep sending neutral signal safely after finishing
  esc.writeMicroseconds(1500); 
}
