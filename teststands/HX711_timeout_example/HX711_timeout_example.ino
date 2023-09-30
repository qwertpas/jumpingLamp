#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
}

void loop() {

  if (scale.wait_ready_timeout(100)) {
    long reading = scale.read();
    Serial.print("force: ");
    Serial.print(reading);
    Serial.print("\n\t");
  } else {
//    Serial.println("HX711 not found.");
  }

//  delay(100);
  
}
