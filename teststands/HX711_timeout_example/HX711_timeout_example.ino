#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 17;

HX711 scale;
float N_per_ct = 0.222e-5 * 9.81;
float calib = 0;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  int samples = 0;
  float alpha = 0.5;

  while(samples < 10){
    if (scale.wait_ready_timeout(100)) {
      float newtons = scale.read() * N_per_ct;
      calib = alpha*newtons + (1-alpha)*calib;
      Serial.print("calib: ");
      Serial.print(calib,4);
      Serial.print("\n\t");
      samples++;
    }else{
    }
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);


  if (scale.wait_ready_timeout(100)) {
    float newtons = scale.read() * N_per_ct - calib;
    Serial.print("force: ");
    Serial.print(newtons,4);
    Serial.print("\n\t");
  } else {
//    Serial.println("HX711 not found.");
  }

//  delay(100);
  
}
