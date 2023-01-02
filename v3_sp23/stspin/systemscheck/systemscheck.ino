//FOR STSPIN32F0A



int P_LED = PF0;
int P_MAG_NCS = PF1;
int P_SNSU = PB1;
int P_SNSV = PA0;
int P_SNSW = PA3;
int P_VBUS = PA4;
int P_TEMP = ATEMP;

//HardwareSerial Serial1(PA15, PA2);

// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(P_LED, OUTPUT);
  pinMode(P_MAG_NCS, OUTPUT);
  Serial.setRx(PA15); // using pin name PY_n
  Serial.setTx(PA2); // using pin number PYn
  Serial.begin(115200);
  while(!Serial);

  analogReadResolution(12);
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(P_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);               // wait for a second
  digitalWrite(P_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(50);               // wait for a second

//  Serial.println(analogRead(P_TEMP));


  float tempr = (1.43 - (3.3 / 4096.0 * analogRead(P_TEMP))) / 0.0043 + 25.0;
  Serial.println(tempr);

}
