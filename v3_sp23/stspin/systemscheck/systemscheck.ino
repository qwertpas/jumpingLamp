//FOR STSPIN32F0A which uses STM32F031C6



#define P_LED PF0;
#define P_MAG_NCS PF1;
#define P_SNSU PB1;
#define P_SNSV PA0;
#define P_SNSW PA3;
#define P_VREF AVREF; //internal ADC reference
#define P_VBUS PA4;
#define P_TEMP ATEMP;
#define P_LSU PB13;
#define P_LSV PB14;
#define P_LSW PB15;
#define P_HSU PA8;
#define P_HSV PA9;
#define P_HSW PA10;

uint16_t VREFINT_CAL;                        // VREFINT calibration value

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
  VREFINT_CAL= *((uint16_t*) 0x1FFFF7BA);     // read VREFINT_CAL_ADDR memory
}

uint32_t readvolt(int pin){ //can use uint64_t for more precision
//  return (uint64_t)(330) * (uint64_t)analogRead(pin) * (uint64_t)VREFINT_CAL / (analogRead(P_VREF) * 4095);
  return 330 * analogRead(pin) / 4095;
  
}

float vbus = 8;
float vbus2 = 8;
float vbus3 = 8;
float alpha = 0.9;


// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(P_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(P_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second

//  Serial.println(analogRead(P_TEMP));

//  float vrefint = 3.3*VREFINT_CAL/4095.;
//  float vrefext = vrefint * 4095. / analogRead(P_VREF);
  vbus = (alpha)*vbus + (1-alpha)*readvolt(P_VBUS)*512;
  vbus2 = (alpha)*vbus2 + (1-alpha)*(3.3 * VREFINT_CAL * analogRead(P_VBUS) / analogRead(P_VREF) / 4095. * 5.12);
//    int vbus = read_volt(P_VBUS) * 5.12;
  vbus3 = (alpha)*vbus3 + (1-alpha)*(3.3*analogRead(P_VBUS)/4095. * 5.12);


//  float tempr = (1.43 - (3.3 / 4096.0 * analogRead(P_TEMP))) / 0.0043 + 25.0;
//  Serial.println("temp: " + String(tempr));
  Serial.println("vbus: " + String(vbus) + ", vbus2: " + String(vbus2) + ", vbus3: " + String(vbus3));
//  Serial.println("vbus2: " + String(vbus2));
//  Serial.println("vrefint: " + String(vrefint));
//  Serial.println("vrefext: " + String(vrefext));

  

}
