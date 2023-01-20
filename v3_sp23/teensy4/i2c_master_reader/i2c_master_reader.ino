// FOR TEENSY4.0. Connect I2C and UART to STSPIN.

#include <Wire.h>

int led = LED_BUILTIN;

void setup()
{

  pinMode(led, OUTPUT);
  Wire.begin();             // join i2c bus (address optional for master)
  Serial.begin(115200);       // start serial for serial monitor output (println)
  Serial1.begin(115200);      // start serial for input from the STM32
}

char i2c_buf[32];

char temp[4];
int values[4];

char uart_buf[128];

void loop()
{
  digitalWrite(led, HIGH);


  //UART: read and print the Serial coming from the STM32
//  if(Serial1.available()) {
//    for(int i=0; Serial1.available() && i < sizeof(uart_buf); i++){
//      uart_buf[i] = Serial1.read();
//    }
//    Serial.print(uart_buf);
//  }
//
//  strncpy(temp, &uart_buf[5], 4);   // Could be put in a loop...
//  values[0] = atoi(temp);
//  strncpy(temp, &uart_buf[10], 4);
//  values[1] = atoi(temp);
//  strncpy(temp, &uart_buf[15], 4);
//  values[2] = atoi(temp);
//  strncpy(temp, &uart_buf[20], 4);
//  values[3] = atoi(temp);

  
//  for(int i=0; i<4; i++){
//    if(values[i] < 4096){
//      Serial.print(String(i) + ": " + String(values[i]) + ", ");
//    }
//  }
//  Serial.println("");

  if(Serial1.available()) {
    for(int i=0; i < sizeof(uart_buf); i++){
      if(Serial1.available()){
        uart_buf[i] = Serial1.read();
      }else{
        uart_buf[i] = 0;
      }
      if(uart_buf[i] == '\n') break;
    }
//    while(Serial1.available()){
//      Serial1.read();
//    }
    Serial.print(uart_buf);
  }
  

  

  //I2C: send 2 bytes of data to the STM32 then receive 2 bytes back
//  Wire.beginTransmission(9);  // transmit to device #9
//  Wire.write(0xA1);            //send 2 bytes
//  Wire.write(0xB1);        
//  Wire.endTransmission(false);     // stop transmitting data but don't end the entire interaction
//  Wire.requestFrom(9, 2);   // request 2 bytes from peripheral device #9
//  for(int i=0; Wire.available() && i < sizeof(i2c_buf); i++) {
//    i2c_buf[i] = Wire.read();   // receive a byte as character
//    Serial.print(i2c_buf[i], HEX);
//    Serial.print(" ");    
//  }
//  Serial.println("\n");
  

  digitalWrite(led, LOW);

  delay(0);
}
