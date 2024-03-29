// FOR TEENSY4.0. Connect I2C and UART to STSPIN.

#include <Wire.h>

#define STICK_X A0
#define STICK_Y A1

int led = LED_BUILTIN;

int16_t stick_x_0 = 0;
int16_t stick_y_0 = 0;

void setup()
{

  pinMode(led, OUTPUT);
  Wire.begin();             // join i2c bus (address optional for master)
  Serial.begin(115200);       // start serial for serial monitor output (println)
  Serial1.begin(115200);      // start serial for input from the STM32
  Serial2.begin(115200);      // start serial for input from the STM32

  stick_x_0 = analogRead(STICK_X);
  stick_y_0 = -analogRead(STICK_Y);

  delay(500);
  digitalWrite(led, HIGH);
}


uint8_t i2c_TX[2] = {1,1}; //out to stm32

int count = 0;
int transmit_i2c = 0;

void loop()
{
  char uart0_RX[2048] = {0}; //in from usb
  char uart1_RX[2048] = {0}; //in from stm32
  uint8_t i2c_RX[2] = {0}; //in from stm32


  //Joystick input
  int16_t stick_x = analogRead(STICK_X) - stick_x_0;
  int16_t stick_y = -analogRead(STICK_Y) - stick_y_0; //between -550 and 550
  if(abs(stick_y) > 61){
    i2c_TX[0] = abs(stick_y) / 100; //scale to 0-9
    transmit_i2c = 1;
//  }else{
//    i2c_
  }


  //USB serial input
  if(Serial.available()) {
    for(int i=0; i < sizeof(uart0_RX); i++){
      if(Serial.available()){
        uart0_RX[i] = Serial.read();
      }
      if(uart0_RX[i] == '\n') break;
    }
    Serial.println(uart0_RX);

    int num = (int)(uart0_RX[0] - '0');
    if(num >= 0 && num <= 9){
      i2c_TX[0] = num;
      transmit_i2c = 1;
    }
  }
    
    
  //UART1 receive all or until buffer filled
  int printall = 1;
  if(Serial1.available()) {
    for(int i=0; i < sizeof(uart1_RX); i++){
      if(Serial1.available()){
        uart1_RX[i] = Serial1.read();
      }else{
        break;
      }
    }
    
    //print to usb
    if(printall){
      Serial.print(uart1_RX);
    }else{
      int start_i = -1;
      for(int i=0; i < sizeof(uart1_RX); i++){
        if(uart1_RX[i] == '\n'){
          if(start_i == -1){
            start_i = i; //found the first newline
          }else{
            Serial.print('\n'); //found the second newline
            break;
          }
        }else if(start_i != -1){ //first newline has been found
          Serial.print(uart1_RX[i]);
        }
      }
    }
  }

  if(count % 10 == 0){
    if (transmit_i2c){
      //  //I2C: send 2 bytes of data to the STM32 then receive 2 bytes back
      Wire.beginTransmission(9);  // transmit to device #9
      Wire.write(i2c_TX[0]);            //send 2 bytes
      Wire.write(i2c_TX[1]);
      Wire.endTransmission(false);     // stop transmitting data but don't end the entire interaction
  //    delay(1);
      Wire.requestFrom(9, 2);   // request 2 bytes from peripheral device #9
      for(int i=0; Wire.available(); i++) {
        char temp = Wire.read();
        if(i < sizeof(i2c_RX)){
          i2c_RX[i] = temp;
        }
      }
      transmit_i2c = 0;

      Serial2.write(abs(stick_y) >> 2);
      Serial2.write(0xBB);

      Serial.println(stick_y);
    }
  }

  

//  Serial2.write(count);
//  Serial2.write(0xBB);
//  Serial2.write(count+1);
  
    delay(1);
    count++;
}
