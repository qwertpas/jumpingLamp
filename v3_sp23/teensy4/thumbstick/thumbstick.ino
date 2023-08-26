// To connect a Teensy 4.0 to Ø32controller using a hack to interface RS485 without a transceiver.
// RS485_B of Ø32 goes to a resistor divider between 3.3V and GND of Teensy (1.65V)
// RS485_A of Ø32 goes to TX of Serial2 on Teensy


#include <Metro.h> // periodic function execution
#include "comdef.h" // periodic function execution


#define STICK_X A0
#define STICK_Y A1


Metro metro_500Hz = Metro(1);  // 2ms, 500Hz
int16_t stick_x_0 = 0;
int16_t stick_y_0 = 0;
uint8_t uart2_TX[7] = {0}; //command RS485 to Ø32


char uart0_RX[2048] = {0}; //USB serial input from laptop
char uart1_RX[2048] = {0}; //debug UART from Ø32



void setup() {

  Serial.begin(115200);       // start serial for serial monitor output (println)
  Serial1.begin(115200);      // start serial for input from the STM32
  
  Serial2.begin(115200);      // start serial for RS485 output
//  Serial2.begin(230400);      // start serial for RS485 output (not captured by scopy)

  stick_x_0 = analogRead(STICK_X);
  stick_y_0 = -analogRead(STICK_Y);

  delay(500);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

uint8_t count = 0;

void loop() {

  if (metro_500Hz.check()) {

    //Joystick input
    int16_t stick_x = analogRead(STICK_X) - stick_x_0;
    int16_t stick_y = -analogRead(STICK_Y) - stick_y_0; //between -550 and 550

    if(count % 2 == 0){
      if(count % 4 == 0){
        uart2_TX[0] = CMD_SET_VOLTAGE | 2;
      }else{
        uart2_TX[0] = CMD_SET_SPEED | 2;
      }
      uart2_TX[1] = count % 0b01111111;
      uart2_TX[2] = (count+1) % 0b01111111;
      Serial2.write(uart2_TX, 3);
    }else{
      uart2_TX[0] = CMD_SET_VOLTAGE | 3;
      uart2_TX[1] = count % 0b01111111;
      uart2_TX[2] = (count+1) % 0b01111111;
      Serial2.write(uart2_TX, 3);
    }

//    uart2_TX[0] = CMD_SET_VOLTAGE | 3;
//    uart2_TX[1] = count % 0b01111111;
//    Serial2.write(uart2_TX, 2);
//
//    uart2_TX[0] = CMD_SET_VOLTAGE | 4;
//    uart2_TX[1] = count % 0b01111111;
//    Serial2.write(uart2_TX, 2);
    
   
    count += 1;


    
//    uart2_TX[1] = abs(stick_x) >> 2;
//    uart2_TX[2] = 0xAA;

//    uart2_TX[0] = count;
//    uart2_TX[1] = count+1;
//    uart2_TX[2] = count+2;
//    uart2_TX[3] = count+3;
//    uart2_TX[4] = count+4;
//    uart2_TX[5] = count+5;
//    uart2_TX[6] = count+6;
//    Serial2.write(uart2_TX, 7);

  }



  //USB serial input goes into uart0_RX
  if (Serial.available()) {
    for (uint8_t i = 0; i < sizeof(uart0_RX); i++) {
      if (Serial.available()) {
        uart0_RX[i] = Serial.read();
      }
      if (uart0_RX[i] == '\n') break;
    }
    Serial.println(uart0_RX);
  }



  //UART1 passthrough for debugging
  if (Serial1.available()) {
    for (uint8_t i = 0; i < sizeof(uart1_RX); i++) {
      if (Serial1.available()) {
        uart1_RX[i] = Serial1.read();
      } else {
        break;
      }
    }
    Serial.print(uart1_RX);
  }


}
