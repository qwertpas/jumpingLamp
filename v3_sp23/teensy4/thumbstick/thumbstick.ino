// To connect a Teensy 4.0 to Ø32controller using a hack to interface RS485 without a transceiver.
// RS485_B of Ø32 goes to a resistor divider between 3.3V and GND of Teensy (1.65V)
// RS485_A of Ø32 goes to TX of Serial2 on Teensy


#define STICK_X A0
#define STICK_Y A1

int led = LED_BUILTIN;

int16_t stick_x_0 = 0;
int16_t stick_y_0 = 0;

void setup()
{

  pinMode(led, OUTPUT);
  Serial.begin(115200);       // start serial for serial monitor output (println)
  Serial1.begin(115200);      // start serial for input from the STM32
  Serial2.begin(115200);      // start serial for RS485 output 

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


  //Joystick input
  int16_t stick_x = analogRead(STICK_X) - stick_x_0;
  int16_t stick_y = -analogRead(STICK_Y) - stick_y_0; //between -550 and 550

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
    
      Serial2.write(abs(stick_y) >> 2);
//      Serial2.write(0xBB);

      Serial.println(stick_y);
    
  }
  
    delay(1);
    count++;
}
