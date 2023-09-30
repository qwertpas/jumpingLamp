// To connect a Teensy 4.0 to Ø32controller using a hack to interface RS485 without a transceiver.
// RS485_B of Ø32 goes to a resistor divider between 3.3V and GND of Teensy (1.65V)
// RS485_A of Ø32 goes to TX of Serial2 on Teensy

#include <Metro.h> // periodic function execution
#include "comdef.h" // periodic function execution


#define STICK_X A0
#define STICK_Y A1
#define RS485_DE (9)

#define MIN_INT8 (0x80) //most negative int8

//Metronome

Metro metro_500Hz = Metro(2);  // 2ms, 500Hz
int16_t stick_x_0 = 0;
int16_t stick_y_0 = 0;
uint8_t uart2_TX[7] = {0}; //command RS485 to Ø32
uint8_t uart2_RX[7] = {0}; //response Ø32 from RS485


char uart0_RX[2048] = {0}; //USB serial input from laptop
char uart1_RX[2048] = {0}; //debug UART from Ø32



int16_t twoscomplement14(int16_t num16) {
  // Convert a twos complement int16_t into a twos complement 14 bit number
  int16_t num14 = num16 & 0x3FFF;
  if (num16 & 0x8000) {
    num14 = -((~num14) + 1);
  }
  return num14;
}

int8_t pad7(uint8_t num7) {
  if (num7 & 0x40) return num7 | 0x80;
  else return num7;
}

int16_t pad14(uint8_t num7_0, uint8_t num7_1) {
  int16_t res = (num7_0 << 7) | (num7_1);
  if (res & 0x2000) return res | 0xC000;
  else return res;
}

int16_t clip(int16_t x, int16_t min, int16_t max) {
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}


void setup() {

  Serial.begin(115200);       // start serial for serial monitor output (println)
  Serial1.begin(115200);      // start serial for input from the STM32

  Serial2.begin(115200);      // start serial for RS485 output
  Serial2.setTimeout(1);
  //  Serial2.begin(230400);      // start serial for RS485 output (not captured by scopy)

  stick_x_0 = analogRead(STICK_X);
  stick_y_0 = -analogRead(STICK_Y);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  delay(500);

}

uint32_t count = 0;

int16_t stick_x = 0;
int16_t stick_y = 0;

void loop() {


  stick_y = clip(-analogRead(STICK_Y) - stick_y_0, -511, 511);
  if (stick_y > 100){
    count++;
  }else if(stick_y < -100){
    count--;
  }
  int16_t angle_des = count;
  Serial.print("des: ");
  Serial.println(angle_des);

  int address = 2;
  uart2_TX[0] = CMD_SET_POSITION | address;
  uart2_TX[1] = (angle_des >> 7) & 0b01111111;
  uart2_TX[2] = (angle_des)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  Serial2.write(uart2_TX, 3);
//  delayMicroseconds(260);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);

//  delayMicroseconds(500);

  int numread = Serial2.readBytesUntil(MIN_INT8, uart2_RX, 6);
  while(Serial2.available()) Serial2.read();
  Serial.println(numread);
  for (int i = 0; i < 5; i++) {
    Serial.print(i);
    Serial.print("uart:");
    Serial.println(uart2_RX[i]);
  }

  //    Serial.print("contdes:");
  //    Serial.println(pad14(uart2_RX[0], uart2_RX[1]));
  Serial.print("cont: ");
  Serial.println(pad14(uart2_RX[0], uart2_RX[1]));
  Serial.print("\t\n");

  delay(500);

}
