#include "HX711.h"
#include "comdef.h"

#define RS485_DE (9)
#define MIN_INT8 (0x80) //most negative int8
#define MOT_ADDR 3
#define STICK_X A0
#define STICK_Y A1

#define ang_per_ct (90/4096.0)

uint8_t uart2_RX[5] = {0}; //response Ø32 from RS485

int16_t clip(int16_t x, int16_t min, int16_t max) {
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}

int16_t twoscomplement14(int16_t num16) {
  // Convert a twos complement int16_t into a twos complement 14 bit number
  int16_t num14 = num16 & 0x3FFF;
  if (num16 & 0x8000) {
    num14 = -((~num14) + 1);
  }
  return num14;
}

int16_t pad14(uint8_t num7_0, uint8_t num7_1) {
  int16_t res = (num7_0 << 7) | (num7_1);
  if (res & 0x2000) return res | 0xC000;
  else return res;
}

uint8_t motor_cmd(uint8_t addr, uint8_t CMD_TYPE, uint16_t data, uint8_t *rx){
  while(Serial2.available()) Serial2.read(); //clear rx buffer

  uint8_t uart2_TX[3] = {0}; //command RS485 to Ø32
  uart2_TX[0] = CMD_TYPE | addr;
  uart2_TX[1] = (data >> 7) & 0b01111111;
  uart2_TX[2] = (data)      & 0b01111111;

  digitalWrite(RS485_DE, HIGH);
  Serial2.write(uart2_TX, 3);
  Serial2.flush();
  digitalWrite(RS485_DE, LOW);
  int numread = Serial2.readBytesUntil(MIN_INT8, rx, 10);
  return numread; 
}

int16_t stick_y_0 = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);      // start serial for RS485 output
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  stick_y_0 = -analogRead(STICK_Y);
  
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop() {

  int16_t stick_y = clip(-analogRead(STICK_Y) - stick_y_0, -511, 511);
  uint16_t power = twoscomplement14(stick_y << 4);
  Serial.print("power: ");
  Serial.println(power);

  motor_cmd(MOT_ADDR, CMD_SET_VOLTAGE, power, uart2_RX);
  float angle = pad14(uart2_RX[3], uart2_RX[4]) * ang_per_ct;
  Serial.print("angle: ");
  Serial.print(angle,4);
  Serial.print("\t\n");

  delay(1);
}
