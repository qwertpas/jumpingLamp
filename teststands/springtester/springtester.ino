#include "HX711.h"
#include "comdef.h"
#include <ADS1256.h>

#define RS485_DE (9)
#define MIN_INT8 (0x80) //most negative int8
#define ENC_ADDR 5
#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN 17

ADS1256 A(4, 0, 5, 10, 2.500); //DRDY, RESET, SYNC(PDWN), CS, VREF(float).    //Teensy 4.0 mine

float N_per_ct = 0.222e-5 * 9.81;
float N_zero = 0;

float ang_per_ct = 90/4096.0;
float ang_zero = 0;

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

uint8_t uart2_RX[5] = {0}; //response Ø32 from RS485

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);      // start serial for RS485 output
//  Serial2.setTimeout(1);
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  float alpha = 0.5;
  for(int samples = 20; samples > 0;){
    if (scale.wait_ready_timeout(100)) {
      float newtons = scale.read() * N_per_ct;
      N_zero = alpha*newtons + (1-alpha)*N_zero;

      motor_cmd(ENC_ADDR, CMD_GET_POSITION, 1, uart2_RX);
      float angle = pad14(uart2_RX[3], uart2_RX[4]) * ang_per_ct;
      ang_zero = alpha*angle + (1-alpha)*ang_zero;
      
      Serial.print("calibrating: ");
      Serial.print(N_zero,4);
      Serial.print(", ");
      Serial.print(angle,4);
      Serial.print(", ");
      Serial.print(ang_zero,4);
      Serial.print("\n\t");
      samples--;
    }
  }
  
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  if (scale.wait_ready_timeout(100)) {
    float newtons = scale.read() * N_per_ct - N_zero;
    Serial.print("force: ");
    Serial.println(newtons,4);

    motor_cmd(ENC_ADDR, CMD_GET_POSITION, 1, uart2_RX);
    float angle = pad14(uart2_RX[3], uart2_RX[4]) * ang_per_ct - ang_zero;
    Serial.print("angle: ");
    Serial.println(angle,4);
    Serial.print("\t");
  }
  
}
