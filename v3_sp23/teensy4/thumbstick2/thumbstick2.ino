#include "HX711.h"
#include "comdef.h"

#include <PWMServo.h>

#define RS485_DE (9)
#define MIN_INT8 (0x80) //most negative int8
#define STICK_X0 A0
#define STICK_Y0 A1
#define STICK_X1 A5
#define STICK_Y1 A6

#define ang_per_ct (90/4096.0)

uint8_t uart2_RX[10] = {0}; //response Ø32 from RS485

PWMServo servo;  // create servo object to control a servo


int16_t clip(int16_t x, int16_t min, int16_t max) {
  if (x > max) {
    return max;
  } else if (x < min) {
    return min;
  } else {
    return x;
  }
}

uint16_t deadband(int16_t x, int16_t deadband) {
  if (abs(x) < deadband){
    return 0;
  }else if(x > 0){
    return x - deadband;
  }else{ //x is negative
    return x + deadband;
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

int16_t stick_x0_init, stick_y0_init, stick_x1_init, stick_y1_init;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);      // start serial for RS485 output
  
  Serial2.setTimeout(1);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);


  servo.attach(12);  // attaches the servo on pin 9 to the servo object
  servo.write(0);              // tell servo to go to position in variable 'pos'


  stick_x0_init = -analogRead(STICK_X0);
  stick_y0_init = -analogRead(STICK_Y0);
  stick_x1_init = -analogRead(STICK_X1);
  stick_y1_init = -analogRead(STICK_Y1);
  
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop() {

  int16_t max_axis = 460;
  int16_t deadband_axis = 20;

  int16_t stick_x0 = deadband(clip(-analogRead(STICK_X0) - stick_x0_init, -max_axis, max_axis), deadband_axis);
  int16_t stick_y0 = deadband(clip(-analogRead(STICK_Y0) - stick_y0_init, -max_axis, max_axis), deadband_axis);
  int16_t stick_x1 = deadband(clip(-analogRead(STICK_X1) - stick_x1_init, -max_axis, max_axis), deadband_axis);
  int16_t stick_y1 = deadband(clip(-analogRead(STICK_Y1) - stick_y1_init, -max_axis, max_axis), deadband_axis);
//  int16_t stick_mag0 = (int16_t) (sqrt(sq(stick_x0) + sq(stick_y0)));
//  int16_t stick_mag1 = (int16_t) (sqrt(sq(stick_x1) + sq(stick_y1)));

  servo.write(stick_x0);

//  motor_cmd(5, CMD_SET_VOLTAGE, twoscomplement14(stick_x1), uart2_RX);
//  motor_cmd(5, CMD_SET_CURRENT, 100, uart2_RX);
//  int16_t ang5 = pad14(uart2_RX[0], uart2_RX[1]);
//  int16_t cur5 = pad14(uart2_RX[2], uart2_RX[3]);
//  Serial.print("ang5: ");
//  Serial.println(ang5);
//  Serial.print("cur5: ");
//  Serial.println(cur5);
  delayMicroseconds(1000);
  
  motor_cmd(6, CMD_SET_VOLTAGE, twoscomplement14(stick_x0), uart2_RX);
  motor_cmd(6, CMD_SET_CURRENT, twoscomplement14(200), uart2_RX);
  int16_t ang6 = pad14(uart2_RX[0], uart2_RX[1]);
  int16_t cur6 = pad14(uart2_RX[2], uart2_RX[3]);
//  Serial.print("ang6: ");
//  Serial.println(ang6);
//  Serial.print("cur6: ");
//  Serial.println(cur6);
//  Serial.print("\t\n");

  
  delayMicroseconds(1000);

  

  for(int i = 0; i< sizeof(uart2_RX); i++){
    Serial.println(uart2_RX[i]);
  }
  Serial.print("\t\n");
  
//  float angle = pad14(uart2_RX[2], uart2_RX[3]) * ang_per_ct;
//  Serial.print("angle: ");
//  Serial.println(angle,4);

//  int16_t a = pad14(uart2_RX[0], uart2_RX[1]);
//  int16_t b = pad14(uart2_RX[2], uart2_RX[3]);
//  int16_t c = pad14(uart2_RX[4], uart2_RX[5]);
//  int16_t d = pad14(uart2_RX[6], uart2_RX[7]);
//
//  Serial.print("D_u: ");
//  Serial.println(uart2_RX[1]);
//  Serial.print("D_v: ");
//  Serial.println(uart2_RX[2]);
//  Serial.print("D_w: ");
//  Serial.println(uart2_RX[3]);
//
//  Serial.print("I_d: ");
//  Serial.println(c);
//  Serial.print("I_q: ");
//  Serial.println(d);


//  int16_t des = pad14(uart2_RX[0], uart2_RX[1]);
//  int16_t ang = pad14(uart2_RX[2], uart2_RX[3]);
//  Serial.print("des: ");
//  Serial.println(des);
//  Serial.print("ang: ");
//  Serial.println(ang);
//  Serial.print("\t\n");

//  delayMicroseconds(1000);
}
