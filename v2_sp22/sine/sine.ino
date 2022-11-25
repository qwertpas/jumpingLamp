
#include <SPI.h>

#define PMUXA2 4
#define PMUXA1 3
#define PMUXA0 2

//the following are for FPC 1 (motor 1)
#define PWMA 10
#define PWMB 9
#define PWMC 8

#define CSOA 14     //current sense analog in
#define SNS_SEL1 19 //current sense mux selection
#define SNS_SEL2 20 //current sense mux selection

#define READ  0x8000  //1 in MSB
#define WRITE 0x0000  //0 in MSB

//16bit SPI, buf used for both send and receive
uint16_t buf = 0;

void selectRAA(){
  //selects MUX0 which goes to CS_DRV1
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
}

void deselectRAA(){
  //selects MUX4 which is unused
  digitalWrite(PMUXA2, HIGH); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
}

uint16_t regRead(unsigned char reg_addr){
  //bit 15: 1 for read, bits 14-12: register address, bits 11-0: don't care
  buf = READ | ((reg_addr & 0b111) << 12);
  selectRAA();
  buf = SPI.transfer16(buf);
  deselectRAA();
  return buf;
}

void regWrite(unsigned char reg_addr, uint16_t data){
  //bit 15: 0 for write, bits 14-12: register address, bits 11-0: data
  buf = WRITE | ((reg_addr & 0b111) << 12) | (data & 0x0FFF);
  selectRAA();
  buf = SPI.transfer16(buf);
  deselectRAA();
}

void regPrint(unsigned char reg_addr){
  Serial.print("Reg ");
  Serial.print(reg_addr, HEX);
  Serial.print(": \n");
  
  int16_t ret = regRead(reg_addr);
  for(int i=15; i>=0; i--){
    Serial.print(bitRead(ret, i));
    if(i % 4 == 0){
      Serial.print(" ");
    }
  }
  Serial.print("\n");
}




/*
Config registers default:
Reg 0x2: 0011 1010 0100
Reg 0x3: 0010 0101 0000
Reg 0x4: 1011 0000 0010
Reg 0x5: 0111 1111 1100

We want 3 phase PWM mode so each phase is only controlled by 1 pin (LIx)
PWM_MODE = 1b;

Current sense mode 5:
DT/IFSEL/BEN = tied to VCC (hardwired)
BEMF_EN = 0b;
CS_MODE = 0b;
CS_SH_EN = 1b;

Clear fault:
CLR_FLT = 1b; 

Config registers with the settings:
Reg 0x2: 1011 1011 0101
Reg 0x3: 0010 0101 0000
Reg 0x4: 1011 0000 0010
Reg 0x5: 0111 1111 1100
*/

void initRAA(){
  regWrite(0x2, 0b101110111101); //clear fault (warning can spike voltage)
//  regWrite(0x2, 0b001110110100); //only change to pwm mode and clear fault
  
  regWrite(0x3, 0b101001010000);
  regWrite(0x4, 0b101100000010);
  regWrite(0x5, 0b010000000000);
}

void disableMotors(){
//  regWrite(0x2, 0b001110101101); //enter HI/LO mode instead of PWM mode

  //force all phases 0V
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  analogWrite(PWMC, 0);
}

void setup() {
  pinMode(PMUXA2, OUTPUT); 
  pinMode(PMUXA1, OUTPUT); 
  pinMode(PMUXA0, OUTPUT); 

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(PWMC, OUTPUT);

  analogWriteFrequency(PWMA, 32226); //Teensy 4.0 at 396MHz https://www.pjrc.com/teensy/td_pulse.html
  analogWriteFrequency(PWMB, 32226); //this is above human hearing so it ok, small motors switch around this range
  analogWriteFrequency(PWMC, 32226);
  analogWriteResolution(12); // analogWrite value 0 to 4095, or 4096 for high

  pinMode(CSOA, INPUT);
  pinMode(SNS_SEL1, OUTPUT);
  pinMode(SNS_SEL2, OUTPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1400000, MSBFIRST, SPI_MODE1));

//  analogWrite(PWMA, 4096); //right now PHA is shorted to VM so don't switch low side at all

  delay(100);
  
  
  initRAA();

  while(!Serial.available()){ //wait until next input
    for(int i=0; i < 6; i++){
      regPrint(i);
    }
    delay(10);
  }
}

//lookup table with 48 values from 0 to 4095
int sine48[] = {
  2048,2315,2577,2831,3071,3294,3495,3672,
  3821,3939,4025,4077,4095,4077,4025,3939,
  3821,3672,3495,3294,3071,2831,2577,2315,
  2048,1780,1518,1264,1024,801,600,423,
  274,156,70,18,0,18,70,156,
  274,423,600,801,1024,1264,1518,1780
};

int step = 0;
float power = 2/7.4;

void loop() {

  //type anything in serial monitor to toggle pause/resume
  if (Serial.available()) {
    disableMotors();
    Serial.println("pause");
    
    while(Serial.available()){ //clear buffer
      Serial.read();
    }
    while(!Serial.available()){ //wait until next input
      for(int i=0; i < 6; i++){
        regPrint(i);
      }
      delay(10);
    }
    while(Serial.available()){ //clear buffer
      Serial.read();
    }
    Serial.println("resume");
  }
  
//  initRAA();

//  regWrite(0x2, 0b001110111101);
//  regWrite(0x3, 0b101001010000);
//  regWrite(0x4, 0b101100000010);
//  regWrite(0x5, 0b010000000000);

//  for(int i=0; i < 6; i++){
//    regPrint(i);
//  }
//
//
  int valA = power * sine48[step];
  int valB = power * sine48[(step + 16)%48];
  int valC = power * sine48[(step + 32)%48];

  step = (step + 1)%48;
  
  analogWrite(PWMA, valA+50);
  analogWrite(PWMB, valB+50);
  analogWrite(PWMC, valC+50);

//  analogWrite(PWMB, 1000);
//  analogWrite(PWMC, 0);

//  Serial.print(", valA: ");
//  Serial.print(valA);
//  Serial.print(", valB: ");
//  Serial.print(valB);
//  Serial.print(", valC: ");
//  Serial.println(valC);

  



//  delay(10);




  digitalWrite(SNS_SEL1, LOW);
  digitalWrite(SNS_SEL2, HIGH);
  delay(2);
  Serial.print("A: ");
  Serial.print(analogRead(CSOA));
  Serial.print(", ");


  digitalWrite(SNS_SEL1, HIGH);
  digitalWrite(SNS_SEL2, LOW);
  delay(4);
  Serial.print("B: ");
  Serial.print(analogRead(CSOA));
  Serial.print(", ");

  digitalWrite(SNS_SEL1, HIGH);
  digitalWrite(SNS_SEL2, HIGH);
  delay(6);
  Serial.print("C: ");
  Serial.print(analogRead(CSOA));
  Serial.print(", ");

  Serial.println("\n");

  
  
}
