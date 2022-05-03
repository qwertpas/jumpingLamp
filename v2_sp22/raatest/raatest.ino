
#include <SPI.h>

#define PMUXA2 4
#define PMUXA1 3
#define PMUXA0 2

#define readReg 0x80
#define writeReg 0x00

uint8_t rcvBuffer[2];
uint8_t sendBuffer[2]= {0x00, 0x00};

void setup() {
  pinMode(PMUXA2, OUTPUT); 
  pinMode(PMUXA1, OUTPUT); 
  pinMode(PMUXA0, OUTPUT); 

  SPI.begin();
//  delay(1000);
  // put your setup code here, to run once:
//  regSend(readReg, 0x05, 0b111001100111);
//  Serial.println(sendBuffer[0], BIN);
//  Serial.println(sendBuffer[1], BIN);

}

void loop() {
  // put your main code here, to run repeatedly:
//  for(int i=0; i<=5; i++){
//      printReg(i);
//  }

  
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
////  delay(1);
//  uint8_t ret = SPI.transfer(0b101000000000000);
  Serial.println(SPI.transfer(0b10010100));
  Serial.println(SPI.transfer(0b00000000));
////  delay(1);
  digitalWrite(PMUXA2, HIGH); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
//  uint8_t ret = ;

  delay(50);
}

void printReg(unsigned char address){
  Serial.print(address, HEX);
  Serial.print(": ");
  regSend(readReg, address, 0x0);
  unsigned long ret = (unsigned int)rcvBuffer[0] << 8 | (unsigned int)rcvBuffer[1];
  for(int i=15; i>=0; i--){
    Serial.print(bitRead(ret, i));
    Serial.print(" ");
  }
  Serial.println("");
}


void regSend(unsigned char RW, unsigned char address, unsigned int data){
  sendBuffer[0] = RW | (address << 4) | (data >> 8);
  sendBuffer[1] = data & 0x00FF;

  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
  rcvBuffer[0] = SPI.transfer(sendBuffer[0]);
  rcvBuffer[1] = SPI.transfer(sendBuffer[1]);

  digitalWrite(PMUXA2, HIGH); //selects mux4 which is unused
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
}
