#include <PWMServo.h>


PWMServo ESC;     // create servo object to control the ESC

boolean calibrated = false;

void calibrateESC(){

//  Serial.println("90");
//  ESC.write(90);
//  delay(2000);
  
  Serial.println("180");
  ESC.write(180);
  digitalWrite(13, HIGH);
  delay(6000);
  digitalWrite(13, LOW);
  
  Serial.println("0");
  ESC.write(0);
  delay(3000);
  
  Serial.println("90");
  ESC.write(90);
  delay(6500);
}

void setup() {
//  pinMode(8, INPUT_PULLUP);
//  pinMode(11, INPUT_PULLUP);
  Serial.begin(9600);

  ESC.attach(11, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  ESC.write(90);

  delay(2000);
}

int dir = 1;
int pos = 90;

void loop() {
  if(!calibrated){
//    calibrateESC();
    calibrated = true;
    delay(100);
  }else{

      pos += dir;
      if(pos == 0 || pos == 180){
        dir *= -1;
      }
      ESC.write(pos);
      Serial.println(pos);

//    if(!digitalRead(8)){
//      ESC.write(170);
//      Serial.println(170);
//    }else if(!digitalRead(11)){
//      ESC.write(10);
//      Serial.println(10);
//    }else{
//      ESC.write(90);
//      Serial.println(90);
//    }


    
  }

  delay(100);
  
}
