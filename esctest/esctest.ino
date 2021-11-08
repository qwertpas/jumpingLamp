#include <PWMServo.h>


PWMServo ESC;     // create servo object to control the ESC

boolean calibrated = false;

void calibrateESC(){
  ESC.attach(23,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

  Serial.println("180");
  ESC.write(180);
  delay(3000);
  
  Serial.println("0");
  ESC.write(0);
  delay(3000);
  
  Serial.println("90");
  ESC.write(90);
  delay(6500);
}

void setup() {
  Serial.begin(9600);
}

int power = 90;
boolean isIncreasing = true;

void loop() {
  if(!calibrated){
    if(digitalRead(21) == HIGH && !calibrated){
      calibrateESC();
      calibrated = true;
    }
    delay(100);
  }else{

//    if(isIncreasing){
//      power += 1;
//    }else{
//      power -= 1;
//    }
//  
//    if(power >= 135){
//      isIncreasing = false;
//    }else if(power <= 45){
//      isIncreasing = true;
//    }
//    
//    Serial.println(power);
//    ESC.write(power);
//    delay(20);

    ESC.write(135);
    delay(500);
    ESC.write(90);
    delay(500);
    ESC.write(45);
    delay(500);
  }
  
}
