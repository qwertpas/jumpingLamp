#include <PWMServo.h>


class Motor{
  
  public:
    int port1, port2, port_pwm;
    Motor(int port1_input, int port2_input, int port_pwm_input){
      port1 = port1_input;
      port2 = port2_input;
      port_pwm = port_pwm_input;
      pinMode(port1, OUTPUT);
      pinMode(port2, OUTPUT);
    }
    void setPower(float power){
      if(power <= 0){
        digitalWrite(port1, LOW);
        digitalWrite(port2, HIGH);
      }else{
        digitalWrite(port1, HIGH);
        digitalWrite(port2, LOW);
      }
      analogWrite(port_pwm, round(255*abs(power)));
    }
};

Motor motor = Motor(12, 11, 9);
PWMServo servo;

void setup() {
  Serial.begin(9600);
  
  pinMode(23, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);
  
  servo.attach(8, 500, 2500);
}

float power = 0;
boolean isIncreasing = true;

void loop() {

  if(isIncreasing){
    power += 0.01;
  }else{
    power -= 0.01;
  }

  if(power >= 0.5){
    isIncreasing = false;
  }else if(power <= -0.5){
    isIncreasing = true;
  }

  motor.setPower(power);

  Serial.println(power);

  if(!digitalRead(23)){
    servo.write(0);
    Serial.println(0);

    digitalWrite(13, HIGH);
  }else if(!digitalRead(20)){
    servo.write(180);
    Serial.println(180);

    digitalWrite(13, HIGH);

  }else{
    digitalWrite(13, LOW);
  }

  delay(10);
  

}
