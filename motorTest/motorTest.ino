

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


Motor motor = Motor(15, 16, 14);
void setup() {
  Serial.begin(9600);
}

float power = 0;
boolean isIncreasing = true;

void loop() {

  

  if(isIncreasing){
    power += 0.01;
  }else{
    power -= 0.01;
  }

  if(power >= 0.3){
    isIncreasing = false;
  }else if(power <= -0.3){
    isIncreasing = true;
  }

  motor.setPower(power);

  Serial.println(power);

  delay(10);
  

}
