#include <PWMServo.h>
#include<Wire.h>

#define Gyr_Gain 0.00763358

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

      //max is +- 1
      if(power > 1){power = 1;}
      if(power < -1){power = -1;}
      
      analogWrite(port_pwm, round(255*abs(power)));
    }
};

float power = 0;
boolean isIncreasing = true;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t ax,ay,az,tmp,gx,gy,gz;
float AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ;
float mixX, mixY, mixYa;

Motor motor = Motor(7, 8, 9);
PWMServo servo;
PWMServo esc;
//esc pwm port is 11

boolean isJumping = false;
boolean isPressed20 = false;

float targetAng = -39;
int servoPos = 90;


void setup() {
  Serial.begin(9600);
  
  pinMode(23, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  
  servo.attach(12, 500, 2500);
  esc.attach(11, 1000, 2000);

  Wire.begin(); //gyro setup
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

}

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long looptime = 10;        // time constant for timer

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= looptime) {  // start timed event
    previousMillis = currentMillis;
    
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    AccelX = ax;
    AccelY = ay;
    AccelZ = az;
    GyroX = Gyr_Gain * (gx);
    GyroY = Gyr_Gain * (gy)*-1;
    GyroZ = Gyr_Gain * (gz);
  
    AccelY = (atan2(AccelY, AccelZ) * 180 / PI);
    AccelX = (atan2(AccelX, AccelZ) * 180 / PI);
  
    float K = 0.8;
    float dt = looptime * 0.001;
    float A = K / (K + dt);
  
    mixX = A *(mixX+GyroX*dt) + (1-A)*AccelY;    
    mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX;
  
    mixYa = mixY + 3.3;
  
//    Serial.print(mixYa, 2);        // roll
//    Serial.print(",");
    Serial.print(mixX, 2);      // pitch 
    Serial.print("\t");
    Serial.print(targetAng, 2);
    Serial.println("\t");

    
    float error = (mixX - targetAng);

    if(error > 0.5){
      power += error * (1/20.0) + 0.4;
    }else if(error < -0.5){
      power += error * (1/20.0) - 0.4;
    }else{
      power += 0;
    }

    if(abs(error) > 2){
      power += 0;
    }

    if(power > 1) power = 1;
    if(power < -1) power = -1;
  
    if(!digitalRead(23)){

      targetAng = mixX;
  
      digitalWrite(13, LOW);
    }
    if(!digitalRead(20)){

      if(!isPressed20){
        isJumping = !isJumping;
        Serial.println("toggle jumping");
      }
  
      digitalWrite(13, LOW);
      isPressed20 = true;
    }else{
      isPressed20 = false;
    }
    if(digitalRead(23) && digitalRead(20)){
      digitalWrite(13, HIGH);
    }

    if(isJumping){
      if(currentMillis % 500 < 250){
        servoPos = 180;
      }else{
        servoPos = 0;
      }

      esc.write(45);
    }else{
      esc.write(135);
    }


    motor.setPower(power);
    servo.write(servoPos);
//    Serial.println(power);

    
  }
  

}
