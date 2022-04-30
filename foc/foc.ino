#include <Tlv493d.h>
#include "src/SimpleFOC.h"



Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
void initMySensorCallback(){
    Tlv493dMagnetic3DSensor.begin();

}

float readMySensorCallback(){
    Tlv493dMagnetic3DSensor.updateData();
//    Serial.println(Tlv493dMagnetic3DSensor.getY());
    return Tlv493dMagnetic3DSensor.getY();
}


GenericSensor sensor = GenericSensor(readMySensorCallback, initMySensorCallback);
BLDCMotor motor = BLDCMotor(11);
BLDCDriver6PWM driver = BLDCDriver6PWM(3, 4, 5, 6, 7, 8); // (Arduino pins 5,6,10,8)

// commander communication instance
Commander command = Commander(Serial);
void doMotor(char* cmd){ command.motor(&motor, cmd); }


void setup() {
  // initialize sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  FLEXPWM4_SM2VAL4 = 0xFF;


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // subscribe motor to the commander
  command.add('M', doMotor, "motor");

  _delay(1000);

  Serial.println("done setup");
}

void loop() {

//  driver.setPwm(0.5, 0.5, 0.5);

  analogWrite(13, 140);

  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // user communication
  command.run();

  
//  // IMPORTANT - call as frequently as possible
//  // update the sensor values 
//  sensor.update();
//  // display the angle and the angular velocity to the terminal
//  Serial.print(sensor.getAngle());
//  Serial.print("\t");
//  Serial.println(sensor.getVelocity());
}
