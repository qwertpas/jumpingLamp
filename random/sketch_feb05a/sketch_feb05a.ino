#include <Tlv493d.h>
#include <PWMServo.h>


// Tlv493d Opject
Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();

PWMServo esc;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  Tlv493dMagnetic3DSensor.begin();
  Tlv493dMagnetic3DSensor.setAccessMode(Tlv493dMagnetic3DSensor.MASTERCONTROLLEDMODE);
  Tlv493dMagnetic3DSensor.disableTemp();

  esc.attach(12, 1000, 2000);
  esc.write(90);
}

void loop() {
  delay(Tlv493dMagnetic3DSensor.getMeasurementDelay());
  Tlv493dMagnetic3DSensor.updateData();

  esc.write(0);

//  Serial.print(Tlv493dMagnetic3DSensor.getAmount());
//  Serial.print(" ; ");
//  Serial.print(Tlv493dMagnetic3DSensor.getAzimuth());
//  Serial.print(" ; ");
  Serial.println(180/3.14 * Tlv493dMagnetic3DSensor.getPolar());
}
