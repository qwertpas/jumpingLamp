/************************************************
   Test code for simple two-pin H-bridge driver
   Uses internal pin inversion hardware to generate 
   complementary outputs

   WARNING: this demo code does not implement a dead
            time between forward and reverse!

   mborgerson  9/6/2020
**********************************************************/

const char compileTime [] = "\n\nH-Bridge driver compiled on " __DATE__ " " __TIME__;

//  Setting the invert bit on channel B  only works for a limited set of pin pairs
//  where the two outputs are the A and B channels of the same PWM module.
//  use these if you don't want small phase delays between signals.
//  The possible pairs are:
//  pins 2 and 3    FlexPWM4_2A and FlexPWM4_2B
//  pins 6 and 9    FlexPWM2_2A and FlexPWM2_2B
//  pins 7 and 8    FlexPWM1_3B and FlexPWM1_3A
//  pins 22 and 23  FlexPWM1_3B and FlexPWM1_3A

//  pads 28 and 29  FlexPWM3_1A and FlexPWM3_1B 
//  pin4 and pad 33 FlexPWM2_0A and FlexPWM2_0B

// For the T4.0, pins 34 and 35, 36 and 37, 38 and 39 can also be used but are on
// the bottom and used for the SD card interface.


// For the T4.1, pins 34 to 39 are available as normal pins and there are more pairs on the 
// bottom at pins 42 to 47,  but I'm not sure where they are.


const int leftpin = 6;  // FlexPWM2_2A
const int rightpin = 9; // FlexPWM2_2B

#define INVERTB 0x0200;   // bit pattern to invert channel B

bool forward;

#define PWMFREQUENCY 32000
#define PWMRES 12   // PWM resolution 12 bits = 4096 steps
#define PWMSTEPS 4096  // to match PWMRES
void SetSpeed(bool dir, uint16_t spd) {
  uint16_t hightime;
  // if you use more than 8 bits resolution, you will
  // need to change the type of spd, righthi and lefthi
  //and add some safety  checks here.
  if (dir) {
    hightime = spd;
  } else {
    hightime = PWMSTEPS - spd;
  }

  Serial.printf("\nSpeed is %u ", spd);
  if (forward) Serial.println("forward"); else Serial.println("reverse");
  analogWrite(rightpin, hightime);
  analogWrite(leftpin, hightime);
  // we may not need to do this at every change---
  // but what's a few extra nanoseconds gonna hurt?
  FLEXPWM2_SM2OCTRL |= 0x0200;
  //ShowFlexPWMInit(); // OK, this takes more than a few nanoseconds ;-)
}


// show the values of the output control register
// the channel B output polarity bit is pin 9
void ShowFlexPWMInit(void) {

  Serial.println("\nFlexPWM1:" );
  Serial.printf("  SM0OCTL: %04X ",FLEXPWM1_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ",FLEXPWM1_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ",FLEXPWM1_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n",FLEXPWM1_SM2OCTRL); 
  
  Serial.println("FlexPWM2:" );
  Serial.printf("  SM0OCTL: %04X ",FLEXPWM2_SM0OCTRL); 
  Serial.printf("  SM1OCTL: %04X ",FLEXPWM2_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ",FLEXPWM2_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n",FLEXPWM2_SM3OCTRL);
     
  Serial.println("FlexPWM3:" );
  Serial.printf("  SM0OCTL: %04X ",FLEXPWM3_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ",FLEXPWM3_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ",FLEXPWM3_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n",FLEXPWM3_SM3OCTRL);
  
  Serial.println("FlexPWM4:" );
  Serial.printf("  SM0OCTL: %04X ",FLEXPWM4_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ",FLEXPWM4_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ",FLEXPWM4_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X\n ",FLEXPWM4_SM3OCTRL);
  
}


void setup() {
  Serial.begin(9600);
  delay(100); // wait for PC to respond
  Serial.println(compileTime);
  analogWriteRes(PWMRES);
  Serial.print("\nBefore setting speed");
  ShowFlexPWMInit();
  analogWriteFrequency(leftpin, PWMFREQUENCY);
  analogWriteFrequency(rightpin, PWMFREQUENCY);
  forward = true;
  SetSpeed(forward, PWMSTEPS / 2);
  Serial.print("\nAfter Setting speed.");
  ShowFlexPWMInit();
}

void loop() {
  char ch;
  static uint16_t spd = PWMSTEPS / 2;
  if (Serial.available()) {
    ch = Serial.read();
    if (ch == 'f' ) {
      forward = true;
      SetSpeed(forward, spd); // SetSpeed needed to update waveforms
    }
    if (ch == 'r' ) {
      forward = false;
      SetSpeed(forward, spd); // SetSpeed needed to update waveforms
    }
    // keys '1' to '8' set the speed
    if ((ch > '0') && (ch < '9')) {
      spd = (ch - 48) * 500;  // spd from 500 to 4000
      SetSpeed(forward, spd);
      ShowFlexPWMInit();

    }
  }

}
