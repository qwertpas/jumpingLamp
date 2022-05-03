//  Setting the invert bit on channel B  only works for a limited set of pin pairs
//  where the two outputs are the A and B channels of the same PWM module.
//  use these if you don't want small phase delays between signals.
//  The possible pairs are:
//  pins 2 and 3    FlexPWM4_2A and FlexPWM4_2B
//  pins 6 and 9    FlexPWM2_2A and FlexPWM2_2B
//  pins 7 and 8    FlexPWM1_3B and FlexPWM1_3A
//  pins 22 and 23  FlexPWM1_3B and FlexPWM1_3A

const int V1pin = 2; // FlexPWM4_2A
const int G1pin = 3; // FlexPWM4_2B
const int V2pin = 6; // FlexPWM2_2A
const int G2pin = 9; // FlexPWM2_2B
const int V3pin = 8; // FlexPWM1_3A
const int G3pin = 7; // FlexPWM1_3B

#define INVERTB 0x0200; // bit pattern to invert channel B

bool forward;

#define PWMFREQUENCY 32000
#define PWMRES 12     // PWM resolution 12 bits = 4096 steps
#define PWMSTEPS 4096 // to match PWMRES
void SetSpeed(bool dir, uint16_t spd)
{
  uint16_t hightime;
  // if you use more than 8 bits resolution, you will
  // need to change the type of spd, righthi and lefthi
  // and add some safety  checks here.
  if (dir)
  {
    hightime = spd;
  }
  else
  {
    hightime = PWMSTEPS - spd;
  }

  Serial.printf("\nSpeed is %u ", spd);
  if (forward)
    Serial.println("forward");
  else
    Serial.println("reverse");
//  analogWrite(rightpin, hightime);
//  analogWrite(leftpin, hightime);
  // we may not need to do this at every change---
  // but what's a few extra nanoseconds gonna hurt?
  FLEXPWM2_SM2OCTRL |= 0x0200;
  // ShowFlexPWMInit(); // OK, this takes more than a few nanoseconds ;-)
}

// show the values of the output control register
// the channel B output polarity bit is pin 9
void ShowFlexPWMInit(void)
{

  Serial.println("\nFlexPWM1:");
  Serial.printf("  SM0OCTL: %04X ", FLEXPWM1_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ", FLEXPWM1_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ", FLEXPWM1_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n", FLEXPWM1_SM2OCTRL);

  Serial.println("FlexPWM2:");
  Serial.printf("  SM0OCTL: %04X ", FLEXPWM2_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ", FLEXPWM2_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ", FLEXPWM2_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n", FLEXPWM2_SM3OCTRL);

  Serial.println("FlexPWM3:");
  Serial.printf("  SM0OCTL: %04X ", FLEXPWM3_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ", FLEXPWM3_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ", FLEXPWM3_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X \n", FLEXPWM3_SM3OCTRL);

  Serial.println("FlexPWM4:");
  Serial.printf("  SM0OCTL: %04X ", FLEXPWM4_SM0OCTRL);
  Serial.printf("  SM1OCTL: %04X ", FLEXPWM4_SM1OCTRL);
  Serial.printf("  SM2OCTL: %04X ", FLEXPWM4_SM2OCTRL);
  Serial.printf("  SM3OCTL: %04X\n ", FLEXPWM4_SM3OCTRL);
}

void setup()
{
  Serial.begin(9600);
  delay(100); // wait for PC to respond
//  analogWriteRes(PWMRES);
  Serial.print("\nBefore setting speed");
//  ShowFlexPWMInit();
//  analogWriteFrequency(V1pin, PWMFREQUENCY);
//  analogWriteFrequency(G1pin, PWMFREQUENCY);
//  analogWriteFrequency(V2pin, PWMFREQUENCY);
//  analogWriteFrequency(G2pin, PWMFREQUENCY);
//  analogWriteFrequency(V3pin, PWMFREQUENCY);
//  analogWriteFrequency(G3pin, PWMFREQUENCY);
  forward = true;
  Serial.print("\nAfter Setting speed.");
  ShowFlexPWMInit();

  digitalWrite(13, HIGH);

  pinMode(V1pin, OUTPUT);
  pinMode(G1pin, OUTPUT);
  pinMode(V2pin, OUTPUT);
  pinMode(G2pin, OUTPUT);
  pinMode(V3pin, OUTPUT);
  pinMode(G3pin, OUTPUT);
}

int com_step = 0;


void loop()
{
  char ch;
  static uint16_t spd = PWMSTEPS / 2;
  if (Serial.available())
  {
    ch = Serial.read();
    if (ch == 'f')
    {
      forward = true;
//      SetSpeed(forward, spd); // SetSpeed needed to update waveforms
    }
    if (ch == 'r')
    {
      forward = false;
//      SetSpeed(forward, spd); // SetSpeed needed to update waveforms
    }
    // keys '1' to '8' set the speed
    if ((ch > '0') && (ch < '9'))
    {
      spd = (ch - 48) * 500; // spd from 500 to 4000
//      SetSpeed(forward, spd);
      ShowFlexPWMInit();
    }
  }




  switch (com_step)
  {
  case 0:
    // V1, G3
    digitalWrite(V1pin, HIGH);
    digitalWrite(G1pin, LOW);
    digitalWrite(V2pin, LOW);
    digitalWrite(G2pin, LOW);
    digitalWrite(V3pin, LOW);
    digitalWrite(G3pin, HIGH);
    break;
  case 1:
    // V1, G2
    digitalWrite(V1pin, HIGH);
    digitalWrite(G1pin, LOW);
    digitalWrite(V2pin, LOW);
    digitalWrite(G2pin, HIGH);
    digitalWrite(V3pin, LOW);
    digitalWrite(G3pin, LOW);
    break;
  case 2:
    // V3, G2
    digitalWrite(V1pin, LOW);
    digitalWrite(G1pin, LOW);
    digitalWrite(V2pin, LOW);
    digitalWrite(G2pin, HIGH);
    digitalWrite(V3pin, HIGH);
    digitalWrite(G3pin, LOW);
    break;
  case 3:
    // V3, G1
    digitalWrite(V1pin, LOW);
    digitalWrite(G1pin, HIGH);
    digitalWrite(V2pin, LOW);
    digitalWrite(G2pin, LOW);
    digitalWrite(V3pin, HIGH);
    digitalWrite(G3pin, LOW);
    break;
  case 4:
    // V2, G1
    digitalWrite(V1pin, LOW);
    digitalWrite(G1pin, HIGH);
    digitalWrite(V2pin, HIGH);
    digitalWrite(G2pin, LOW);
    digitalWrite(V3pin, LOW);
    digitalWrite(G3pin, LOW);
    break;
  case 5:
    // V2, G3
    digitalWrite(V1pin, LOW);
    digitalWrite(G1pin, LOW);
    digitalWrite(V2pin, HIGH);
    digitalWrite(G2pin, LOW);
    digitalWrite(V3pin, LOW);
    digitalWrite(G3pin, HIGH);
    break;
  }

  com_step++;
  if(com_step == 6) com_step=0;

  delay(10);

  Serial.println(com_step);
}
