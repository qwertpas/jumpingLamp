#include "../hardware_api.h"

#if defined(__arm__) && defined(CORE_TEENSY)

// #define _PWM_FREQUENCY 25000 // 25khz
// #define _PWM_FREQUENCY_MAX 50000 // 50khz

// //  configure High PWM frequency
// void _setHighFrequency(const long freq, const int pin){
//   analogWrite(pin, 0);
//   analogWriteFrequency(pin, freq);
// }


// // function setting the high pwm frequency to the supplied pins
// // - Stepper motor - 2PWM setting
// // - hardware speciffic
// void _configure2PWM(long pwm_frequency, const int pinA, const int pinB) {
//   if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
//   else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
//   _setHighFrequency(pwm_frequency, pinA);
//   _setHighFrequency(pwm_frequency, pinB);
// }

// // function setting the high pwm frequency to the supplied pins
// // - BLDC motor - 3PWM setting
// // - hardware speciffic
// void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
//   if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
//   else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
//   _setHighFrequency(pwm_frequency, pinA);
//   _setHighFrequency(pwm_frequency, pinB);
//   _setHighFrequency(pwm_frequency, pinC);
// }

// // function setting the high pwm frequency to the supplied pins
// // - Stepper motor - 4PWM setting
// // - hardware speciffic
// void _configure4PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC, const int pinD) {
//   if(!pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
//   else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
//   _setHighFrequency(pwm_frequency, pinA);
//   _setHighFrequency(pwm_frequency, pinB);
//   _setHighFrequency(pwm_frequency, pinC);
//   _setHighFrequency(pwm_frequency, pinD);
// }

// // function setting the pwm duty cycle to the hardware
// // - Stepper motor - 2PWM setting
// // - hardware speciffic
// void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
//   // transform duty cycle from [0,1] to [0,255]
//   analogWrite(pinA, 255.0f*dc_a);
//   analogWrite(pinB, 255.0f*dc_b);
// }
// // function setting the pwm duty cycle to the hardware
// // - BLDC motor - 3PWM setting
// // - hardware speciffic
// void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
//   // transform duty cycle from [0,1] to [0,255]
//   analogWrite(pinA, 255.0f*dc_a);
//   analogWrite(pinB, 255.0f*dc_b);
//   analogWrite(pinC, 255.0f*dc_c);
// }

// // function setting the pwm duty cycle to the hardware
// // - Stepper motor - 4PWM setting
// // - hardware speciffic
// void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
//   // transform duty cycle from [0,1] to [0,255]
//   analogWrite(pin1A, 255.0f*dc_1a);
//   analogWrite(pin1B, 255.0f*dc_1b);
//   analogWrite(pin2A, 255.0f*dc_2a);
//   analogWrite(pin2B, 255.0f*dc_2b);
// }

// // 6pwm setting - added by chris
// // - hardware speciffic
// void _writeDutyCycle6PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, float dc_3a, float dc_3b, int pin1A, int pin1B, int pin2A, int pin2B, int pin3A, int pin3B){
//   // transform duty cycle from [0,1] to [0,255]
//   analogWrite(pin1A, 255.0f*dc_1a);
//   analogWrite(pin1B, 255.0f*dc_1b);
//   analogWrite(pin2A, 255.0f*dc_2a);
//   analogWrite(pin2B, 255.0f*dc_2b);
//   analogWrite(pin2A, 255.0f*dc_3a);
//   analogWrite(pin2B, 255.0f*dc_3b);
// }

// set pwm frequency to 32KHz
void _pinHighFrequency(const int pin){
  //  High PWM frequency
  //  https://sites.google.com/site/qeewiki/books/avr-guide/timers-on-the-atmega328
  // if (pin == 5 || pin == 6  ) {
  //     TCCR0A = ((TCCR0A & 0b11111100) | 0x01); // configure the pwm phase-corrected mode
  //     TCCR0B = ((TCCR0B & 0b11110000) | 0x01); // set prescaler to 1
  // }
  // if (pin == 9 || pin == 10 )
  //     TCCR1B = ((TCCR1B & 0b11111000) | 0x01);     // set prescaler to 1
  // if (pin == 3 || pin == 11)
  //     TCCR2B = ((TCCR2B & 0b11111000) | 0x01);// set prescaler to 1

  analogWriteFrequency(pin, 32000); // Teensy 3.0 pin 3 also changes to 32 kHz

}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 2PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure2PWM(long pwm_frequency,const int pinA, const int pinB) {
  _UNUSED(pwm_frequency);
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
}

// function setting the high pwm frequency to the supplied pins
// - BLDC motor - 3PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  _UNUSED(pwm_frequency);
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pinA);
  _pinHighFrequency(pinB);
  _pinHighFrequency(pinC);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 2PWM setting
// - hardware speciffic
void _writeDutyCycle2PWM(float dc_a,  float dc_b, int pinA, int pinB){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0f*dc_a);
  analogWrite(pinB, 255.0f*dc_b);
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
// - hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, int pinA, int pinB, int pinC){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pinA, 255.0f*dc_a);
  analogWrite(pinB, 255.0f*dc_b);
  analogWrite(pinC, 255.0f*dc_c);
}

// function setting the high pwm frequency to the supplied pins
// - Stepper motor - 4PWM setting
// - hardware speciffic
// supports Arudino/ATmega328
void _configure4PWM(long pwm_frequency,const int pin1A, const int pin1B, const int pin2A, const int pin2B) {
   //  High PWM frequency
   // - always max 32kHz
  _pinHighFrequency(pin1A);
  _pinHighFrequency(pin1B);
  _pinHighFrequency(pin2A);
  _pinHighFrequency(pin2B);
}

// function setting the pwm duty cycle to the hardware
// - Stepper motor - 4PWM setting
// - hardware speciffic
void _writeDutyCycle4PWM(float dc_1a,  float dc_1b, float dc_2a, float dc_2b, int pin1A, int pin1B, int pin2A, int pin2B){
  // transform duty cycle from [0,1] to [0,255]
  analogWrite(pin1A, 255.0f*dc_1a);
  analogWrite(pin1B, 255.0f*dc_1b);
  analogWrite(pin2A, 255.0f*dc_2a);
  analogWrite(pin2B, 255.0f*dc_2b);
}




// 6pwm BLDC below





// function configuring pair of high-low side pwm channels, 32khz frequency and center aligned pwm
int _configureComplementaryPair(int pinH, int pinL) {
  if( (pinH == 5 && pinL == 6 ) || (pinH == 6 && pinL == 5 ) ){
    // configure the pwm phase-corrected mode
    TCCR0A = ((TCCR0A & 0b11111100) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 6 ) TCCR0A = 0b10110000 | (TCCR0A & 0b00001111) ;
    else TCCR0A = 0b11100000 | (TCCR0A & 0b00001111) ;
    // set prescaler to 1 - 32kHz freq
    TCCR0B = ((TCCR0B & 0b11110000) | 0x01);
  }else if( (pinH == 9 && pinL == 10 ) || (pinH == 10 && pinL == 9 ) ){
    // set prescaler to 1 - 32kHz freq
    TCCR1B = ((TCCR1B & 0b11111000) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 9 ) TCCR1A = 0b10110000 | (TCCR1A & 0b00001111) ;
    else TCCR1A = 0b11100000 | (TCCR1A & 0b00001111) ;
  }else if((pinH == 3 && pinL == 11 ) || (pinH == 11 && pinL == 3 ) ){
    // set prescaler to 1 - 32kHz freq
    TCCR2B = ((TCCR2B & 0b11111000) | 0x01);
    // configure complementary pwm on low side
    if(pinH == 11 ) TCCR2A = 0b10110000 | (TCCR2A & 0b00001111) ;
    else TCCR2A = 0b11100000 | (TCCR2A & 0b00001111) ;
  }else{
    return -1;
  }
  return 0;
}

// Configuring PWM frequency, resolution and alignment
// - BLDC driver - 6PWM setting
// - hardware specific - this one written by chris for teensy4.0
]int _configure6PWM(long pwm_frequency, float dead_zone, const int pinA_h, const int pinA_l,  const int pinB_h, const int pinB_l, const int pinC_h, const int pinC_l) {
  _UNUSED(pwm_frequency);
  _UNUSED(dead_zone);
  //  High PWM frequency
  // - always max 32kHz
  int ret_flag = 0;
  ret_flag += _configureComplementaryPair(pinA_h, pinA_l);
  ret_flag += _configureComplementaryPair(pinB_h, pinB_l);
  ret_flag += _configureComplementaryPair(pinC_h, pinC_l);
  return ret_flag; // returns -1 if not well configured
}




// function setting the pwm duty cycle
void _setPwmPair(int pinH, int pinL, float val, int dead_time)
{
  int pwm_h = _constrain(val - dead_time/2, 0, 255);
  int pwm_l = _constrain(val + dead_time/2, 0, 255);

  analogWrite(pinH, pwm_h);
  if(pwm_l == 255 || pwm_l == 0)
    digitalWrite(pinL, pwm_l ? LOW : HIGH);
  else
    analogWrite(pinL, pwm_l);
}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
//  - BLDC driver - 6PWM setting
//  - hardware specific
// supports Arudino/ATmega328 - not anymore heheheheh
void _writeDutyCycle6PWM(float dc_a,  float dc_b, float dc_c, float dead_zone, int pinA_h, int pinA_l, int pinB_h, int pinB_l, int pinC_h, int pinC_l){
  _setPwmPair(pinA_h, pinA_l, dc_a*255.0, dead_zone*255.0);
  _setPwmPair(pinB_h, pinB_l, dc_b*255.0, dead_zone*255.0);
  _setPwmPair(pinC_h, pinC_l, dc_c*255.0, dead_zone*255.0);
}

#endif