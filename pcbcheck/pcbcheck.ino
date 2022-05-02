#include <SPI.h>

#define readReg 0x80
#define writeReg 0x0
#define STAY 0xffff
#define axis_X      0x0
#define axis_Y      0x1
#define axis_Z      0x2
//------------------Register Address------------------------
#define DEVICE_CONFIG 0x0
#define SENSOR_CONFIG 0x1
#define SYSTEM_CONFIG 0x2
#define ALERT_CONFIG 0x3
#define X_THRX_CONFIG 0x4
#define Y_THRX_CONFIG 0x5
#define Z_THRX_CONFIG 0x6
#define T_THRX_CONFIG 0x7
#define CONV_STATUS 0x8
#define X_CH_RESULT 0x9
#define Y_CH_RESULT 0xA
#define Z_CH_RESULT 0xB
#define TEMP_RESULT 0xC
#define AFE_STATUS 0xD
#define SYS_STATUS 0xE
#define TEST_CONFIG 0xF
#define OSC_MONITOR 0x10
#define MAG_GAIN_CONFIG 0x11
#define ANGLE_RESULT 0x13
#define MAGNITUDE_RESULT 0x14

#define start_DE_CRC      0x0f000400  
#define DeviceConfigData  0b0101000000001000
#define DeviceStart       0b0101000000101000
#define RANGE_50mT 0x0

//------------------DEVICE_CONFIG------------------------
#define CONV_AVG_MASK 0x7000
#define CONV_AVG_1x 0x0
#define CONV_AVG_2x 0x1000
#define CONV_AVG_4x 0x2000
#define CONV_AVG_8x 0x3000
#define CONV_AVG_16x 0x4000
#define CONV_AVG_32x 0x5000

#define MAG_TEMPCO_MASK 0x300
#define MAG_TEMPCO_0pd 0x0
#define MAG_TEMPCO_0R12pd 0x100
#define MAG_TEMPCO_0R2pd 0x300

#define OPERATING_MODE_MASK 0x70
#define OPERATING_MODE_ConfigurationMode 0x0
#define OPERATING_MODE_StandbyMode 0x10
#define OPERATING_MODE_activeMeasureMode 0x20
#define OPERATING_MODE_ActiveTriggerMode 0x30
#define OPERATING_MODE_WakeupAndSleepMode 0x40
#define OPERATING_MODE_SleepMode 0x50
#define OPERATING_MODE_DeepsleepMode 0x60

#define T_CH_EN_TempChannelDisabled 0x0
#define T_CH_EN_TempChannelEnabled 0x8

#define T_RATE_sameAsOtherSensors 0x0
#define T_RATE_oncePerConversionSet 0x4

#define T_HLT_EN_tempLimitCheckOff 0x0
#define T_HLT_EN_tempLimitCheckOn 0x2

#define TEMP_COMP_EN_TempCompensationDisenabled 0x0
#define TEMP_COMP_EN_TempCompensationEnabled 0x1
//------------------SENSOR_CONFIG------------------------
#define ANGLE_EN_NoAngleCalculation 0x0
#define ANGLE_EN_X_Y 0x4000
#define ANGLE_EN_Y_Z 0x8000
#define ANGLE_EN_Z_X 0xC000

#define SLEEPTIME_1ms 0x0
#define SLEEPTIME_5ms 0x400
#define SLEEPTIME_10ms 0x800
#define SLEEPTIME_15ms 0xC00
#define SLEEPTIME_20ms 0x1000
#define SLEEPTIME_30ms 0x1400
#define SLEEPTIME_50ms 0x1800
#define SLEEPTIME_100ms 0x1C00
#define SLEEPTIME_500ms 0x2000
#define SLEEPTIME_1000ms 0x2400

#define MAG_CH_EN_MASK 0x3C0
#define MAG_CH_EN_OFF 0x0
#define MAG_CH_EN_Xenabled 0x40
#define MAG_CH_EN_Yenabled 0x80
#define MAG_CH_EN_XYenabled 0xC0
#define MAG_CH_EN_Zenabled 0x100
#define MAG_CH_EN_ZXenabled 0x140
#define MAG_CH_EN_YZenabled 0x180
#define MAG_CH_EN_XYZenaled 0x1C0
#define MAG_CH_EN_XYXenaled 0x200
#define MAG_CH_EN_YXYenaled 0x240
#define MAG_CH_EN_YZYenaled 0x280
#define MAG_CH_EN_ZYZenaled 0x2C0
#define MAG_CH_EN_ZXZenaled 0x300
#define MAG_CH_EN_XZXenaled 0x340
#define MAG_CH_EN_XYZYXenaled 0x380
#define MAG_CH_EN_XYZZYXenaled 0x3C0

#define Z_RANGE_MASK 0x30
#define Z_RANGE_50mT 0x0
#define Z_RANGE_25mT 0x10
#define Z_RANGE_100mT 0x20

#define Y_RANGE_MASK 0xC
#define Y_RANGE_50mT 0x0
#define Y_RANGE_25mT 0x4
#define Y_RANGE_100mT 0x8

#define X_RANGE_MASK 0x3
#define X_RANGE_50mT 0x0
#define X_RANGE_25mT 0x1
#define X_RANGE_100mT 0x2
//------------------SYSTEM_CONFIG------------------------
#define DIAG_SEL_AllDataPath 0x0
#define DIAG_SEL_enabledDataPath 0x1000
#define DIAG_SEL_enabledDataPathInsequence 0x2000
#define DIAG_SEL_enabledDataPathInsequence 0x3000

#define TRIGGER_MODE_MASK 0x600
#define TRIGGER_MODE_SPI 0x0
#define TRIGGER_MODE_nCSsyncPulse 0x200
#define TRIGGER_MODE_ALERTsyncPulse 0x400

#define DATA_TYPE_32bit 0x0
#define DATA_TYPE_12bit_XY 0x40
#define DATA_TYPE_12bit_XZ 0x80
#define DATA_TYPE_12bit_ZY 0xC0
#define DATA_TYPE_12bit_XT 0x100
#define DATA_TYPE_12bit_YT 0x140
#define DATA_TYPE_12bit_ZT 0x180
#define DATA_TYPE_12bit_AM 0x1C0

#define DIAG_EN_AFEdiagnosticsDisabled 0x0
#define DIAG_EN_ExecutionOftheDiagnosticsSelectedInDEVICE_CFG 0x20

#define Z_HLT_EN_ZaxisLimitCheckoff 0x0
#define Z_HLT_EN_ZaxisLimitCheckon 0x4

#define Y_HLT_EN_YaxisLimitCheckoff 0x0
#define Y_HLT_EN_YaxisLimitCheckon 0x2

#define X_HLT_EN_XaxisLimitCheckoff 0x0
#define X_HLT_EN_XaxisLimitCheckon 0x1
//------------------ALERT_CONFIG------------------------
#define ALERT_LATCH_sourcesNotLatched 0x0
#define ALERT_LATCH_sourcesLatched 0x2000

#define ALERT_MODE_interruptMode 0x0
#define ALERT_MODE_comparatorMode 0x1000

#define STATUS_ALRT_ALERTisNotAsserted 0x0
#define STATUS_ALRT_ALERTisAsserted 0x800

#define RSLT_ALRT_ALERTisNotUsedToSignal 0x0
#define RSLT_ALRT_ALERTisUsedToSignal 0x100

#define THRX_COUNT_1_ConversionResult 0x0
#define THRX_COUNT_2_ConversionResult 0x10
#define THRX_COUNT_3_ConversionResult 0x20
#define THRX_COUNT_4_ConversionResult 0x30

#define T_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define T_THRX_ALRT_ALERTisUsedToSignal 0x8

#define Z_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Z_THRX_ALRT_ALERTisUsedToSignal 0x4

#define Y_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Y_THRX_ALRT_ALERTisUsedToSignal 0x2

#define X_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define X_THRX_ALRT_ALERTisUsedToSignal 0x1
//------------------TEST_CONFIG------------------------
#define CRC_DIS_CRCenabled 0x0
#define CRC_DIS_CRCdisabled 0x4

#define OSC_CNT_CTL_ResetCounters 0x0
#define OSC_CNT_CTL_StartOscCounterdrivenbyHFOSC 0x1
#define OSC_CNT_CTL_StartOscCounterdrivenbyLFOSC 0x2
#define OSC_CNT_CTL_stopCounter 0x3
//------------------MAG_GAIN_CONFIG------------------------
#define GAIN_SELECTION_noAxisSelected 0x0
#define GAIN_SELECTION_XisSelected 0x4000
#define GAIN_SELECTION_YisSelected 0x8000
#define GAIN_SELECTION_ZisSelected 0xC000

//SPI CS pins are controlled by teensy pins 2, 3, 4
//Teensy2-MUXA0, Teensy3-MUXA1, Teensy4-MUXA2
//to enable TMAG1, A2A1A0 = 001

#define PMUXA2 4
#define PMUXA1 3
#define PMUXA0 2


uint8_t rcvBuffer[4];
uint8_t sendBuffer[4]= {0x0F, 0x00, 0x04, 0x00};


void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  SPI.begin();
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, HIGH);


//  SPI.transfer(0x 0F 00 04 07); //writes to register 0x0F to disable CRC checking
  SPI.transfer(writeReg | 0x0F);
  SPI.transfer(0x00);
  SPI.transfer(0x04);
  SPI.transfer(0x07); //CRC checking just this once


  

  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
//
//
//  digitalWrite(PMUXA2, LOW); 
//  digitalWrite(PMUXA1, LOW); 
//  digitalWrite(PMUXA0, HIGH);
    // set to continuous conversion: 
//  SPI.transfer(writeReg | 0x00);
//  SPI.transfer(0x00);
//  SPI.transfer(0x20); //set bit 5 to 1
//  SPI.transfer(0x00);
//  digitalWrite(PMUXA2, LOW); 
//  digitalWrite(PMUXA1, LOW); 
//  digitalWrite(PMUXA0, LOW);

  initTMAG5170_forEval();



//  SPI.transfer(0x0F000407); //writes to register 0x0F to disable CRC checking



// 1110 0000 0000 0000 0000 0000 1000 1011

// 1110 0000 0000 0000 0000 0000 1000 1010


// 1110 0000 0000 0000 0000 0000 1000 1011

// 1110 0000 0000 0000 0100 0000 1000 0011

// 110 0000 0111 1101 1000 0011 1000 0110
//          0111 1101 1000 0011
// 110 0000 0111 1101 1000 0011 1000 0110

//reg 0x00
// 0110 0000 0100 0000 0000 1000 1000 0011

//reg 0x07
// 0100 0000 0110 0111 0011 0010 0000 1001
//           0110 0111 0011 0010

//reg 0x06
// 0100 0000 0111 1101 1000 0011 0000 0000
//           0111 1101 1000 0011

//reg 0x0F
// 0100 0000 0000 0000 0101 0100 0000 1001



}

void loop() {

  Serial.println("");
  Serial.println("------------------- F E D C B A 9 8 7 6 5 4 3 2 1 0 ---------------");
  for(int i=0; i<=0x14; i++){
      printReg(i);
  }

  delay(50);
}

void print32(unsigned long val){
  for(int i=31; i>=0; i--){
    Serial.print(bitRead(val, i));
    Serial.print(" ");
  }
  Serial.println("");
}

void printReg(unsigned char address){
  if(address < 0x10){Serial.print(" ");}
  Serial.print(address, HEX);
  Serial.print(": ");
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, HIGH);
  regConfig(readReg, address, 0x0);
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
  unsigned long ret = (unsigned int)rcvBuffer[0] << 24 | (unsigned int)rcvBuffer[1] << 16 | (unsigned int)rcvBuffer[2] << 8 | (unsigned int)rcvBuffer[3];
  print32(ret);
}


void sndSPI(){
  digitalWrite(PMUXA2, LOW); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, HIGH);
  for (int i = 0; i < 4; i++)
  {
    rcvBuffer[i] = SPI.transfer(sendBuffer[i]);
  }
  digitalWrite(PMUXA2, HIGH); 
  digitalWrite(PMUXA1, LOW); 
  digitalWrite(PMUXA0, LOW);
}

void initTMAG5170_forEval()
{
  unsigned int data;
  for (byte address = 0x0; address < 0x15; address++){
    switch (address){
    case DEVICE_CONFIG:
      data = CONV_AVG_16x | MAG_TEMPCO_0pd | OPERATING_MODE_ConfigurationMode | T_CH_EN_TempChannelEnabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationDisenabled;
      sendBuffer[0] = writeReg | address;
      sendBuffer[1] = (byte)(data >> 8);
      sendBuffer[2] = (byte)(data & 0x00ff);
      sndSPI();
      break;
    case SENSOR_CONFIG:
      data = ANGLE_EN_Y_Z | SLEEPTIME_1ms | MAG_CH_EN_XYZYXenaled | Z_RANGE_25mT | Y_RANGE_25mT | X_RANGE_25mT;
      sendBuffer[0] = writeReg | address;
      sendBuffer[1] = (byte)(data >> 8);
      sendBuffer[2] = (byte)(data & 0x00ff);
      sndSPI();
      break;
    case SYSTEM_CONFIG:
      data = DIAG_SEL_AllDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
      sendBuffer[0] = writeReg | address;
      sendBuffer[1] = (byte)(data >> 8);
      sendBuffer[2] = (byte)(data & 0x00ff);
      sndSPI();
      break;
    case TEST_CONFIG:
      data = CRC_DIS_CRCdisabled;
      sendBuffer[0] = writeReg | address;
      sendBuffer[1] = (byte)(data >> 8);
      sendBuffer[2] = (byte)(data & 0x00ff);
      sndSPI();
      break;
    case MAG_GAIN_CONFIG:
      data = GAIN_SELECTION_noAxisSelected;
      sendBuffer[0] = writeReg | address;
      sendBuffer[1] = (byte)(data >> 8);
      sendBuffer[2] = (byte)(data & 0x00ff);
      sndSPI();
      break;
    default:
      break;
    }
  }
  regConfig(writeReg, DEVICE_CONFIG, DeviceStart);
}

void regConfig(unsigned char RW, unsigned char address, unsigned int data){
  sendBuffer[0] = RW | address;
  sendBuffer[1] = (unsigned char)(data >> 8);
  sendBuffer[2] = (unsigned char)(data & 0x00ff);
  sendBuffer[3] = 0x0F;

  sndSPI();
}
