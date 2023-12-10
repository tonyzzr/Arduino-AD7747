#include <Wire.h>
#include "Arduino.h"

// Define all the pin and address here
#define TCAADDR 0x70                                  // TCA9548 address
const uint8_t AD774X_ADDRESS = 0x48;                  // AD774X I2C address
const uint8_t CAP_INTERRUPT_PIN[] = {D2, D3, D4, D5}; // BLE 33 interrput pin
const uint8_t TCA_address[] = {6, 5, 3, 2};           // TCA used i2c channel

//*********************** Settings! ********************************
// AD7746 default registers definition for the differential input configuration
// +- 8.192pF  or for the single-ended input configuration 0 - 8.192pF.
// registers settings for AD7746 here
const uint8_t DATA_CAP_SETUP = B11100000;  // 7  CAPEN+CIN2+CAPDIF+0000+CAPCHOP according, default cin2 single mode, channel 1
const uint8_t DATA_CAP2_SETUP = B10100000; // 7  CAPEN+CIN2+CAPDIF+0000+CAPCHOP according, default cin2 single mode, channel 2
const uint8_t DATA_VT_SETUP = B00000001;   // 8  VTEN+VTMD1+VTMD0+EXTREF+00+VTSHORT+VTCHOP for internal temperature, default disable
const uint8_t DATA_EXC_SETUP = B00001011;  // 9  CLKCTRL+EXCON+EXCB+EXCB^+EXCA+EXCA^+EXCLVL1+EXCLVL0, default EXCA 1/2Vdd
const uint8_t DATA_EXC2_SETUP = B00100011; // 9  CLKCTRL+EXCON+EXCB+EXCB^+EXCA+EXCA^+EXCLVL1+EXCLVL0, default EXCA 1/2Vdd, channel 2
const uint8_t DATA_CFG = B11111000;        // 10 VTF1+VTF0+CAPF2+CAPF1+CAPF0+MD2+MD1+MD0 , VT conversion time 8.2Hz, Cap conversion time 90.9Hz,Idel time
const uint8_t DATA_CAPDACA = B00000000;    // 11 CAPDACA OFF
const uint8_t DATA_CAPDACB = B00000000;    // 12 CAPDACB OFF
const uint8_t DATA_CAP_OFFH = B10000000;   // 13 OFFSET 0x8000 - the middle of the interval (the full range is is approximately +- 1 pF)
const uint8_t DATA_CAP_OFFL = B00000000;   // 14 0x00     "                 "
//******************** Settings end  ********************************
// AD774X Register address Definition
const uint8_t ADR_STATUS = 0;      // Read Only
const uint8_t ADR_CAP_DATAH = 1;   // Read Only
const uint8_t ADR_CAP_DATAM = 2;   // Read Only
const uint8_t ADR_CAP_DATAL = 3;   // Read Only
const uint8_t ADR_VT_DATAH = 4;    // Read Only
const uint8_t ADR_VT_DATAM = 5;    // Read Only
const uint8_t ADR_VT_DATAL = 6;    // Read Only
const uint8_t ADR_CAP_SETUP = 7;   // CAP SETUP REGISTER
const uint8_t ADR_VT_SETUP = 8;    // VT SETUP REGISTER
const uint8_t ADR_EXC_SETUP = 9;   // EXC SETUP REGISTER
const uint8_t ADR_CFG = 10;        // CONFIGURATION REGISTER
const uint8_t ADR_CAPDACA = 11;    // CAP DAC A REGISTER
const uint8_t ADR_CAPDACB = 12;    // CAP DAC B REGISTER
const uint8_t ADR_CAP_OFFH = 13;   // CAP OFFSET CALIBRATION REGISTER HIGH
const uint8_t ADR_CAP_OFFL = 14;   // CAP OFFSET CALIBRATION REGISTER LOW
const uint8_t ADR_CAP_GAINH = 15;  // factory calibration
const uint8_t ADR_CAP_GAINL = 16;  // factory calibration
const uint8_t ADR_VOLT_GAINH = 17; // factory calibration
const uint8_t ADR_VOLT_GAINL = 18; // factory calibration
volatile uint8_t iFlag[4] = {0, 0, 0, 0};
volatile uint8_t totalFlag = B00000000;
volatile float floatOut[8];
uint8_t output_total[8][3];
uint8_t totalFlag_done = B00000000;

//-------------------------------------------------------------------
const uint8_t Channel[8] = {B00000001, B00000010, B00000100, B00001000, B00010000, B00100000, B01000000, B10000000};
const uint8_t Channel_mask[8] = {B11111110, B11111101, B11111011, B11110111, B11101111, B11011111, B10111111, B01111111};
//
const uint8_t MODES = B11111000;      // 0xF8 is "AND" mask for preset convert mode
const uint8_t SINGLE = B00000010;     // 0x02 is "OR" mask for start single convert mode
const uint8_t CONTIN = B00000001;     // 0x01 is "OR" mask for start continual convert mode
const uint8_t CAPDAC_ON = B10000000;  // 0x80 is "OR" mask for CAPDAC ON
const uint8_t CAPDAC_OFF = B00000000; // 0x00 is value for CAPDAC OFF
const uint8_t CAP_RDY = B00000001;    // 0x01 is "AND" mask for CAP READY
const uint8_t REFERENCE = B11101111;  // 0xEF is "AND" mask for unconditionally set internal reference
const uint8_t OneByte = 1;            // auxiliary variables
unsigned long TimeTemp = 0;
unsigned long TimeTempInt = 0;
unsigned long TimeTempOut = 0; // auxiliary variables for timing
volatile uint8_t RTxBuff[20];
volatile uint8_t RTxBuff2[20];                                  // I/O buffer for AD774X registers
uint8_t I2C_State = 0;                                          // status of I2C bus, 0 = without error
unsigned int SamplePeriod = 1;                                  // sample period in [ms]
float C1 = 0, C2 = 0;                                           // auxiliary variables for zero correction calculation
float Capacitance = 0.0, Capacitance2 = 0.0, Temperature = 0.0; // real data
bool EnablePeriodicSampling = false;                            // periodic sampling with output to serial port, is default disabled
// const uint8_t DefaultRegisters[] PROGMEM = {0, 0, 0, 0, 0, 0, 0, DATA_CAP_SETUP, DATA_VT_SETUP, DATA_EXC_SETUP, DATA_CFG,
//                                            DATA_CAPDACA, DATA_CAPDACB, DATA_CAP_OFFH, DATA_CAP_OFFL, 0, 0, 0, 0
//                                         };
//----------------------------------------------------------------------
// declare Arduino reset function at address 0
//----------------------------------------------------------------------
void (*resetFunc)(void) = 0;

// setup
void setup()
{
  // serial start
  Serial.begin(115200);
  while (!Serial)
    ;
  Wire.begin();
  Serial.println(F("\r\nArduino Restart"));
  //--------------------------------------------------------------------
  // set pinout mode
  // SW reset and init AD774X
  //--------------------------------------------------------------------sizeof(CAP_INTERRUPT_PIN)
  for (uint8_t i = 0; i < sizeof(CAP_INTERRUPT_PIN); i++)
  {
    pinMode(CAP_INTERRUPT_PIN[i], INPUT); // define callback funtion pin
    tcaselect(TCA_address[i]);
    AD774X_Reset();

    if (I2C_State != 0)
    {
      Serial.print(TCA_address[i]);
      Serial.println(F(" ,AD774X not responding !"));
    }
    else
    {
      AD774X_Write_Single_Register(ADR_CAP_SETUP, DATA_CAP_SETUP);
      AD774X_Write_Single_Register(ADR_VT_SETUP, DATA_VT_SETUP);
      AD774X_Write_Single_Register(ADR_EXC_SETUP, DATA_EXC_SETUP);
      AD774X_Write_Single_Register(ADR_CFG, DATA_CFG);
      AD774X_Write_Single_Register(ADR_CAPDACA, DATA_CAPDACA);
      AD774X_Write_Single_Register(ADR_CAPDACB, DATA_CAPDACB);
      AD774X_Write_Single_Register(ADR_CAP_OFFH, DATA_CAP_OFFH);
      AD774X_Write_Single_Register(ADR_CAP_OFFL, DATA_CAP_OFFL);
      if (i == 0)
      {
        attachInterrupt(digitalPinToInterrupt(CAP_INTERRUPT_PIN[i]), ReadNewData_1, FALLING);
      }
      if (i == 1)
      {
        attachInterrupt(digitalPinToInterrupt(CAP_INTERRUPT_PIN[i]), ReadNewData_2, FALLING);
      }
      if (i == 2)
      {
        attachInterrupt(digitalPinToInterrupt(CAP_INTERRUPT_PIN[i]), ReadNewData_3, FALLING);
      }
      if (i == 3)
      {
        attachInterrupt(digitalPinToInterrupt(CAP_INTERRUPT_PIN[i]), ReadNewData_4, FALLING);
      }
    }
  }
  tcaselect(TCA_address[0]);
  AD774X_Write_Single_Register(ADR_CFG, (AD774X_Read_Single_Register(ADR_CFG) & MODES) | CONTIN);

  // StartNewConversion();
}
//----------------------------------------------------------------------
void loop()
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (((totalFlag | Channel_mask[i]) & Channel[i]) == Channel[i])
    {
      Read_raw(i, Channel[i], Channel_mask[i]);
    }
  }
 //generate output
  if (totalFlag_done == B00000011)
  {
    for (int8_t j = 0; j < 8; j++)
    {
      Serial.print(floatOut[j]);
      Serial.print(" ");
    }
    Serial.println();
    totalFlag_done = 0;
  }
}
// Interrupt handler for channel 1
void ReadNewData_1()
{
  if (iFlag[0] == 0)
  {
    iFlag[0] = 1;
    totalFlag = (totalFlag | Channel[0]);
  }
  else
  {
    iFlag[0] = 0;
    totalFlag = (totalFlag | Channel[1]);
  }
}
// Interrupt handler for channel 2
void ReadNewData_2()
{
  if (iFlag[1] == 0)
  {
    iFlag[1] = 1;
    totalFlag = (totalFlag | Channel[2]);
  }
  else
  {
    iFlag[1] = 0;
    totalFlag = (totalFlag | Channel[3]);
  }
}
// Interrupt handler for channel 3
void ReadNewData_3()
{
  if (iFlag[2] == 0)
  {
    iFlag[2] = 1;
    totalFlag = (totalFlag | Channel[4]);
  }
  else
  {
    iFlag[2] = 0;
    totalFlag = (totalFlag | Channel[5]);
  }
}
// Interrupt handler for channel 4
void ReadNewData_4()
{
  if (iFlag[3] == 0)
  {
    iFlag[3] = 1;
    totalFlag = (totalFlag | Channel[6]);
  }
  else
  {
    iFlag[3] = 0;
    totalFlag = (totalFlag | Channel[7]);
  }
}
