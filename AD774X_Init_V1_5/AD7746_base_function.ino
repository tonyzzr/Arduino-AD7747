//---------------------------------------------------------------------
//all the base function used in AD7746
//---------------------------------------------------------------------

//----------------------------------------------------------------------
//TCA9548 select funtcion
//tca select function 0->7, input i -> seriral port 0 to 7
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
//---------------------------------------------------------------------
//AD774X Reset funtion, command: 0xBF
void AD774X_Reset() {
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(0xBF);
  I2C_State = Wire.endTransmission();
  delay(1);
}
//---------------------------------------------------------------------
// Reads one register AD774X
// Input: RegAdres - registry address
// Output: content of the registry
// Output: I2C_State - returns an error of I2C, 0 = without error
//---------------------------------------------------------------------
uint8_t AD774X_Read_Single_Register(uint8_t RegAdres) {
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  I2C_State = Wire.endTransmission(false);
  Wire.requestFrom(AD774X_ADDRESS, OneByte);
  return Wire.read();
}
//---------------------------------------------------------------------
// Writes one byte to one AD774X register
// Input: RegAdres - registry address
// Input: DataSingl - written data
// Output: I2C_State - returns an error of I2C, 0 = without error
//----------------------------------------------------------------------
void AD774X_Write_Single_Register(uint8_t RegAdres, uint8_t DataSingl) {
  Wire.beginTransmission(AD774X_ADDRESS);
  Wire.write(RegAdres);
  Wire.write(DataSingl);
  I2C_State = Wire.endTransmission();
}
//----------------------------------------------------------------------
//conversion of the obtained data to real values
//----------------------------------------------------------------------
void Read_raw(uint8_t channel, uint8_t channelIndex, uint8_t channelIndex_mask){
    //Serial.println(F(" ,in loop 1 !"));
    uint8_t channel_TCA = channel/2;
    //Serial.println(channel);
   // Serial.println(channel_TCA);
    tcaselect(TCA_address[channel_TCA]);
    Wire.beginTransmission(AD774X_ADDRESS);
    Wire.write(ADR_CAP_DATAH);
    I2C_State = Wire.endTransmission(false);
    Wire.requestFrom(AD774X_ADDRESS, 3);
    for (uint8_t i = 0; i < 3; i++)
    {
      output_total[channel][i] = Wire.read();
    }
    if(channel%2 == 0){
    AD774X_Write_Single_Register(ADR_CAPDACA,B00110111);//code work for length demo, need adjust for real data
    AD774X_Write_Single_Register(ADR_CAP_SETUP,DATA_CAP2_SETUP);//change to channel 2
    AD774X_Write_Single_Register(ADR_EXC_SETUP,DATA_EXC2_SETUP);//change to channel 2
    }
    else{
    AD774X_Write_Single_Register(ADR_CAP_SETUP,DATA_CAP_SETUP);//change to channel 2
    AD774X_Write_Single_Register(ADR_CAPDACA,B10110111);//code work for length demo, need adjust for real data
    AD774X_Write_Single_Register(ADR_EXC_SETUP,DATA_EXC_SETUP);//change to channel 2
    }
    floatOut[channel] =  (float)((long((long)output_total[channel][0] << 16) + ((long)output_total[channel][1] << 8) + (long)output_total[channel][2] - 0x800000) / 2048000.0);
    totalFlag_done = (totalFlag_done | channelIndex);
    totalFlag = totalFlag & channelIndex_mask;
  }