#include <Wire.h>
#include <PID_v1.h>

#define ESC1 9
#define ESC2 10

#define I2C_FASTMODE 1
#define I2C_NOINTERRUPT 1

#define gyroAddress  0x69
#define DLPF_FS      0x16
#define SMPLRT_DIV   0x15
#define GYRO_ROLL_H  0x1D
#define GYRO_ROLL_L  0X1E
#define GYRO_PITCH_H 0x1F
#define GYRO_PITCH_L  0x20
#define GYRO_YAW_H   0x21
#define GYRO_YAW_L   0x22
#define TEMP_OUT_H   0x1B
#define TEMP_OUT_L   0x1C

#define DLPF_FS_CONFIG 0x1A // Sets Gyro to +/- 2000deg/sec, Set LPF 98Hz ISR 1kHz
#define SMPLRT_DIV_CONFIG 0x09 // Sets Sampling time to 100Hz

double zeroValue[3] = {0};
double sensitivityGyro = 14.375;
double sensitivityTemp = 280.0;
double offset = 0.0;
double error = -1.0;
unsigned long timer;
double pos = 0.0;
double posNew = 0.0;
double t1 = 0;
double t2 = 0;
int setInput[5] = {0};
double Setpoint = 0;
double Input = 0;
double Output1= 0.0;
double Output2 = 0.0;

// stable values:
// 0.25,0.3,0.05

PID PID1(&Input, &Output1, &Setpoint, 0.25, 0.3, 0.05, DIRECT);
PID PID2(&Input, &Output2, &Setpoint, 0.25, 0.3, 0.05, REVERSE);

void setup() {
//    analogWrite(ESC1,90);
// analogWrite(ESC2,90);
 
  Serial.begin(9600);
  Wire.begin();
 
  i2cWrite(gyroAddress, DLPF_FS, DLPF_FS_CONFIG);
  i2cWrite(gyroAddress, SMPLRT_DIV, SMPLRT_DIV_CONFIG);
  


  while(error < -0.5 || error > 0.5) {
    zeroValue[0] = 0;
    for (long i = 0; i < 100; i++)
    {
      readX();
      readY();
      readZ();
      delay(10);
    }
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[0] += ((double)readX() - offset);
      readY();
      readZ();
    }
  
    offset = offset + (zeroValue[0]/50);
    zeroValue[0] = 0;
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[0] += ((double)readX() - offset);
      readY();
      readZ();
    }
    error = ((zeroValue[0]/50));
  }
  zeroValue[0] = offset;
  error = -1;
  offset = 0.0;
  Serial.println("X calibration complete");
  
  while(error < -0.5 || error > 0.5) {
    zeroValue[1] = 0;
    for (long i = 0; i < 100; i++)
    {
      delay(10);
      readX();
      readY();
      readZ();
    }
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[1] += ((double)readY() - offset);
      readX();
      readZ();
    }
  
    offset = offset + (zeroValue[1]/50);
    zeroValue[1] = 0;
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[1] += ((double)readY() - offset);
      readX();
      readZ();
    }
    error = (zeroValue[1]/50);
  }
  zeroValue[1] = offset;
  error = -1;
  offset = 0.0;
  Serial.println("Y calibration complete");
  
  while(error < -0.5 || error > 0.5) {
    zeroValue[2] = 0;
    for (long i = 0; i < 100; i++)
    {
      delay(10);
      readX();
      readY();
      readZ();
    }
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[2] += ((double)readZ() - offset);
      readY();
      readX();
    }
  
    offset = offset + (zeroValue[2]/50);
    zeroValue[2] = 0;
    for (long i = 0; i < 50; i++)
    {
      delay(10);
      zeroValue[2] += ((double)readZ() - offset);
      readX();
      readY();
    }
    error = (zeroValue[2]/50);
  }
  zeroValue[2] = offset;
  Serial.println("Z calibration complete");
  t1 = millis();

  pinMode(ESC1, OUTPUT);
  pinMode(ESC2, OUTPUT);
  
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
   
  PID1.SetOutputLimits(100,150); 
  PID2.SetOutputLimits(100,150);

  PID1.SetSampleTime(10);
  PID2.SetSampleTime(10);
 
}

void loop() {
  int i = 0;
  int j = 1;
  int k = 0;
  if(Serial.available() > 0) {
    while(Serial.available() > 0) {
      setInput[i] = Serial.read()-48;
      if(setInput[0] == -3) 
          k = 1;
      while((i-j) >= k) {
        setInput[i-j] = setInput[i-j]*10;
        j++;
      }
      i++;
      j=1;
    }
    Setpoint = 0;
    if(setInput[0] == -3) {
      for(int k=1;k<=i;k++) {
        Setpoint = Setpoint - setInput[k];
      }
    }
    else {
      for(int k=0;k<=i;k++) {
        Setpoint = Setpoint + setInput[k];
      }
    }
    while(Setpoint == 67) {
       analogWrite(ESC1,0);
       analogWrite(ESC2,0);
    }
    if(Setpoint > 40 || Setpoint < -40) 
      Setpoint = 0;
    Serial.println(setInput[0]);
    Serial.println(setInput[1]);
    setInput[0] = 0;
    setInput[1] = 0;
    setInput[2] = 0;
    setInput[3] = 0;
  }
  
  
 posNew = ((double)readX()-zeroValue[0])/sensitivityGyro;
 t2 = millis();
 pos = pos + posNew*((t2-t1)/1000.0000000);
 t1 = millis();
 Input = pos;
 
 PID1.Compute();
 PID2.Compute();

 analogWrite(ESC1,Output1);
 analogWrite(ESC2,Output2);
//Serial.print(Output1);
//Serial.print("---");
//Serial.println(Output2);
 Serial.println(pos);


 delay(1);

}

void i2cWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t i2cRead(uint8_t deviceAddress, uint8_t registerAddress)
{
  
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();

  Wire.beginTransmission(deviceAddress);
  Wire.requestFrom(gyroAddress, 0x01);
  uint8_t data = Wire.read();
  Wire.endTransmission();
  
  return data;
  
}

int readX()
{
  
  uint8_t High = 0x00;
  uint8_t Low = 0x00;
  
  High = i2cRead(gyroAddress, GYRO_ROLL_H);
  Low = i2cRead(gyroAddress, GYRO_ROLL_L);

  return High << 8 | Low;
}

int readY()
{
  uint8_t High = 0x00;
  uint8_t Low = 0x00;
  
  High = i2cRead(gyroAddress, GYRO_PITCH_H);
  Low = i2cRead(gyroAddress, GYRO_PITCH_L);

  return High << 8 | Low;
}

int readZ()
{
  uint8_t High = 0x00;
  uint8_t Low = 0x00;
  
  High = i2cRead(gyroAddress, GYRO_YAW_H);
  Low = i2cRead(gyroAddress, GYRO_YAW_L);

  return High << 8 | Low;
}

float readTemps()
{

  uint8_t High = 0x00;
  uint8_t Low = 0x00;
  
  High = i2cRead(gyroAddress, TEMP_OUT_H);
  Low = i2cRead(gyroAddress, TEMP_OUT_L);
  
  float conversion = 35 + ((High << 8 | Low) + 13200) / sensitivityTemp; 

  return conversion;
}



