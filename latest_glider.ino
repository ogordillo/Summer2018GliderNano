//Download Nano D2XX drivers here: http://www.ftdichip.com/Drivers/D2XX.htm
//Download Pixy Drivers (install pixymon) here: http://www.cmucam.org/projects/cmucam5/wiki/Latest_release
//Download Grove lib here: https://github.com/Seeed-Studio/Sketchbook_Starter_Kit_for_Arduino
//Download Pixy lib here: http://www.cmucam.org/projects/cmucam5/wiki/Latest_release
//Grove components I2C addresses are here: http://wiki.seeedstudio.com/I2C_And_I2C_Address_of_Seeed_Product/
//Learned how to make bus here: https://www.youtube.com/watch?v=QQLfzlPGjjE

#include <SPI.h>  
#include <Pixy.h>
#include<Wire.h>
#include "rgb_lcd.h"
#include <Servo.h>

#define ADC_REF 5
#define ROTARY_ANGLE_SENSOR A0
#define GROVE_VCC 5 //VCC of the grove interface is normally 5v
#define FULL_ANGLE 300 //full value of the rotary angle is 300 degrees

rgb_lcd lcd;
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

int modecounter = 0; 
// This is the main Pixy object 
Pixy pixy;

int servoLeftPin = 9;
Servo servoLeft;  
int servoLeftAngle = 0;   // servo position in degrees

int servoRightPin = 10;
Servo servoRight;  
int servoRightAngle = 0;   

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ, t_sensor;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  lcd.setRGB(colorR, colorG, colorB);
  
  // Print a message to the LCD.
  lcd.print("hello!");

  pinMode(ROTARY_ANGLE_SENSOR, INPUT);

  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
}

void loop(){

  servoLeft.write(45);      // Turn SG90 servo Left to 45 degrees
  delay(1000);          // Wait 1 second
  servoLeft.write(90);      // Turn SG90 servo back to 90 degrees (center position)
  delay(1000);          // Wait 1 second
  servoLeft.write(135);     // Turn SG90 servo Right to 135 degrees
  delay(1000);          // Wait 1 second
  servoLeft.write(90);      // Turn SG90 servo back to 90 degrees (center position)
  delay(1000);

  servoRight.write(45);      // Turn SG90 servo Left to 45 degrees
  delay(1000);          // Wait 1 second
  servoRight.write(90);      // Turn SG90 servo back to 90 degrees (center position)
  delay(1000);          // Wait 1 second
  servoRight.write(135);     // Turn SG90 servo Right to 135 degrees
  delay(1000);          // Wait 1 second
  servoRight.write(90);      // Turn SG90 servo back to 90 degrees (center position)
  delay(1000);

  
  float voltage;
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
  voltage = (float)sensor_value*ADC_REF/1023;
  float degrees = (voltage*FULL_ANGLE)/GROVE_VCC;
  if(degrees > 0 and degrees < 20)
  {
    modecounter = 0; 
  }
  if(degrees > 20 and degrees < 50)
  {
    modecounter = 1; 
  }
  if(degrees > 50 and degrees < 80)
  {
    modecounter = 2; 
  }
  if(degrees > 80 and degrees < 110)
  {
    modecounter = 3; 
  }

  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%10==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        if(modecounter == 3)
        {
          lcd.setRGB(255, 255, 255);
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Pixy");
          lcd.setCursor(0,2);
          //sprintf(buf, "", pixy.blocks[j]);
          lcd.print(pixy.blocks[j].width); 
          lcd.print(",");
          lcd.print(pixy.blocks[j].height); 
          pixy.blocks[j].print();
        }
      }
    }
 }

 Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  if(modecounter == 0)
  {
    lcd.setRGB(0, 255, 0);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Acceleration");
    lcd.setCursor(0,2);
    lcd.print(AcX);
    lcd.print(",");
    lcd.print(AcY);
    lcd.print(",");
    lcd.print(AcZ);
    delay(500);
  }else
  if(modecounter == 1)
  {
    lcd.setRGB(0, 0, 255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature");
    lcd.setCursor(0,2);
    lcd.print(Tmp/340.00+36.53);
    delay(500);
  }
  else
  if(modecounter == 2)
  {
    lcd.setRGB(255, 0, 255);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gyro");
    lcd.setCursor(0,2);
    lcd.print(GyX);
    lcd.print(",");
    lcd.print(GyY);
    lcd.print(",");
    lcd.print(GyZ);
    delay(500);
  }

}





