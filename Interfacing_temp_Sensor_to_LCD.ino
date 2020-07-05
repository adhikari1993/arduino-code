#include <Wire.h>
void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);             // wait for serial monitor
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
  Serial.println("No I2C devices found\n");
 else
   Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

Source code for thermocouple

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "max6675.h"

//Declare variable 
int ktcSO = 8;
int ktcCS = 9;
int ktcCLK = 10;
 
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3 ,POSITIVE);
MAX6675 ktc(ktcCLK, ktcCS, ktcSO);
 
   
void setup() {
 
  lcd.begin(16,2);
  lcd.clear();
  Serial.begin(9600);
  // give the MAX a little time to settle
  delay(500);
}
 
void loop() {
  // basic readout 
   
   lcd.setCursor(1,0);
   lcd.write("Deg F = ");//print the phase
   lcd.print(ktc.readFahrenheit());//print the temperature
   //Serial.println(ktc.readFahrenheit()); 
   delay(2500);
}

