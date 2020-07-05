
//****************EGR 604 ***********************************
//Project: Follow Me Cooler
//Source Code incorporating GPS, Compass, Bluetooth, Ultrasonic, and Polulu motor driver shield
//Group: F
//David
//Prashant
//Quang
//Prem
//*****************************************************



#include <Wire.h>
#include <Adafruit_Sensor.h> // including libraries for sensors 
#include <Adafruit_LSM303_U.h> // including libraries for compass
#include<math.h> // including libraries for mathematical calculation
#include "DualVNH5019MotorShield.h" //including libraries for polulu motor driver shield
#include <Adafruit_GPS.h> // including libraries for adafruit GPS module
#include <SoftwareSerial.h> //inclusing libraries for serial communication
#include <HCSR04.h> // library for ultrasonic sensor

//---------------------------- characters defined for SensoDuino app communication with HC-05 module-------------------------------------
#define START_CMD_CHAR '>'
#define END_CMD_CHAR '\n'
#define DIV_CMD_CHAR ','
#define DEBUG 1




SoftwareSerial mySerial(3, 2); // GPS TX connected to Arduino pin 3 and RX to Arduino 2



Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345); // compass module initialization

Adafruit_GPS GPS(&mySerial); // GPS serial communication initialization


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

DualVNH5019MotorShield md; // md object initialized for motor driver shield


// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


String inText;
float value0, value1, value2; // values initialized for storing latitude and longitude from mobile app
const int trigPin1 = 13; // trigger pin defined for port 13 in arduino
const int echoPin1 = 11; // echo pin definef for port 11 in arduino
long duration1;
int ult_front;

void setup()  
{

    md.init();  // initialzing motor driver by calling init function with object md
  Serial.begin(9600); // baud rate defined for serial communication

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

   // delay(1000);
  // Ask for firmware version
 // mySerial.println(PMTK_Q_RELEASE);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{

   Serial.println("\nSensoDuino 0.18 by TechBitar.com (2013).\n");
   int inCommand = 0;
  int sensorType = 0;
  unsigned long logCount = 0L;

  char getChar = ' ';  //read serial
  
  // wait for incoming data
  if (Serial.available() < 1) return; // if serial empty, return to loop().

  // parse incoming command start flag
  getChar = Serial.read();
 // Serial.println(getChar);
  if (getChar != START_CMD_CHAR) return; // if no command start flag, return to loop().

  // parse incoming pin# and value 
  sensorType = Serial.parseInt(); // read sensor type
  logCount = Serial.parseInt();  // read total logged sensor readings
  value0 = Serial.parseFloat();  // 1st sensor value which is latitude value from mobile sensoduino app
  value1 = Serial.parseFloat();  // 2rd sensor value which is longitude value from mobile sensoduino app
  
  // send sensoduino readings to serial monitor/terminal

   if (sensorType !=98 && value0>=44 && value0<=42) return;   

  if (DEBUG) {
    Serial.print("Sensor type: ");
    Serial.println(sensorType);
    Serial.print("Sensor log#: ");
    Serial.println(logCount);
    Serial.print("Val[0]: ");
    Serial.println(value0);
    Serial.print("Val[1]: ");
    Serial.println(value1);
    Serial.println("-----------------------");
    //delay(1000);
  }


  sensors_event_t event; // defining events for compass module
  mag.getEvent(&event);
  
  float Pi = 3.14159;
  
  // Calculate the angle of the vector y,x
float  heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
  Serial.println(event.magnetic.y);
  Serial.println(event.magnetic.x);
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading); 
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
//    if (GPSECHO)
//      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 10000) { 
    timer = millis(); // reset the timer
    
   
   
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      
    }
  }

  //Distance calculation for Ultrasonic sensor '1' 
    digitalWrite(trigPin1, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration1 = pulseIn(echoPin1, HIGH);
    // Calculating the distance
    ult_front= duration1*0.034/2;
    // Prints the distance on the Serial Monitor
    Serial.print("ult_front: ");
    Serial.println(ult_front);
   // delay(1000);

 //heading angle calculation
float lat_start=(GPS.latitudeDegrees)* (Pi/180); //start point
float lon_start=(GPS.longitudeDegrees)* (Pi/180); // start point
int  R=6371000; //earth radius
float lat_end=(value0)* (Pi/180); // end point
float lon_end=(value1)* (Pi/180);// end point

float diff= lon_end-lon_start;

float y= sin(diff)*cos(lat_end);
float x= cos(lat_start)*sin(lat_end)-sin(lat_start)*cos(lat_end)*cos(diff);

float angle= (atan2(y,x))* (180/Pi) + 180;
int temp= angle/360;
float angle_new= angle-(temp*360);
//float angle_new= (angle)%360; // angle in degrees
Serial.println("angle is");
Serial.println(angle_new) ;

//distance calculation using haversine formula
float diff2= lat_end-lat_start;
float y1=cos(lat_start)*cos(lat_end)*(sin(diff/2)*sin(diff2/2));
float x1= sin(diff2/2)*sin(diff2/2);
float a= (x1+y1);
float c=2*atan2(sqrt(a), sqrt(1-a));
float d=R*c; // distance between end and start point
Serial.println("distance is");
Serial.println(d);

if (angle_new== heading -30)
{
  if (d >=5)
  {
    if(ult_front<=30)
    {
    Rotate();
    Serial.println("Turning cooler to avoid obstacle");
    }
    else{
    Forward(); // function call
    Serial.println("cooler is moving forward");
    }
  }
  else
  {
    pause(); // function call
  }
}
else
{
  Rotate(); // function call
  Serial.println("Cooler is rotating to find the heading angle");  
}

}

void Forward() // function definition for Forward()
{
  md.setM1Speed(250); // setting the PWM of Motor1 to 250
  md.setM2Speed(250); // setting the PWM of Motor2 to 250
}

void Rotate() // function definition for Rotate()
{
  md.setM1Speed(150); // setting the PWM of Motor1 to 150
  md.setM2Speed(-150); // setting the PWM of Motor2 to -150
}

void pause() // function definition for pause()
{
  md.setM1Speed(0); // setting the PWM of Motor1 to 0
  md.setM2Speed(0); // setting the PWM of Motor2 to 0
}
 put your main code here, to run repeatedly:

}
