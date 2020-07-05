#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <HCSR04.h>
#define IRFL 5
#define IRFR 6
#define IRBL 3
#define IRBR 2


    
   
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

 int IRFL_status=0;
 int IRFR_status=0;
 int IRBL_status=0;
 int IRBR_status=0;
 const int trigPin1 = 12;
 const int echoPin1 = 11;
 const int trigPin2 = 8;
 const int echoPin2 = 7;
 long duration1;
 int ult_front;
 long duration2;
 int ult_back;
 float diff=0;
 
    
   void setup ()
{ 
 
  AFMS.begin(); 
  myMotor1->setSpeed(255);
  myMotor2->setSpeed(255);
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
  pinMode(IRFL_status,INPUT);
  pinMode(IRFR_status,INPUT);
  pinMode(IRBL_status,INPUT);
  pinMode(IRBR_status,INPUT);
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
    // Reading from all 4 IR sensors
     
     IRFR_status=digitalRead(IRFR);
     IRFL_status=digitalRead(IRFL);
     IRBR_status=digitalRead(IRBR);
     IRBL_status=digitalRead(IRBL);
     Serial.print("StatusfrontleftIR::");
     Serial.println(IRFL_status);
     Serial.print("StatusfrontrightIR::");
     Serial.println(IRFR_status);
     Serial.print("StatusbackleftIR::");
     Serial.println(IRBL_status);
     Serial.print("StatusbackrightIR::");
     Serial.println(IRBR_status);
    // delay(1000);
     
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

 //Distance calculation for Ultrasonic sensor '2' 
    digitalWrite(trigPin2, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin2, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration2 = pulseIn(echoPin2, HIGH);
    // Calculating the distance
    ult_back= duration2*0.034/2;
    // Prints the distance on the Serial Monitor
     Serial.print("ult_back: ");
     Serial.println(ult_back);
    //delay(1000);


diff= ult_front-ult_back;
if(ult_front<50 || ult_back<50)
{
if (diff<=0)
{
if(ult_front<=40)
{
  front_ultrasonic();
}
else if(ult_front>40)
{
  front_ultrasonic2();
}
}
else if(diff >0)
{

 if(ult_back<=40)
{
  back_ultrasonic();
}
else if(ult_back>40)
{
  back_ultrasonic2();
}

}
}

else
{
  Serial.println("Scanning");
  Scanning();
}
}
  



 void front_ultrasonic()
 {
    // For Front Ultrasonic detection >>obstacle less than or equal to 40

        if(IRFR_status == 1 && IRFL_status==1)
        {
          Serial.println("FORWARD");
          Forward();
          //delay(50);
         
        }
        else if (IRFR_status == 0 && IRFL_status==1)
        {
          Serial.println("Turn Left");
          //pause();
          // delay(100);
          Reverse();
          delay(500);
          Left();
          delay(500);
          
          
        }
        else if (IRFR_status == 1 && IRFL_status==0)
        {
          Serial.println("Turn Right");
          //pause();
          //delay(100);
          Reverse();
          delay(500);
          Right();
          delay(500);
          
        }
        else 
        {
          Serial.println("Reverse");
          //pause();
          //delay(100);
          Reverse();
          delay(1000);
          
        }
    }

   void front_ultrasonic2()
   {
   
      if(IRFR_status == 1 && IRFL_status==1)
          {
          Serial.println("Scanning");
          Scanning();
         // delay(50);
          }
        else if (IRFR_status == 0 && IRFL_status==1)
        {
          Serial.println("Turn Right");
          //pause();
         // delay(100);
           Reverse();
          delay(500);
          Left();
          delay(500);
          
        }
        else if (IRFR_status == 1 && IRFL_status==0)
        {
          Serial.println("Turn Right");
          //pause();
         // delay(100);
          Reverse();
          delay(500);
          Right();
          delay(500);
          
        }
        else 
        {
          Serial.println("Reverse");
          //pause();
          //delay(100);
          Reverse();
          delay(1000);
          
        }
         
   }
   

 void back_ultrasonic()
 {
 // For back ultrasonic detection obstacle less than or equal to 40

        if(IRBR_status == 1 && IRBL_status==1)
        {
          Serial.println("Reverse");
          Reverse();
          //delay(50);
         
        }
        else if (IRBR_status == 0 && IRBL_status==1)
        {
          Serial.println("Turn Right");
          //pause();
          // delay(100);
          Forward();
          delay(500);
          Left();
          delay(500);
          
        }
        else if (IRBR_status == 1 && IRBL_status==0)
        {
          Serial.println("Turn left");
          //pause();
          //delay(100);
          Forward();
          delay(500);
          Right();
          delay(500);
          
        }
        else 
        {
          Serial.println("Forward");
          //pause();
          //delay(100);
          Forward();
          delay(1000);
          
        }
    }

    void back_ultrasonic2()
    {
   
      if(IRBR_status == 1 && IRBL_status==1)
          {
          Serial.println("Scanning");
          Scanning();
         // delay(50);
          }
        else if (IRBR_status == 0 && IRBL_status==1)
        {
          Serial.println("Turn Right");
          //pause();
         // delay(100);
            Forward();
          delay(500);
          Left();
          delay(500);
          
        }
        else if (IRBR_status == 1 && IRBL_status==0)
        {
          Serial.println("Turn Left");
          //pause();
         // delay(100);
         Forward();
         delay(500);
         Right();
         delay(500);
          
        }
        else 
        {
          Serial.println("Forward");
          //pause();
          //delay(100);
          Forward();
          delay(1000);
          
        }
         
   }
   
    
    void Forward()
    {
         myMotor1->run(FORWARD);
         myMotor2->run(FORWARD);
           
    }
    
    void Reverse()
    {
         myMotor1->run(BACKWARD);
         myMotor2->run(BACKWARD);
    }
    
    void Scanning()
    {

//     myMotor1->setSpeed(150);
//     myMotor2->setSpeed(150);
       myMotor1->run(FORWARD);
       myMotor2->run(BACKWARD);
//     delay(1500);
//     myMotor1->run(BACKWARD);
//     myMotor2->run(FORWARD);
        
    }
    
//    void pause()
//    {
//         myMotor1->run(RELEASE);
//         myMotor2->run(RELEASE);
//       
//    }
    
    void Left()
    {
       myMotor1->run(FORWARD);
       myMotor2->run(BACKWARD);
               
    }
    
    
    void Right()
    {
        myMotor1->run(BACKWARD);
        myMotor2->run(FORWARD);
       
    
    }
 







    
