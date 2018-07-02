#include "MeMegaPi.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//MeEncoderOnBoard motor2(SLOT2); 
MeMegaPiDCMotor motor2(PORT2B); 
MeMegaPiDCMotor motor1(PORT1B); 
MeGyro gyro;

float offset = 1;
float Kp = 21;
float Kd = 0; //0.1 0.2 1.5 2.0 ???
float Ki = 0; //0.01
float error = 0;
float prevError = 0;
float sum = 0;
float outputI = 0;

float lastTime = 0;
float curTime = 0;

void setup() {

 Serial.begin(9600);
   gyro.begin();

}

void loop() {
  curTime = millis()/1000.0;
  float deltaTime = (curTime - lastTime);
  lastTime = curTime;
  Serial.print("DELTA T:");
  Serial.print(deltaTime);
  
  gyro.update();
  Serial.read();
  Serial.print("    X:");
  Serial.print(gyro.getAngleX());
  error = gyro.getAngleX() - offset;
  Serial.print("   ERROR:");
  Serial.print(error);
  //outputI += (error * deltaTime);
    Serial.print("   AccuError:");
  Serial.print(outputI);
  sum = error*Kp + outputI*Ki + Kd*((error-prevError)/deltaTime);
  if (sum>255) {sum = 255;}
  if (sum<-255) {sum = -255;}
  Serial.print("   SUM:");
  Serial.print(sum);
  Serial.println();
//  if (error < 0){
//    motor1.run(sum);
//    motor2.run(-sum);
//  } else {
    motor1.run(-sum);
    motor2.run(sum);
//  }
  prevError = error;

}




//#include "MeMegaPi.h"
//const byte interruptPin =18;    
//const byte NE1=31;                 
//long count=0;
//unsigned long time;
//unsigned long last_time;
//MeMegaPiDCMotor motor1(PORT1B);   
//uint8_t motorSpeed = 100;
//void setup()
//{
//    pinMode(interruptPin, INPUT_PULLUP);
//    pinMode(NE1, INPUT);
//    attachInterrupt(digitalPinToInterrupt(interruptPin), blink,RISING);   
//    Serial.begin(9600);   
//}
//void loop()
//{
//    motor1.run(motorSpeed);       // value: between -255 and 255
//    time =millis(); 
//    if(time-last_time>2000)    
//    {
//          Serial.println(count);
//          last_time=time;
//   }
//}
//void blink()
//{
//    if (digitalRead(NE1)>0)   
//    count++;
//    else
//    count--;
//}
