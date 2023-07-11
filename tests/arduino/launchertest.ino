#include <Servo.h>
Servo launcher;
int servoPin = 13;

void setup() {
  // put your setup code here, to run once:
  launcher.attach(servoPin);
  launcher.write(40);
  //Serial.begin(9600);
  //while(!Serial){
    //delay(1);
  }

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  launcher.write(0);
  delay(100000000000000000);
}
