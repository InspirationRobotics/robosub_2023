#include <Wire.h>
#include <string.h>
#include <Adafruit_SI5351.h>
#include <SPI.h>

const int CS1 = 30;
const int CS2 = 31;
const int CS3 = 32;

Adafruit_SI5351 clockgen = Adafruit_SI5351();

void setClock(int DIV)
{
  if (clockgen.begin() != ERROR_NONE)
  {
    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  clockgen.setupPLLInt(SI5351_PLL_A, 35);
  clockgen.setupMultisynth(2, SI5351_PLL_A, DIV, 1, 1);

  /*
     874 - 1.0
     582 - 1.5
     436 - 2.0
     349 - 2.5
     290 - 3.0
     249 - 3.5
     235 - 3.7
     220 - 4.0
  */
  clockgen.enableOutputs(true);

}

void digitalPotWrite(int value, int Chip_s) {
  digitalWrite(Chip_s, LOW);
  SPI.transfer(0x00);
  SPI.transfer(value);
  digitalWrite(Chip_s, HIGH);
}



void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  //pinMode(CS1, OUTPUT);
  //pinMode(CS2, OUTPUT);
  //pinMode(CS3, OUTPUT);
  pinMode(A16, INPUT);
  //digitalWrite(CS1, HIGH);
  delay(10);
  setClock(235);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*digitalPotWrite(0,CS1);
    digitalPotWrite(0,CS2);
    digitalPotWrite(0,CS3);
  */int sensorValue = analogRead(A16);
  Serial.println(sensorValue);
  /*float voltage = sensorValue * (5.0 / 1023.0);
    if(voltage>3.8) {
      Serial.println(voltage);
      delay(50);
    }*/
  delay(10);
  if (Serial.available())
  {
    String temp;
    temp = Serial.readString();
    Serial.print(temp); //Printing the Serial data
    char tab1[3];
    strcpy(tab1, temp.c_str());
    int tab2 = atoi(tab1);
    if (tab2 > 5000) {
      tab2 = tab2 - 5000;
      setClock(tab2);
    }
  }
}
