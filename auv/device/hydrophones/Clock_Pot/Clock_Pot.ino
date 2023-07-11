#include <Wire.h>
#include <string.h>
#include <Adafruit_SI5351.h>
#include <SPI.h>

#define mic1 A10 //Right
#define mic2 A11 //Middle
#define mic3 A12 //Left

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(mic1, INPUT_PULLDOWN);
  pinMode(mic2, INPUT_PULLDOWN);
  pinMode(mic3, INPUT_PULLDOWN);
  delay(10);
  setClock(235);
}

void loop() {
  // put your main code here, to run repeatedly:

  int mic1Data = analogRead(mic1);
  int mic2Data = analogRead(mic2);
  int mic3Data = analogRead(mic3);
  Serial.print(mic1Data); Serial.print(", "); Serial.print(mic2Data); Serial.print(", "); Serial.println(mic3Data);
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
