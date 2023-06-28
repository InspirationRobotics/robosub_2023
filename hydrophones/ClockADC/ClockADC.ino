#include <string.h>
#include <Adafruit_SI5351.h>
#include <ADC.h>
#include <ADC_util.h>

#define mic1  A10 //ADC 0 //Right
#define mic2  A11 //ADC 0 //Middle
#define mic3  A12 //ADC 1 //Left

ADC *adc = new ADC(); // adc object

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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(mic1, INPUT_PULLDOWN);
  pinMode(mic2, INPUT_PULLDOWN);
  pinMode(mic3, INPUT_PULLDOWN);

  Serial.begin(9600);

  ///// ADC0 ////
  adc->adc0->setAveraging(2);  // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  ////// ADC1 /////
  adc->adc1->setAveraging(2);  // set number of averages
  adc->adc1->setResolution(10); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  setClock(235);
}

int mic1Data;
int mic2Data;
int mic3Data;

elapsedMicros timeMic_;
elapsedMillis timeMil_;
boolean state = false;

unsigned long count;

void loop() {
  // put your main code here, to run repeatedly:
  state = !state;
  if(state) {
    adc->adc0->startSingleRead(mic1);
  }
  else {
    adc->adc0->startSingleRead(mic2);
  }
  adc->adc1->startSingleRead(mic3);
  while(!(adc->adc0->isComplete() && adc->adc1->isComplete())) {}
  if(state) {mic1Data = adc->adc0->readSingle();}
  else {mic2Data = adc->adc0->readSingle();}
  mic3Data = adc->adc1->readSingle();
  count++;

  if(timeMil_>=6000) {
    double freq = count/timeMil_;
    Serial.println(freq);
    while(true) {}
  }

  
}
