#include <string.h>
#include <Adafruit_SI5351.h>
#include <ADC.h>
#include <ADC_util.h>

#define mic1  A10 //ADC 0 //Right
#define mic2  A11 //ADC 0 //Middle
#define mic3  A12 //ADC 1 //Left

#define oldPin1 A16
#define oldPin2 A15
#define oldPin3 A14

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
  pinMode(oldPin1, INPUT_PULLDOWN);
  pinMode(oldPin2, INPUT_PULLDOWN);
  pinMode(oldPin3, INPUT_PULLDOWN);

  Serial.begin(115200);

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

  setClock(220);
}

int mic1Val = 0;
int mic2Val = 0;
int mic3Val = 0;

int preMic1Val = 0;
int preMic2Val = 0;
int preMic3Val = 0;

const int recordingSize = 500;
uint16_t mic1Data[recordingSize];
uint16_t mic2Data[recordingSize];
uint16_t mic3Data[recordingSize];

elapsedMicros timeMic_;
elapsedMillis timeMil_;
boolean state = true;
boolean record = false;
int count = 0;


void analysis(uint16_t data1[], uint16_t data2[], uint16_t data3[]) {
  for(int i = 0; i<recordingSize;i++) {
    Serial.print(mic1Data[i]); Serial.print(", "); Serial.print(mic2Data[i]); Serial.print(", "); Serial.println(mic3Data[i]);
    delay(10);
  }
  //while(true){}
}


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
  if(state) {mic1Val = adc->adc0->readSingle();}
  else {mic2Val = adc->adc0->readSingle();}
  mic3Val = adc->adc1->readSingle();

  /*if(((mic1Val > 120 && preMic1Val <= 120) || (mic2Val > 120 && preMic2Val <= 120) || (mic3Val > 120 && preMic3Val <= 120)) && !record) {
    record = true;
  }
  
  if(record) {
    if(count==500) {
      count = 0;
      record = false;
      Serial.println("Done recording data");
      Serial.println("Right Middle Left");
      analysis(mic1Data, mic2Data, mic3Data);
      delay(5000);
    }
    else {
    mic1Data[count] = mic1Val;
    mic2Data[count] = mic2Val;
    mic3Data[count] = mic3Val;
    count++;}
  }

  preMic1Val = mic1Val;
  preMic2Val = mic2Val;
  preMic3Val = mic3Val;
*/

  /*if (Serial.available()) {
    String temp;
    temp = Serial.readString();
    Serial.print(temp); //Printing the Serial data
    char tab1[3];
    strcpy(tab1, temp.c_str());
    int tab2 = atoi(tab1);
    if (tab2 > 200 && tab2 < 900) {
      setClock(tab2);
    }
  }*/
  /*if(mic1Val<115) {mic1Val = 0;}
  //else{Serial.println(mic1Val);}
  if(mic2Val<115) {mic2Val = 0;}
  //else{Serial.println(mic2Val);}
  if(mic3Val<115) {mic3Val = 0;}
  //else{Serial.println(mic3Val);}*/
  
  Serial.print(mic1Val); Serial.print(", "); Serial.print(mic2Val); Serial.print(", "); Serial.println(mic3Val);
  delay(1);
  
}
