/* Example for synchonized measurements using both ADC present in Teensy 3.1
 *  You can change the number of averages, bits of resolution and also the
 * comparison value or range.
 */

#include <ADC.h>
#include <ADC_util.h>

#ifdef ADC_DUAL_ADCS

//Pins are 30, 31, 32 (need to resolder)
//https://github.com/KurtE/TeensyDocuments/blob/master/Teensy4.1%20Pins.pdf

int mic1 = A10; //ADC 0
int mic2 = A11; //ADC 0
int mic3 = A12; //ADC 1

ADC *adc = new ADC(); // adc object

elapsedMicros time_;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(mic1, INPUT_PULLDOWN);
  pinMode(mic2, INPUT_PULLDOWN);
  pinMode(mic3, INPUT_PULLDOWN);

  Serial.begin(9600);

  ///// ADC0 ////
  adc->adc0->setAveraging(2);  // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  ////// ADC1 /////
  adc->adc1->setAveraging(2);  // set number of averages
  adc->adc1->setResolution(10); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed

  adc->startSynchronizedContinuous(mic1, mic3);
  // You can also try:
  // adc->startSynchronizedContinuousDifferential(A10, A11, A12, A13);
  // Read the values in the loop() with readSynchronizedContinuous()

  delay(100);
  Serial.println("end setup");
}

int value = 0;
int value2 = 0;

ADC::Sync_result result;

void loop() {

  // You can also try:
  // result = adc->analogSynchronizedRead(readPin, readPin2);
  // result = adc->analogSynchronizedReadDifferential(A10, A11, A12, A13);

  result = adc->readSynchronizedContinuous();
  // if using 16 bits and single-ended is necessary to typecast to unsigned,
  // otherwise values larger than 3.3/2 will be interpreted as negative
  //result.result_adc0 = (uint16_t)result.result_adc0;
  //result.result_adc1 = (uint16_t)result.result_adc1;

  // digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

  // Serial.print("Value ADC0: ");
  Serial.print(result.result_adc0);
  Serial.print(", ");
  Serial.println(result.result_adc1);

  // Print errors, if any.
  if (adc->adc0->fail_flag != ADC_ERROR::CLEAR) {
    Serial.print("ADC0: ");
    Serial.println(getStringADCError(adc->adc0->fail_flag));
  }
#ifdef ADC_DUAL_ADCS
  if (adc->adc1->fail_flag != ADC_ERROR::CLEAR) {
    Serial.print("ADC1: ");
    Serial.println(getStringADCError(adc->adc1->fail_flag));
  }
#endif

  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

  // delay(100);
}

#else  // make sure the example can run for any boards (automated testing)
void setup() {}
void loop() {}
#endif // ADC_DUAL_ADCS
