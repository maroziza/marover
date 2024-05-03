
#include "DigiJoystick.h"   // libreria Joystick HID Digispark
#define CHANNELS 6
#define READ_PIN PB1

unsigned int channel[16]; 
unsigned int memleak[16]; 

unsigned char report[8];
unsigned int memcorrupt[16]; 


unsigned int pulseUpTime, pulseLowTime, startTime;
boolean lastPinState = LOW;
int numCurPulse=-1;
#define DEBUG 0
void setup() {
  startTime = micros();
  pinMode (READ_PIN, INPUT);
  lastPinState = digitalRead(READ_PIN);

#if DEBUG
  Serial.begin(38400);
#else
  usbDeviceConnect();
  DigiJoystick.delay(5000); // Let the AVRUSB driver do some houskeeping
#endif


}
bool started = false;
int ready = 0;
void loop () {
  calculateReport(); // Jump to our port read routine that orders the values

 if(ready>1)  {
    ready = 0;
#if DEBUG
#else
  DigiJoystick.delay(10); // Let the AVRUSB driver do some houskeeping
#endif
  }


}


void calculateReport() {  //The values read from the analog ports have to be ordered in a way the HID protocol wants it; a bit confusing.
  boolean curPinState = digitalRead(READ_PIN);
  // if (curPinState != lastPinState) {

  if (curPinState==HIGH && lastPinState==LOW) {
      pulseLowTime = pulseUpTime;
      pulseUpTime = micros();
      long diff = pulseUpTime-pulseLowTime;

      numCurPulse++;
      channel[numCurPulse] = diff;

      if(diff<1020) diff= 1020;
      if(diff>2040) diff= 2040;
      
if(numCurPulse <= CHANNELS)
      report[numCurPulse] = (uint8_t)(0xFF&((diff-1020)/4));


if(channel[numCurPulse]>10000) { numCurPulse=-1;}

      if(numCurPulse==CHANNELS) { 
        ready++;
#if DEBUG
  for(int c = 0; c < CHANNELS;c++) {
   Serial.print(c);
   Serial.print(": ");
   Serial.print(report[c],10);
   Serial.print("\t");
  }
  Serial.println();
#else 
        DigiJoystick.setValues(report);
#endif

      }
  }
    lastPinState = curPinState;
}
