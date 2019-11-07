/*
 Add an FM receiver to an old tube radio.
 Use a RDA5807M chip as receiver znd an Arduino Pro Mini as driver.
 The original variable capacitor in the radio is used for tunning.
 An external NE555 oscillator is used for varcap measuring..
 Source pour l'oscillateur NE555 et la mesure de fréquences: http://www.f-legrand.fr/scidoc/docmml/sciphys/arduino/oscillateur/oscillateur.html
 
 Philippe Leclercq 2019
 */
#define DEBUG 1

#include <radio.h>
#include <RDA5807M.h> // Radio library from https://github.com/mathertel/Radio

RDA5807M radio;    // Create an instance of a RDA5807 chip radio

RADIO_INFO info;


char inputPin = 5; // oscillator input to Timer1
 
volatile uint16_t count_high,count_low;
volatile uint32_t count;
volatile uint16_t ic; // interrupts counter
uint16_t intPerSample=10; // number of interrupts for a sample
float deltaT;
                      
void start_count() {
  // Timer2 : periodic interrupts
  TCCR2A |= (1 << WGM21); // CTC mode 'clear timer on compare), top = OCR2A
  OCR2A = 0xFF; // fréquence d'interruption 16/8 MHz /1024/256
  TIMSK2 = 1 << TOIE2; // overflow interrupt enable
  TCNT2 = 0;
  ic = 0;
 
  // Timer1 : pulse counter
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TIMSK1 = 1<<TOIE1; // overflow interrupt enable
  count = 0;
 
  sei(); // activation des interruptions
  TCCR2B |= (1 << CS12) | (1 << CS11) | (1 << CS10); // prescaler = 1024
  TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10); // external clock on rising edge
}
                       
ISR(TIMER2_OVF_vect) { // Overflow interrupt
   ic++;
   if (ic==intPerSample) {
     ic = 0;
     count_low = TCNT1;
     TCNT1 = 0;
     count = ((uint32_t)count_high)<<16 | count_low;
    count_high = 0;
   }
}
                       
ISR(TIMER1_OVF_vect) {
    count_high++;
}

// Min and max varcap values (pF)
#define MINCAP 41 
#define MAXCAP 439

// NE555 oscillator freq-capacitance conversion (R1=1Mohm R2=1Mohm)
// F = 1.44 / ( (R1+2R2) * C )
#define CONVERTFACTOR 480000
unsigned long freqBound=CONVERTFACTOR/MINCAP-CONVERTFACTOR/MAXCAP;
unsigned long lowBound=CONVERTFACTOR/MAXCAP;
int prevChan=0;
#define TICK 1000


void setup() 
{
#if DEBUG
  Serial.begin(115200); 
  Serial.println("");
  Serial.print("Radio...");
#endif

   pinMode(inputPin,INPUT);
  deltaT = 1.0*intPerSample*1024*256/F_CPU; // 1.0 necessary to compute as float
  start_count();
  
   // Initialize the Radio 
  int s=radio.init();
#if DEBUG
  if (s)
    Serial.println(" Found");
  else
    Serial.println(" Not found");
#endif
  radio.setMono(true);
  radio.setVolume(2);
  radio.setMute(true); 
}

void loop() {
 
  unsigned long freq = count/deltaT;
  if (freq < lowBound)
    freq=lowBound;
  unsigned capa = CONVERTFACTOR/freq;
  unsigned long chan = 1080-(freq-lowBound)*210/freqBound;
#if DEBUG
  Serial.print("Interrupts=");
  Serial.print(count);
  Serial.print(" Freq=");
  Serial.print(freq);
  Serial.print("Hz Capa=");
  Serial.print(capa);
  Serial.print("pF chan=");
  Serial.print(chan);
  Serial.println("");
#endif
     if (chan < 870) chan=870;
     if (chan > 1080) chan=1080;
      chan*=10; // to radio library format
      if (chan != prevChan) {
           prevChan = chan;
#if DEBUG
             Serial.print (" Tuned to ");
             Serial.println (chan);
#endif
         radio.setFrequency(chan);
      }
    delay (100); // pause!
#if DEBUG
   delay (1000);
#endif
}
