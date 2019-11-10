/*
 tube-radio-FM
 Adds an FM receiver to an old tube radio.
 The receiver is an RDA5807M module driven by an Arduino Pro Mini.
 The original variable capacitor in the radio is used for tunning.
 An external NE555 oscillator is used for varcap measuring.
 Source pour l'oscillateur NE555 et la mesure de fréquences: http://www.f-legrand.fr/scidoc/docmml/sciphys/arduino/oscillateur/oscillateur.html
 
 Philippe Leclercq 2019
 */
 
#define DEBUG 1

#include <radio.h>
#include <RDA5807M.h> // Radio library from https://github.com/mathertel/Radio

RDA5807M radio;    // Create an instance of a RDA5807 chip radio
int prevChan=0;

// Min and max varcap values (pF)
#define MINCAP 50
#define MAXCAP 439

// Convert pF boundaries to oscillator frequencies
// NE555 oscillator freq-capacitance conversion: F = 1.44 / ( (R1+2R2) * C ) with R1=R2=1Mohm
#define CONVERTFACTOR 480000 // (1.44/3)*10e6
unsigned long lowBound = CONVERTFACTOR/MAXCAP;
unsigned long freqBound = CONVERTFACTOR/MINCAP - CONVERTFACTOR/MAXCAP;


#define INPUTPIN 5 // oscillator signal to pulse counter (Timer1)
 
volatile uint16_t count_high,count_low;
volatile uint32_t count;
volatile uint16_t ic; // interrupts counter
uint16_t nbPerSample=5; // number of periodic interrupts for a sample
float deltaT; // Elapsed time per sample
                      
void start_count() {
  // Timer2 : periodic interrupts
  TCCR2A |= (1 << WGM21); // CTC mode 'clear timer on compare), top = OCR2A
  OCR2A = 0xFF; // periodic interrupt frequency 16or8 MHz /1024/256
  TIMSK2 = 1 << TOIE2; // overflow interrupt enable
  TCNT2 = 0;
  ic = 0;
 
  // Timer1 : pulse counter (on rising edge)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TIMSK1 = 1<<TOIE1; // overflow interrupt enable
  count = 0;
 
  sei(); // enable interrupts
  TCCR2B |= (1 << CS12) | (1 << CS11) | (1 << CS10); // prescaler = 1024
  TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10); // external clock on rising edge
}
                       
ISR(TIMER2_OVF_vect) { // Overflow interrupt
   ic++;
   if (ic==nbPerSample) {
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

void setup() 
{
#if DEBUG
  Serial.begin(115200); 
  Serial.println("\nRadio...");
#endif

  pinMode(INPUTPIN,INPUT);
  deltaT = 1.0*nbPerSample*1024*256/F_CPU; // 1.0 necessary to compute as float
  start_count();
  
   // Initialize the Radio 
  radio.init();
  radio.setMono(true);
  radio.setVolume(2);
}

void loop()
{
  unsigned long chan;
  unsigned long freq = count/deltaT;
  if (freq < lowBound)
    chan = 1080;
  else
    chan = 1080 - (freq - lowBound) * 210 / freqBound;
  if (chan < 870) chan = 870;

#if DEBUG
  unsigned capa = CONVERTFACTOR/freq;
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
  delay (900);  // slow debug :)
#endif
}
