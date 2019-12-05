/*
 tube-radio-FM
 Adds an FM receiver to an old tube radio.
 The receiver is an RDA5807M module driven by an Arduino Pro Mini.
 The original variable capacitor in the radio is used for tunning.
 An external NE555 oscillator is used for varcap measuring.
 Source pour l'oscillateur NE555 et la mesure de fréquences: http://www.f-legrand.fr/scidoc/docmml/sciphys/arduino/oscillateur/oscillateur.html

 Channels are scanned during initial setup and values read on varcap are converted to a value corresponding to the nearest channel.

 Version without radio library. Compatible ATM168 (16ko)
 
 Philippe Leclercq 2019
 */
 
#define DEBUG 1

#include <Wire.h>

int prevChan=0;

// Min and max varcap values (pF)
//#define MINCAP 50
//#define MAXCAP 350
#define MINCAP 32
#define MAXCAP 512

// Convert pF boundaries to oscillator frequencies
// NE555 oscillator freq-capacitance conversion: F = 1.44 / ( (R1+2R2) * C ) with R1=R2=1Mohm
#define CONVERTFACTOR 480000 // (1.44/3)*10e6
unsigned long oscLowBound = CONVERTFACTOR/MAXCAP;
unsigned long oscFreqWidth = CONVERTFACTOR/MINCAP - CONVERTFACTOR/MAXCAP;

#define MAXCHAN 100
int sChan[MAXCHAN];
int nbChan=0;

// ----- Register Definitions ----
int reg;

// this chip only supports FM mode
#define FM_WIDTH 210
#define FM_MIN  870
#define FM_MAX 1080

#define RADIO_REG_CHIPID  0x00

#define RADIO_REG_CTRL    0x02
#define RADIO_REG_CTRL_OUTPUT 0x8000
#define RADIO_REG_CTRL_UNMUTE 0x4000
#define RADIO_REG_CTRL_MONO   0x2000
#define RADIO_REG_CTRL_BASS   0x1000
#define RADIO_REG_CTRL_SEEKUP 0x0200
#define RADIO_REG_CTRL_SEEK   0x0100
#define RADIO_REG_CTRL_RDS    0x0008
#define RADIO_REG_CTRL_NEW    0x0004
#define RADIO_REG_CTRL_RESET  0x0002
#define RADIO_REG_CTRL_ENABLE 0x0001


#define RADIO_REG_VOL     0x05
#define RADIO_REG_VOL_VOL   0x000F

#define RADIO_REG_RA      0x0A
#define RADIO_REG_RA_RDS       0x8000
#define RADIO_REG_RA_STC       0x4000
#define RADIO_REG_RA_SF        0x2000
#define RADIO_REG_RA_RDSBLOCK  0x0800
#define RADIO_REG_RA_STEREO    0x0400
#define RADIO_REG_RA_NR        0x03FF

#define RADIO_REG_RB          0x0B
#define RADIO_REG_RB_RSSI     0xFE00
#define RADIO_REG_RB_FMTRUE   0x0100
#define RADIO_REG_RB_FMREADY  0x0080
#define RADIO_RSSI_SHIFT 10

// I2C-Address RDA Chip for sequential  Access
#define I2C_RDA_SEQ  0x10

// I2C-Address RDA Chip for Index  Access
#define I2C_RDA_INDX  0x11



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

// Store channels in sChan[]
int scan() {
  int i = 0, j=0;
  int f, prev;
  int tuned;    // A stable frequency is tuned.

  TuneTo(FM_MIN); // Min of band
  do {
    seekUp(true);
    do {
      delay(20);
      tuned=radioLevel();
    } while (j++<40 && !tuned);
    if (tuned) {
     prev=f;
     f = getFrequency();
#if DEBUG
     Serial.print("chan ");
     Serial.print(nbChan);
     Serial.print(" freq ");
     Serial.println(f);
#endif
     sChan[nbChan++]=f;
    }
    j=0;
  } while (i++ < MAXCHAN && f>prev);
  nbChan--;
  seekUp(false);
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
  Wire.begin();   
  WriteReg(RADIO_REG_CTRL, RADIO_REG_CTRL_RESET); 
  reg = RADIO_REG_CTRL_OUTPUT|RADIO_REG_CTRL_MONO|RADIO_REG_CTRL_NEW|RADIO_REG_CTRL_ENABLE; 
  WriteReg(RADIO_REG_CTRL, reg); 

  scan();  // Initial scan
  
  WriteReg(RADIO_REG_VOL, 0x9080 | 13 ); // 15 = full volume

  reg |= RADIO_REG_CTRL_UNMUTE; 
  WriteReg(RADIO_REG_CTRL, reg); 

}

void loop()
{
  unsigned long chan;
  unsigned long freq = count/deltaT;
  if (freq < oscLowBound)
    chan = FM_MAX;
  else
    chan = FM_MAX - (freq - oscLowBound) * FM_WIDTH / oscFreqWidth;
  if (chan < FM_MIN) chan = FM_MIN;

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

// Search nearest channel
      int d, diff=0xFFF, index;
      for (int i=0; i<nbChan; i++) {
        d = chan - sChan[i];
        if (d < 0) d = -d;
        if (d < diff) {
          diff = d;
          index=i;
        }
       }
      chan=sChan[index];
// Use nearest channel in chan array
// Set radio if channel changed
  if (chan != prevChan) {
     prevChan = chan;

#if DEBUG
     Serial.print (" Tuned to ");
     Serial.println (chan);
#endif

     TuneTo((int)chan);
  }
  delay (100); // pause!

#if DEBUG
  delay (900);  // slow debug :)
#endif
}

void TuneTo( int ch)
{
       byte numH,numL;

       ch -= FM_MIN;
       numH=  ch>>2;
       numL = ((ch&3)<<6 | 0x10); 
       Wire.beginTransmission(I2C_RDA_INDX);
       Wire.write(0x03);
       Wire.write(numH);                     // write frequency into bits 15:6, set tune bit         
       Wire.write(numL);
       Wire.endTransmission();
}

//RDA5807_addr=0x11;       
// I2C-Address RDA Chip for random access
void WriteReg(byte reg,unsigned int val)
{
  Wire.beginTransmission(I2C_RDA_INDX);
  Wire.write(reg); Wire.write(val >> 8); Wire.write(val & 0xFF);
  Wire.endTransmission();
}

// Returns 1 word register
uint16_t ReadReg(byte reg)
{
  Wire.beginTransmission(I2C_RDA_INDX);
  Wire.write(reg);
  Wire.endTransmission(0);                         // restart condition
  Wire.requestFrom(I2C_RDA_INDX,2, 1);         // Retransmit device address with READ, followed by 2 bytes
  uint8_t hiByte = Wire.read();
  uint8_t loByte = Wire.read();
  Wire.endTransmission();
  return(256*hiByte + loByte);
}

// start seek mode upwards
void seekUp(bool enable) {
  if (enable) {
    // start seek mode
    reg |= RADIO_REG_CTRL_SEEK; 
    reg |= RADIO_REG_CTRL_SEEKUP; 
    WriteReg(RADIO_REG_CTRL, reg); 
  }
  else {
    // stop scanning right now
    reg &= (~RADIO_REG_CTRL_SEEK);
    WriteReg(RADIO_REG_CTRL, reg);
  } // if
} // seekUp()

// Signal level of radio station
int radioLevel() {
  int sig=0;
  // get register B
  uint16_t regRB = ReadReg(RADIO_REG_RB);
  if (regRB & RADIO_REG_RB_FMTRUE)
    sig = (regRB & RADIO_REG_RB_RSSI)>>RADIO_RSSI_SHIFT;
  return (sig);
} // radioLevel()

// retrieve the real frequency from the chip after automatic tuning.
int getFrequency() {
  // get register A
  uint16_t regRA = ReadReg(RADIO_REG_RA);
  uint16_t ch = regRA & RADIO_REG_RA_NR;
  int freq = FM_MIN + ch;
  return (freq);
}  // getFrequency
