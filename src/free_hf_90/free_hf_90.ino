/*  QMAC HF-90 free-tune conversion software for Teensy 4.0
    Copyright (C) 2024 Wouter Jan Ubbels PE4WJ
    pe4wj@amsat.org

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Rotary.h> // Ben Buxton's rotary library https://github.com/buxtronix/arduino/tree/master/libraries/Rotary
#include <SPI.h>  // include the SPI library:
#include "Luts.h" // Look up tables for the synth division ratios
#include "Seven_segment.h" // 7 segment display conversion LUTs
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Bounce.h>



// SPI settings
SPISettings settingsA(1000, MSBFIRST, SPI_MODE0); // 1 MHz appears to be the lowest SPI speed supported on Teensy4.0. Both the MC145220 and the 74HC595 support SPI Mode 0


// pin assigmments
#define ADC_INPUT_PIN   A2 // audio input to ADC
#define SEN 10 // Synth chip select
#define DEN 9 // Display and control shift registers chip select
#define PEN 8 // Volume pot chip select
#define PTT_OUT 5 // PTT output, active low
#define PTT_IN 4 // PTT input, active low, use series diode
#define BR 6 // keypad bottom row
#define TR 7 // keypad top row
#define LCL 20 // keypad left column
#define MCL 21 // keypad middle column
#define RCL 19 // keypad right column
#define ROTARY_0 3 // rotary encoder
#define ROTARY_1 2 // rotary encoder
// Note: SPI library configures the following pins:
// SCK = 13
// MOSI = 11


// rotary
Rotary rotary = Rotary(ROTARY_0, ROTARY_1);

// PTT debouncer
Bounce pttBouncer = Bounce(PTT_IN, 50);

bool is_tx = false;

// const parameters
const uint8_t PALPF_NULL = 0x00;
const uint8_t PALPF_RESET = 0x01;
const uint8_t POT_COMMAND_BYTE = 0b00010001; // C1 = 0, C0 = 1, P1 = 0, P1 = 1
const int TONE_FREQ = 1000; // Hz tune tone frequency
const int TUNE_DELAY = 3000; // msec duration of tune signal after pressing "TUNE" key
const int BEEP_FREQ = 1238;//1700; // Hz roger beep frequency, Quindar outro trone = 2475 Hz
const int BEEP_DELAY = 250; // msec roger beep duration
const int keyscan_time = 40; // msec wait time between keyscans

// squelch
AudioInputAnalog         input(ADC_INPUT_PIN);
AudioAnalyzeFFT1024 myFFT;
AudioConnection patchCord1(input, 0, myFFT, 0);

elapsedMillis hangcounter = 0; // counter which increments when the squelch hangs

const int FFT_SIZE = 1024;
uint8_t low_idx = 7; // lower frequency limit - 300 Hz
uint8_t high_idx = 63; // upper frequency limit - 3000 Hz
int n_bins = high_idx - low_idx + 1; // the number of bins to be processed in the FFT for a 1024 point FFT

float thresh = 0.6; // squelch threshold
float ext_thresh = 2; // direct open squelch threshold
uint16_t hangtime = 2700;// // hang time in milliseconds (monoflop)
bool hang = 0; // boolean to indicate whether the squelch is hanging


uint8_t nsigs = 0;
uint8_t nsigs_threshold = 0;
bool was_sig = false;

// LPF bi-stable relay control
elapsedMillis lpfcounter = 0;
uint LPF_NULL_TIME = 3000; // time between setting a new frequency and issueing a NULL to the LPF latching relay driver to conserve power
bool newfreq = false;


// LO1 low / hi band VCO treshold
int HIGH_BAND_XHOLD = 8500000; // Hz


// synth registers
int RREGs;
int NREGs;
int RREG;
int NREG;

// struct to hold synth registers
struct synthRegs
{
  int RREGs_out;
  int NREGs_out;
  int RREG_out;
  int NREG_out;
  uint8_t CREG_out;
  uint8_t CREGs_out;
};

// 7 segment display struct
struct digits
{
  uint8_t d100Hz;
  uint8_t d1kHz;
  uint8_t d10kHz;
  uint8_t d100kHz;
  uint8_t d1MHz;
  uint8_t d10MHz;

};


// declare global synth regs
synthRegs outreg;
synthRegs writereg;


// declare and initialize variable register values
uint8_t  MISCREG = 0b00000000;
uint8_t  PALPFREG = 0b00000000;
uint8_t  VOLUMEREG = 0b00000000;
uint8_t  MICAGCMODEREG = 0b00000000;
uint8_t previousPALPFREG = 0b000000001;

uint8_t volume = 16; // audio volume
const uint8_t VOLUME_STEP = 12;; // volume step size
bool is_muted = false;


int freq = 3630000; // defaul startup frequency
int previousfreq = freq + 100; // offset to invoke setting frequency upon startup

// mode, AGC, mic inhibit
bool is_agc_slow = true;
bool is_mic_inhibit = false;

// menu strings
String mode = "LSB";
String sql = "SQL";
String pwr = "HIP";
String agc = "SLOAGC";
String rbp = "BEEP";


uint8_t menu_idx = 0;
const uint8_t N_MENUS = 6; // number of available menus
// 0 = vfo <frequency display>
// 1 = mode USB LSB
// 2 = sql NOSQL SQL
// 3 = pwr HIPWR LOPWR
// 4 = agc SLOAGC FSTAGC
// 5 = roger beep NOBEEP BEEP


const uint8_t N_STEPS = 6;
const int steps[N_STEPS] = {25, 100, 1000, 10000, 100000, 1000000};
uint8_t step_idx = 2;

// "misc" shift register
bool is_atu_on = true;
bool is_low_power = false;
bool is_50V_off = false;
bool is_disable_selcall = true;
bool is_disable_loopback = true;

// PA LPF selection LUTs
const int BANDLIMIT[6] = {32770, 51200, 81920, 116730, 161790, 300000};
const uint8_t palpflut[6] = {0x40, 0x10, 0x04, 0x02, 0x08, 0x20};


String welcomestr = "hf-90f";

const int F_RF_MIN = 2000000; // Hz Minimum RF frequency
const int F_RF_MAX = 30000000; // *Hz Minimum RF frequency


void rotate() {
// called everytime a rotary encoder interrupt fires

  unsigned char result = rotary.process();

  if (!is_tx) { // only act upon rotary when not transmitting

    switch (menu_idx) {
      case 0:

      if (result == DIR_CW) {
        if (freq + steps[step_idx] < F_RF_MAX) {
          freq = freq + steps[step_idx];


        }
      }
      else if (result == DIR_CCW) {
        if (freq - steps[step_idx] > F_RF_MIN) {

          freq = freq - steps[step_idx];
        }
      }
      break;

      case 1:
      if (result == DIR_CW) { // only change upon CW direction rotation


        if (mode == "LSB") {
          mode = "USB";
        }
        else {
          mode = "LSB";
        }
        
        writereg = freq2synthval(freq);
        write_synth(writereg);
        // display mode and set shift registers (including LSB/USB selection bit)
        String text = get_menu_text(menu_idx);
        display(text, true);    
      }
      break;


      case 2:
      if (result == DIR_CW) { // only change upon CW direction rotation


        if (sql == "NOSQL") {
          sql = "SQL";
          // force immediate squelch closure
          hang = 0;
          hangcounter = 0;
          mute_audio();
          
          
        }
        else {
          sql = "NOSQL";
          unmute_audio();
        }
        // display mode
        String text = get_menu_text(menu_idx);

        display(text, true);    
      }
      break;

      case 3:
      if (result == DIR_CW) { // only change upon CW direction rotation


        if (pwr == "HIP") {
          pwr = "LOP";
          is_low_power = true;
        }
        else {
          pwr = "HIP";
          is_low_power = false;
        }

        
        // display mode
        String text = get_menu_text(menu_idx);

        display(text, true); // this also updates the power register value
      }
      break;

      
      case 4:
      if (result == DIR_CW) { // only change upon CW direction rotation


        if (agc == "SLOAGC") {
          agc = "FSTAGC";
          is_agc_slow = false;
        }
        else {
          agc = "SLOAGC";
          is_agc_slow = true;
        }

        
        // display mode
        String text = get_menu_text(menu_idx);

        display(text, true); // this also updates the agc register value
      }
      break;

      case 5:
      if (result == DIR_CW) { // only change upon CW direction rotation


        if (rbp == "BEEP") {
          rbp = "NOBEEP";
        }
        else {
          rbp = "BEEP";
        }

        
        // display mode
        String text = get_menu_text(menu_idx);

        display(text, true); // this also updates the power register
      }
      break;
      

     

    }

  set_new_freq();
  }
}



int freq2palpfreg(int freq_in) {
  // derive a PA / LPF register word from the tuned frequency
  uint8_t palpfreg = 0;
  uint8_t i = 0;
  while (freq_in > BANDLIMIT[i]) {
    i++;
  }
  palpfreg = palpflut[i];
  
  return palpfreg;
}

synthRegs freq2synthval(int freq_in) {
  // convert a combination of receive frequency + mode (USB or LSB) to a set of synth values
  
  // look up the required LO1 frequency index
  //int lo1_idx = int((freq_in - F_RF_MIN) / LO1_INC);
  //float inter = float(freq_in + 100 - F_RF_MIN) / LO1_INC - lo1_idx - 0.5;

  // bound freq_in
  if (freq_in < F_RF_MIN) {
    freq_in = F_RF_MIN;
  }

  if (freq_in > F_RF_MAX) {
    freq_in = F_RF_MAX;
  }
  

  // calculate offset depending on sideband used (LSB or USB)
  int sideband_offset;
  if (mode == "USB") {

    sideband_offset = -2400; // 453.600 kHz = -2400 Hz relative to 456 kHz
  }
  else {

    sideband_offset = 400; // 456.400 kHz = 400 Hz relative to 456 kHz

  }
  
  int lo1_idx;
  int lo2_idx;

  // calculate LO1 LUT index
  lo1_idx = int((freq_in + int(0.5*LO1_STEPSIZE)- F_RF_MIN) / LO1_STEPSIZE);

  // calculate LO1 LUT index
  float inter = float(freq_in - F_RF_MIN) / LO1_STEPSIZE - lo1_idx;
  lo2_idx = LO2_MID_IDX - int(inter * (LO1_STEPSIZE / LO2_STEPSIZE) ) + int(sideband_offset/LO2_STEPSIZE) + Cor_LUT[lo1_idx];
  
  // Retrieve total division ratios N and R for LO1 and LO2 PLLs from the Look Up Tables
  // SUB / LO1
  short Ns = Ns_LUT[lo1_idx];
  short As = As_LUT[lo1_idx];
  short Rs = Rs_LUT[lo1_idx];

  // MAIN / LO2
  short N = N_LUT[lo2_idx];
  short A = A_LUT[lo2_idx];
  short R = R_LUT[lo2_idx];  

  // now assemble the registers
  // SUB / LO1
  uint8_t BUFFER_AND_CONTROLs = 0x02;
  uint8_t STEERs = 0x01;
  uint8_t OUTPUT_A_FUNCTIONs = 0x00;
  uint8_t PRESCREGs = 0x00;

  // MAIN / LO2
  uint8_t BUFFER_AND_CONTROL = 0x02;
  uint8_t STEER = 0x00;
  uint8_t OUTPUT_A_FUNCTION = 0x00;
  uint8_t PRESCREG = 0x01;


  // SUB / LO1
  outreg.RREGs_out  = (Rs & 0x1FFF) | ((BUFFER_AND_CONTROLs & 0x07) << 13); // 16 bit
  outreg.NREGs_out  = (As & 0xFF) | ((Ns & 0xFFF) << 8 ) | (PRESCREGs << 20) | ((OUTPUT_A_FUNCTIONs & 0x03) << 21) | (STEERs << 23); // 24 bit
  

  // MAIN / LO2
  outreg.RREG_out  = (R & 0x1FFF) | ((BUFFER_AND_CONTROL & 0x07) << 13); // 16 bit
  outreg.NREG_out  = (A & 0xFF) | ((N & 0xFFF) << 8 ) | (PRESCREG << 20) | ((OUTPUT_A_FUNCTION & 0x03) << 21) | (STEER << 23); // 24 bit

  // Control (C) registers
  if (freq_in > HIGH_BAND_XHOLD) { // switch VCO range on LO1 above HIGH_BAND_XHOLD
    outreg.CREG_out = 0b00110000;
  }
  else {
    outreg.CREG_out = 0b00110010;
  }
  outreg.CREGs_out = 0b10110000;

  return outreg;
}



uint8_t lookupsegments(String instr) {
  // look up 7 segment segments depending on the input character
  uint8_t i = 0;
  uint8_t out = 0;
  while (lookup_segments[i].alphanum != instr) {
    i++;
    if (i == nalphas) { // now the alphanumeric char was not found in the table, replace by null

      break;
    }
  }
  if (i == nalphas) {
    out = 0;
  }

  else {// it was found
    out = lookup_segments[i].segments;
  }

  return out;
}

digits str2disp(String disp_in, bool is_text_in) {
  // convert a string to a set of 7 segment characters in an array for displaying
  uint8_t lsb;
  if (mode == "LSB") {
    lsb = 1;
  }
  else {
    lsb = 0;
  }

  digits disp;
  if (is_text_in) {
    while (disp_in.length() < 6) {
      disp_in+= " "; // add white space to pad to 6 digits
    }
      disp.d100Hz = lookupsegments(disp_in[5]);
      disp.d1kHz = lookupsegments(disp_in[4]);
      disp.d10kHz = lookupsegments(disp_in[3]);
      disp.d100kHz = lookupsegments(disp_in[2]);
      disp.d1MHz = lookupsegments(disp_in[1]);
      disp.d10MHz = lookupsegments(disp_in[0]);

  }
  else { // now, it is a numeric frequency value
  
    if (disp_in.length() == 5) {
      disp.d100Hz = lookupsegments(disp_in[4]);
      disp.d1kHz = lookupsegments(disp_in[3]) + 1; // decimal point
      disp.d10kHz = lookupsegments(disp_in[2]);
      disp.d100kHz = lookupsegments(disp_in[1]);
      disp.d1MHz = lookupsegments(disp_in[0]);
      disp.d10MHz = lookupsegments("x") + lsb; // decimal point
    }
    else if (disp_in.length() == 6) {
      disp.d100Hz = lookupsegments(disp_in[5]);
      disp.d1kHz = lookupsegments(disp_in[4]) + 1; // decimal point
      disp.d10kHz = lookupsegments(disp_in[3]);
      disp.d100kHz = lookupsegments(disp_in[2]);
      disp.d1MHz = lookupsegments(disp_in[1]);
      disp.d10MHz = lookupsegments(disp_in[0]) + lsb; // decimal point
    }
  }  

  return disp;
}

uint8_t genmiscreg(bool is_atu_on_in, bool is_low_power_in, bool is_50V_off_in, bool is_disable_selcall_in, bool is_disable_loopback_in) {
  // generate contents for the "MISC" register
  uint8_t misc = (int(is_atu_on_in) << 1) | (int(is_low_power_in) << 2) | (int(is_50V_off_in) << 3) | (int(is_disable_selcall_in) << 6) | (int(is_disable_loopback_in) << 7);

  return misc;
}

uint8_t genmicagcmodereg(bool mic_in, bool agc_in, String mode_in) {
  // convert mic inhibit, agc and mode to a register value
  bool mode_is_usb = false;
  if (mode_in == "USB") {

    mode_is_usb = true;
  }
  else {

    mode_is_usb = false;

  }

  uint8_t mic_agc_mode = int(mode_is_usb) | (int(agc_in) << 1) | (int(mic_in) << 6);

  return mic_agc_mode;

}


void write_synth(synthRegs inregs) {
  // Write an entire synth transaction
  SPI.beginTransaction(settingsA);

  digitalWrite(SEN, LOW);
  SPI.transfer(inregs.CREG_out);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);
  delayMicroseconds(50);

  digitalWrite(SEN, LOW);
  SPI.transfer(inregs.CREGs_out);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);
  delayMicroseconds(50);

  digitalWrite(SEN, LOW);
  SPI.transfer((inregs.RREGs_out >> 8) & 0xFF);
  SPI.transfer((inregs.RREGs_out) & 0xFF);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);
  delayMicroseconds(50);

  digitalWrite(SEN, LOW);
  SPI.transfer((inregs.NREGs_out >> 16) & 0xFF);
  SPI.transfer((inregs.NREGs_out >> 8) & 0xFF);
  SPI.transfer((inregs.NREGs_out) & 0xFF);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);
  delayMicroseconds(50);

  digitalWrite(SEN, LOW);
  SPI.transfer((inregs.RREG_out >> 8) & 0xFF);
  SPI.transfer((inregs.RREG_out) & 0xFF);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);
  delayMicroseconds(50);

  digitalWrite(SEN, LOW);
  SPI.transfer((inregs.NREG_out >> 16) & 0xFF);
  SPI.transfer((inregs.NREG_out >> 8) & 0xFF);
  SPI.transfer((inregs.NREG_out) & 0xFF);
  delayMicroseconds(50);
  digitalWrite(SEN, HIGH);

  SPI.endTransaction();
}

void write_shiftreg(uint8_t MISCREG_in, uint8_t PALPFREG_in, uint8_t VOLUMEREG_in, uint8_t MICAGCMODEREG_in, digits disp_in) {
  // Write an entire Display / control shift register transaction
  SPI.beginTransaction(settingsA);  

  delayMicroseconds(500);
  digitalWrite(DEN, LOW); // Assert DEN line low
  delayMicroseconds(5000); // set to 5000 to avoid spurious triggering of PEN due to xtalk

  // Transfer the contents of the 10 shift registers

  SPI.transfer(MISCREG_in);

  SPI.transfer(PALPFREG_in);

  SPI.transfer(VOLUMEREG_in);

  SPI.transfer(MICAGCMODEREG_in);

  SPI.transfer(disp_in.d10MHz);

  SPI.transfer(disp_in.d1MHz);

  SPI.transfer(disp_in.d100kHz);

  SPI.transfer(disp_in.d10kHz);

  SPI.transfer(disp_in.d1kHz);

  SPI.transfer(disp_in.d100Hz);

  delayMicroseconds(5000);
  
  digitalWrite(DEN, HIGH); // assert DEN line high
  delayMicroseconds(500);
  SPI.endTransaction();
}


void write_pot(uint8_t volume_in) {
  // write to SPI controlled volume pot

  SPI.beginTransaction(settingsA);
    
  delayMicroseconds(500);
  digitalWrite(PEN, LOW); // Assert PEN line low
  delayMicroseconds(500);
  SPI.transfer(POT_COMMAND_BYTE);
  SPI.transfer(volume_in);
  delayMicroseconds(5000);  
  digitalWrite(PEN, HIGH); // assert DEN line high
  delayMicroseconds(500);

  SPI.endTransaction();
}


void mute_audio() {
  write_pot(0);
  is_muted = true;
}

void unmute_audio() {
  write_pot(volume);
  is_muted = false;
}


uint8_t scan_keys() {
  // scan the front panel keys
  int8_t key = 0;
  delay(keyscan_time);
  digitalWrite(BR, LOW);
  if (!digitalRead(LCL)) {
    key = 1;
    delay(100);
  }
  else if (!digitalRead(MCL)) {
    key = 3;
    delay(100);
  }
  else if (!digitalRead(RCL)) {
    key = 5;
    delay(100);
  }
  
  delay(keyscan_time);
  digitalWrite(BR, HIGH);

  digitalWrite(TR, LOW);
  if (!digitalRead(LCL)) {
    key = 2;
    delay(100);
  }
  else if (!digitalRead(MCL)) {
    key = 4;
    delay(100);
  }
  else if (!digitalRead(RCL)) {
    key = 6;
    delay(100);
  }
  
  delay(keyscan_time);
  digitalWrite(TR, HIGH);

return key;
}


void set_new_freq() {
  // update frequency on synth and set shift registers
  if (freq != previousfreq)  {
    writereg = freq2synthval(freq);

    write_synth(writereg);

    // calculate shift register values
    MISCREG = genmiscreg(is_atu_on, is_low_power, is_50V_off, is_disable_selcall, is_disable_loopback);

    MICAGCMODEREG = genmicagcmodereg(is_mic_inhibit, is_agc_slow, mode);
    digits dispd = str2disp(String(freq/100), false);

    PALPFREG = freq2palpfreg(freq/100);
    if (PALPFREG != previousPALPFREG) { // now we need to set the latching relays properly
      // first do a reset of the latching relays
      
      write_shiftreg(MISCREG, PALPF_RESET, VOLUMEREG, MICAGCMODEREG, dispd);
      // wait 3 msec
      delay(3);
      // now write the proper value to the PALPFREG
      write_shiftreg(MISCREG, PALPFREG, VOLUMEREG, MICAGCMODEREG, dispd);

      // update previousPALPFREG)
      previousPALPFREG = PALPFREG;
      // set lpfcounter to 0 and set newfreq flag such that pa lpf gets set to NULL after NULL_TIME
      lpfcounter = 0;
      newfreq = true;
      
    }

    else {
      // now, the PALPFREG value has not changed, write NULL directly
      write_shiftreg(MISCREG, PALPF_NULL, VOLUMEREG, MICAGCMODEREG, dispd);
    }
          
    previousfreq = freq;

  }
}

void squelch() {  
    // Frequency domain relative variance squelch for SSB/CW, invented by PA3FWM: https://www.pa3fwm.nl/technotes/tn16f.html
    float sum;
    float mean;
    float variance;
    float relvar;
    uint8_t i;
 
    if (myFFT.available()) {
      // each time new FFT data is available

      // calculate the mean of the data
      sum = 0;

      n_bins = high_idx - low_idx + 1; // the number of bins to be processed in the FFT for a 1024 point FFT
      for (i = low_idx; i <= high_idx; i++) {
          sum += myFFT.read(i); // sum over all bins in the FFT     
      }
      mean = sum / float(n_bins); // divide the sum by the number of datapoints

      // calculate the sum of the squares of the differences from the mean
      sum = 0; // re-initialize sum
      for (i = low_idx; i <= high_idx; i++) {
          sum += pow((mean - myFFT.read(i)), 2); // sum over all bins in the FFT       
      }

      // from that, calculate the variance
      variance = sum / float(n_bins); // divide the sum by the number of datapoints

      // now calculate the relative variance by dividing the variance by the mean squared
      relvar = variance / (mean * mean);

      // simple state machine - determine if a signal is present, but only when the squelch is not hanging on a previous signal

        if (relvar > thresh) {

          if (was_sig) {
            nsigs++;
          }
          else {
            nsigs = 0;
            was_sig = true;
          }

        }
        else {
          was_sig = false;
          nsigs = 0;
          if (hang) {
          // do nothing except check whether the max hangtime has been obtained
            if (hangcounter > hangtime) {
              hangcounter = 0;
              hang = 0;
              mute_audio();
              //is_muted = true;

            }
          }
          else {

            nsigs = 0;
          }

        }

        if (nsigs > nsigs_threshold) {
          if (hang ==0) {// only unmute once to limit SPI traffic
            unmute_audio();
            //is_muted = false;
          }

          //start "hanging"
          hang = 1;
          hangcounter = 0;
          nsigs = 0;          
        }
        // open directly on 2nd threshold
        if (relvar > ext_thresh) {
          if (hang ==0) {// only unmute once to limit SPI traffic
            unmute_audio();
            //is_muted = false;
          }
          //start "hanging"
          hang = 1;
          hangcounter = 0;
          nsigs = 0;          
        }
    }
  }


void display(String dispinput_in, bool is_text_in) {

      MISCREG = genmiscreg(is_atu_on, is_low_power, is_50V_off, is_disable_selcall, is_disable_loopback);
      PALPFREG = PALPF_NULL;
          
      MICAGCMODEREG = genmicagcmodereg(is_mic_inhibit, is_agc_slow, mode);
      digits dispd = str2disp(dispinput_in, is_text_in);
    
      // write to shift register chain
      write_shiftreg(MISCREG, PALPFREG, VOLUMEREG, MICAGCMODEREG, dispd);
}

String get_menu_text(uint8_t menu_idx_in) {
  String menu_text;
    switch (menu_idx_in) {
      case 0: menu_text = ""; // VFO display
      break;

      case 1: menu_text = mode;
      break;

      case 2: menu_text = sql;
      break;

      case 3: menu_text = pwr;
      break;

      case 4: menu_text = agc;
      break;

      case 5: menu_text = rbp;
      break;
    }

  return menu_text;
}

void setup() {

  

  // init debug serial port
  Serial.begin(9600);

  // allocate audio memory
  AudioMemory(120);
  // set FFT window function
  myFFT.windowFunction(AudioWindowHanning1024);

  // set the SEN pin as an output:
  pinMode(SEN, OUTPUT);
  digitalWrite(SEN, HIGH);

  // set the DEN pin as an output:
  pinMode(DEN, OUTPUT);
  digitalWrite(DEN, HIGH);

  // set the PEN pin as an output:
  pinMode(PEN, OUTPUT);
  digitalWrite(PEN, HIGH);

  // initialize SPI:
  SPI.begin();

  // mute the audio
  mute_audio();

  // setup rotary pins to use internal pullup
  pinMode(2, INPUT_PULLUP); // Rotary
  pinMode(3, INPUT_PULLUP); // Rotary


  // setup keypad pins
  pinMode(BR, OUTPUT); // Bottom Row
  pinMode(TR, OUTPUT); // Top Row
  pinMode(LCL, INPUT_PULLUP); // Left Column
  pinMode(MCL, INPUT_PULLUP); // Middle Column
  pinMode(RCL, INPUT_PULLUP); // Right Column

  digitalWrite(BR, HIGH);
  digitalWrite(TR, HIGH);

  // setup PTT pins
  pinMode(PTT_OUT, OUTPUT); // PTT output
  digitalWrite(PTT_OUT, HIGH); // set PTT output to high to ensure RX mode
  pinMode(PTT_IN, INPUT_PULLUP); // PTT input

  // setup rotary interrupts for rotary encoder
  attachInterrupt(ROTARY_1, rotate, CHANGE);
  attachInterrupt(ROTARY_0, rotate, CHANGE);


  // display welcome message and calculate shift register values
  MISCREG = genmiscreg(is_atu_on, is_low_power, is_50V_off, is_disable_selcall, is_disable_loopback);
  PALPFREG = PALPF_NULL;

  MICAGCMODEREG = genmicagcmodereg(is_mic_inhibit, is_agc_slow, mode);
  digits dispd = str2disp(String(welcomestr), true);

  // write to shift register chain
  write_shiftreg(MISCREG, PALPFREG, VOLUMEREG, MICAGCMODEREG, dispd);
  delay(500); // startup delay
  set_new_freq();
}

void loop() { // main loop

  // check PTT
  pttBouncer.update ( );
  uint8_t is_ptt = pttBouncer.fallingEdge();
  uint8_t was_ptt = pttBouncer.risingEdge();

  if (is_ptt == HIGH) { // now the PTT button was pressed
    is_tx = true;
    mute_audio(); // first, mute the audio
    
    digitalWrite(PTT_OUT, LOW); // assert PTT line    
    
  }

  
  if (was_ptt == HIGH) { // now the PTT button was released
    if (rbp == "BEEP") { // if roger beep is enabled
      // do roger beep for 250 msec
      tone(TR, BEEP_FREQ); // output tone
      delay(BEEP_DELAY); // wait for duration of roger beep
      noTone(TR); // stop tone
    }
    digitalWrite(PTT_OUT, HIGH); // release the PTT line
    is_tx = false;

    if (sql == "NOSQL") { // if squelch is not active
      unmute_audio(); // unmute the audio again
    }
  }

  
  // run squelch algorithm, scan keys and if necessary de-energize latching relays. But only if we are not transmitting.
  if (!is_tx) {
    if (sql == "SQL") {
      squelch();
    }
      

    // scan keys and interpret the result
    uint8_t key_int = scan_keys();

    if (key_int == 1) {
      if (step_idx > 0) {
          step_idx -= 1;
      }
    }
    else if (key_int ==2) {
      if (step_idx < N_STEPS-1) {
        step_idx += 1;
      }

    }  

    if (key_int == 4) {
      if (menu_idx < N_MENUS-1) {
        menu_idx+=1;
      }
      else {
        menu_idx = 0;
      }
      
      if (menu_idx == 0) {
        // VFO mode
        display(String(freq/100), false);
      }
      else {
        String text = get_menu_text(menu_idx);
        display(text, true);    
      }
      
    }

    if (key_int == 3) {
      // invoke tune
      // first mute audio
      is_tx = true;

      display(" tune ", true);
      mute_audio();

      // go zero beat
      if (mode == "USB") {
        // tune lower in frequency, but do not update display
        writereg = freq2synthval(freq - TONE_FREQ);
        write_synth(writereg);
      }
      else { // LSB
        // tune higher in frequency, but do not update display
        writereg = freq2synthval(freq + TONE_FREQ);
        write_synth(writereg);
      }

      digitalWrite(PTT_OUT, LOW); // assert PTT line
      
      tone(TR, TONE_FREQ); // output tone
      delay(TUNE_DELAY); // wait
      noTone(TR); // stop tone
      digitalWrite(PTT_OUT, HIGH); // release PTT line
      // go back to original frequency, but do not update display
      writereg = freq2synthval(freq);
      write_synth(writereg);
      if (sql == "NOSQL") { // if squelch is not active
        unmute_audio(); // unmute the audio again
      }
  
      display(String(freq/100), false);
      is_tx = false;

    }



    if (key_int == 5) {
      if (volume >= VOLUME_STEP) {
          volume -= VOLUME_STEP;
      }
      if (!is_muted) {
        write_pot(volume);
      }
    }
    else if (key_int ==6) {
      if (volume < 256-VOLUME_STEP) {
        volume += VOLUME_STEP;
      }
      if (!is_muted) {
        write_pot(volume);
      }

    }


    if (lpfcounter > LPF_NULL_TIME) { // de-energize latching relays after LPF_NULL_TIME to save power
      if (newfreq) {
        newfreq = false;
        MISCREG = genmiscreg(is_atu_on, is_low_power, is_50V_off, is_disable_selcall, is_disable_loopback);
        PALPFREG = PALPF_NULL;    

        MICAGCMODEREG = genmicagcmodereg(is_mic_inhibit, is_agc_slow, mode);
        digits dispd = str2disp(String(freq/100), false);    

        // write to shift register chain
        write_shiftreg(MISCREG, PALPFREG, VOLUMEREG, MICAGCMODEREG, dispd);      

        }
      lpfcounter = 0;
    }

  }

}
