#include "Arduino.h"
#include "mbed.h"
#include "USBAudio.h"
#include <Wire.h>

#define AUDIOSAMPLING 48000  // USB Audio sampling frequency

#define N_FREQ 2 // number of using RF frequencies　(<= 7)
#define FREQ_0 7041000 // RF freqency in Hz
#define FREQ_1 7074000 // in Hz
//#define FREQ_2 7074000 // in Hz
//#define FREQ_3 7074000 // in Hz
//#define FREQ_4 7074000 // in Hz
//#define FREQ_5 7074000 // in Hz
//#define FREQ_6 7074000 // in Hz
extern uint64_t Freq_table[N_FREQ]={FREQ_0,FREQ_1}; // Freq_table[N_FREQ]={FREQ_0,FREQ_1, ...}

#define pin_RX 27 //pin for RX switch (D2,output)
#define pin_TX 28 //pin for TX switch (D3,output)
#define pin_SW 3 //pin for freq change switch (D10,input)
#define pin_RED 17 //pin for Red LED (output)
#define pin_GREEN 16 //pin for GREEN LED (output)
#define pin_BLUE 25 //pin for BLUE LED (output)
#define pin_LED_POWER 11 //pin for NEOPIXEL LED power (output)
#define pin_LED 12 //pin for NEOPIXEL LED (output)

#include <si5351.h>
Si5351 si5351;

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, pin_LED);
uint8_t color = 0;
#define BRIGHTNESS 5  //max 255
uint32_t colors[] = {pixels.Color(BRIGHTNESS, 0, 0), pixels.Color(0, BRIGHTNESS, 0), pixels.Color(0, 0, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, 0), pixels.Color(BRIGHTNESS, 0, BRIGHTNESS), pixels.Color(0, BRIGHTNESS, BRIGHTNESS), pixels.Color(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS)};
  
uint64_t RF_freq;   // RF frequency (Hz)
int C_freq = 0;  //FREQ_x: In this case, FREQ_0 is sellected as the initial freqency.
int Tx_Status = 0; //0=RX, 1=TX
int Tx_Start = 0;  //0=RX, 1=TX
int Tx_modulation = 0; //flag for freqency modulation in transmitting
int16_t Tx_modulation_counter = 0; //counter for inhibt the frequency change faster than 2mS in trasnmitting freqency modulation

//Audio signal frequency determination
int16_t mono_prev=0;  
float delta_prev=0;
int16_t sampling=0;
int16_t cycle=0;
float cycle_period[16];

//ADC offset for Reciever
int16_t adc_offset = 0;   

//USB Audio modified from pdmAudio.h in https://github.com/tierneytim/Pico-USB-audio
class rp2040Audio {
  public:
    //Constructor
    rp2040Audio();
    void USB_UAC();
    void USBread();
    void USBread2();
    void USBwrite(int16_t left,int16_t right);
    int16_t monodata[24];
  private:
    USBAudio* audio;
    uint8_t myRawBuffer[96]; //24 sampling (= 0.5 ms at 48000 Hz sampling) data are send from PC in one packet.
    uint8_t myRawBuffer2[24]; //6 sampling = 8 kHz at 48 kHz sampling
    int16_t pcBuffer16[48];  //24 sampling date are written to PC in one packet.
    uint16_t pcCounter=0;
    uint16_t nBytes=0;
 };

rp2040Audio rp;

void setup() {
  //Digital pin setting ----- 
  pinMode(A0, INPUT); //ADC input pin
  pinMode(pin_SW, INPUT_PULLUP); //SW (freq. change)
  pinMode(pin_RX, OUTPUT); //RX →　High, TX →　Low (for RX switch)
  pinMode(pin_TX, OUTPUT); //TX →　High, RX →　Low (for Driver switch)
  pinMode(pin_RED, OUTPUT); //On →　LOW (for RED LED)
  pinMode(pin_GREEN, OUTPUT); //On →　LOW (for GREEN LED)
  pinMode(pin_BLUE, OUTPUT); //On →　LOW (for BLUE LED)
  pinMode(pin_LED_POWER, OUTPUT); //NEOPIXEL LED

  //I2c initialization-----  
  Wire.begin();

  //USB Audio initialization
  rp.USB_UAC();

  //si5351 initialization-----  
  int32_t cal_factor = -11800;
  RF_freq = Freq_table[C_freq];
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0); 
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK0);  //for TX
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.set_freq(RF_freq*100ULL, SI5351_CLK1);  //for RX
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK1, 0);

  //NEOPIXEL LED  initialization-----
  digitalWrite(pin_LED_POWER, HIGH);  //NEOPIXEL LED ON
  pixels.begin();  // initialize the NEOPIXEL
  pixels.setPixelColor(0, colors[C_freq]);
  pixels.show();

  //transciver initialization-----
  si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
  si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
  digitalWrite(pin_TX,0);  //TX off
  digitalWrite(pin_RX,1);  //RX on
  digitalWrite(pin_RED, HIGH);
  digitalWrite(pin_GREEN, LOW); //Green LED ON
  digitalWrite(pin_BLUE, HIGH);

  //read the DC offset value of ADC input (about 0.6V)----- 
  delay(500);
  for (int i=0; i<10; i++) {
    adc_offset = adc();
    delayMicroseconds(25);
  }
}

void loop() {
  if (Tx_Start==0) receiving();
  else transmitting();
}

void transmitting(){
  int64_t audio_freq;
  rp.USBread();
  for (int i=0;i<24;i++){
    int16_t mono = rp.monodata[i];
    if ((mono_prev < 0) && (mono > 0)) {
      int16_t diference = mono - mono_prev;
      // x=0付近のsin関数をテーラー展開の第1項まで(y=xで近似）
      float delta = (float)mono_prev / (float)diference;
      float period = (1.0 + delta_prev) + (float)sampling - delta;
      audio_freq = AUDIOSAMPLING*100.0/period; // in 0.01Hz    
      if ((audio_freq>20000) && (audio_freq<300000)){
        Tx_modulation = 1;
        cycle_period[cycle]=audio_freq;
        cycle++;
      }
      delta_prev = delta;
      sampling=0;
      mono_prev = mono;     
    }
    else {
      sampling++;
      mono_prev = mono;
    }
  }
  if ((Tx_modulation == 1) && (Tx_modulation_counter > 9)){
    audio_freq = 0;
    for (int i=0;i<cycle;i++){
      audio_freq += cycle_period[i];
    }
    audio_freq = audio_freq / (float)cycle;
    transmit(audio_freq);
    Tx_modulation = 0;
    Tx_modulation_counter = 0;
    cycle = 0;
  }
  Tx_modulation_counter++;
  if (sampling > 4800){
    recieve();
    Tx_Start = 0;
  }
  for (int i=0;i<24;i++){
    //rp.USBwrite(rx_adc, rx_adc);
    rp.USBwrite(rp.monodata[i], rp.monodata[i]);
  }
}

void receiving() {
  rp.USBread2();  // read in the USB Audio buffer (myRawBuffer2) for control of ADC timing and detection of the transmitting
  if (rp.monodata[0] > 0){
    Tx_Start = 1;
  }
  freqChange();
  int16_t rx_adc = adc() - adc_offset;    //read ADC data
  // write stereo data to computer
  for (int i=0;i<6;i++){
    rp.USBwrite(rx_adc, rx_adc);
  }
}

void transmit(int64_t freq){
  if (Tx_Status==0){
    digitalWrite(pin_RX,0);   //RX off
    digitalWrite(pin_TX,1);   //TX on
    si5351.output_enable(SI5351_CLK1, 0);   //RX osc. off
    si5351.output_enable(SI5351_CLK0, 1);   //TX osc. on
    Tx_Status=1;
    digitalWrite(pin_RED, 0);
    digitalWrite(pin_GREEN, 1);
    //digitalWrite(pin_BLUE, 1);
  }
  si5351.set_freq((RF_freq*100 + freq), SI5351_CLK0);  
}

void recieve(){
  if (Tx_Status==1){
    digitalWrite(pin_TX,0);  //TX off
    digitalWrite(pin_RX,1);  //RX on
    si5351.output_enable(SI5351_CLK0, 0);   //TX osc. off
    si5351.set_freq(RF_freq*100, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK1, 1);   //RX osc. on
    Tx_Status=0;
    digitalWrite(pin_RED, 1);
    digitalWrite(pin_GREEN, 0);
    //digitalWrite(pin_BLUE, 1);
  }
}

void freqChange(){
  if (digitalRead(pin_SW)==LOW){
    delay(100);
    if  (digitalRead(pin_SW)==LOW){
      C_freq++;
      if  (C_freq >= N_FREQ){
        C_freq = 0;
      }
    }
    RF_freq = Freq_table[C_freq];
    si5351.set_freq(RF_freq*100, SI5351_CLK1);
    //NEOPIXEL LED change
    pixels.setPixelColor(0, colors[C_freq]);
    pixels.show();
    delay(100);
    adc_offset = adc();
  }
}

int16_t adc() {
  int16_t adc = 0;
  for (int i=0;i<13;i++){
    adc += analogRead(A0);
  }  
  return adc;
}

//USB Audio modified from pdmRP2040.cpp in https://github.com/tierneytim/Pico-USB-audio
rp2040Audio::rp2040Audio() {
}
void rp2040Audio::USB_UAC() {
 audio= new USBAudio(true, AUDIOSAMPLING, 2, AUDIOSAMPLING, 2);
}
void rp2040Audio::USBread() {
  if (audio->read(myRawBuffer, sizeof(myRawBuffer))) {
    int16_t *lessRawBuffer = (int16_t *)myRawBuffer;
    for (int i = 0; i < 24; i++) {
      // the left value;
      int16_t outL = *lessRawBuffer;
      lessRawBuffer++;
      // the right value
      int16_t outR = *lessRawBuffer;
      lessRawBuffer++;
      //mono value
      int16_t mono = (outL + outR) / 2;
      monodata[i] = mono;
    }
  }
}

void rp2040Audio::USBread2() {
  if (audio->read(myRawBuffer2, sizeof(myRawBuffer2))) {
    int16_t *lessRawBuffer = (int16_t *)myRawBuffer2;
    int16_t outL = *lessRawBuffer;
    lessRawBuffer++;
    int16_t outR = *lessRawBuffer;
    monodata[0] = (outL + outR) / 2;
  }
}

void rp2040Audio::USBwrite(int16_t left,int16_t right) {
  if(nBytes>95){
    uint8_t *pcBuffer =  (uint8_t *)pcBuffer16;
    audio->write(pcBuffer, 96);
    pcCounter =0;
    nBytes =0;
  }
  pcBuffer16[pcCounter]=left;
  pcCounter++;
  nBytes+=2;
  pcBuffer16[pcCounter]=right;
  pcCounter++;
  nBytes+=2;
}
