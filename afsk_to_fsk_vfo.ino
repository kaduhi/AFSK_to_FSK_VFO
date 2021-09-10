//update 4/1/21 band tab
// 4/6/21 added Display shift
// 4/7/21 updated band FT8 band table. 
// 4/8/21 added latest FT8 audio routines.
// 4/9/21 A0 used to enable tx for testing.
// 4/12/21 fixed tuning rate stuck on 10 Hz in kHz display mode.  
// 4/16/21 fixed 80M start up freq,fixed 17M resistor sense voltage.  
// 4/21/21 adjusted FSK updating rate for 45.45bps RTTY.
// 4/25/21 improve resolution of FSK from 1Hz to 0.0625Hz
// 5/6/21  added firmware rev# to boot up. 
// 5/6/21 added auto incerment of calibbration tuning
// 6/26/21 fixed no RIT shift on transmit in MultiDC mode
// 6/30/21 fixed RIT not initiallized on band change. 

//FT8 fsk mods by Kazuhisa "Kazu" Terasaki AG6NS 
//Si5351 routine by Jerry Gaffke, KE7ER
//everything else by Steven Weber, KD1JV  

#define ENABLE_VOICE_SSB_TX             1
#define AFSK_TO_FSK_VFO                 1

#include <EEPROM.h>
#if !ENABLE_VOICE_SSB_TX
#include "Wire.h"
#endif
//*********************************************************************
//USE this configuration for MULTI-DC VFO
//#define MULTIDC_VFO                     1
//#define FT8_VFO                         0
//*********************************************************************
// Use this configuration for FT-8 VFO
#define MULTIDC_VFO                     0
#define FT8_VFO                         1
//*********************************************************************

#define CPU_CLOCK_FREQ                  16000000                    // 16MHz


// si5451 definations 
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351BX_XTALPF 3               // 1:6pf  2:8pf  3:10pf

#define SI5351BX_XTAL 25002700          // Crystal freq in Hz
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz

#define SI5351BX_CRYSTAL_CAL_STEP   5   // Crystal frequency calibration step = 5Hz on 25MHz
#define SI5351_CALIBRATED_CRYSTAL_FREQ  (si5351bx_vcoa / SI5351BX_MSA)

// User program may have reason to poke new values into these 3 RAM variables
uint32_t si5351bx_vcoa = (SI5351BX_XTAL*SI5351BX_MSA);  // 25mhzXtal calibrate
uint8_t  si5351bx_rdiv = 0;             // 0-7, CLK pin sees fout/(2**rdiv) 
uint8_t  si5351bx_drive[3] = {0,0,0};   // 0=2ma 1=4ma 2=6ma 3=8ma for CLK 0,1,2

uint8_t  si5351bx_clken = 0xFF;         // Private, all CLK output drivers off


//Si5351 si5351;
#define  SLED5  9
#define  SLED4  10
#define  SLED3  11
#define  SLED2  12
#define  SLED1  13


/* 7 segment LED bit mask:
        0x04
    0x08    0x10
        0x01
    0x80    0x02
        0x40
                0x20
 */

#define   LED_N_0  0xde
#define   LED_N_1  0x12
#define   LED_N_2  0xd5
#define   LED_N_3  0x57
#define   LED_N_4  0x1b
#define   LED_N_5  0x4f
#define   LED_N_6  0xcf
#define   LED_N_7  0x16
#define   LED_N_8  0xdf
#define   LED_N_9  0x5f
#define   LED_r    0x81
#define   LED_neg   0x01
#define   LED_C     0xcc
#define   LED_n     0x83
#define   LED_E     0xcd
#define   LED_F     0x8d
#define   LED_t     0xc9
#define   LED_o     0xc3 
#define   LED_d     0xd3
#define   LED_A     0x9f
#define   LED_L     0xc8 
#define   LED_P     0x9d
#define   LED_b     0xcb
#define   LED_U     0xda
#define   LED_BLANK 0x00
#define   LED_question 0x95
#define   LED_point 0x20



//flag definitions 
#define     RIT_ON 0x01
#define     RIT_FC 0xfe
#define     E_sw    2
#define     U_sw    0
#define     D_sw    1
#define     ENC_A   A0
#define     ENC_B   A1

// register names 
byte        SRtemp;
byte        Etemp; 
byte        BANDpointer; 
byte        outofband =0;
byte        shift_display =0;  
byte        ritflag = 0;
//byte        IF_flag = 0; 
byte        EncoderFlag = 0;
byte        c = 0;
byte        d = 0;
byte        cLast =1; 

volatile unsigned long tcount;

int                Eadr;
volatile byte      sw_inputs =0 ; 


volatile byte      digit1 = 0xff;
volatile byte      digit2 = 0xff;
volatile byte      digit3 = 0xff;
volatile byte      digit4 = 0xff;
volatile byte      digit5 = 0xff;
volatile byte      digitX = 0;

volatile byte      d1temp = 0;

volatile byte    digit_counter = 1;
    //frequency tuning 
int         stepSize;   // tuning rate pointer 
int         tempSS; 

long int    stepK;      // temp storage of freq tuning step
long int    tempSK; 
int         Bvoltage;
long int    Fstep10  = 10 ;                // 10 Hz step
long int    Fstep100 = 100;
long int    Fstep1K =  1000;
long int    Fstep5K =  5000;
long int    Fstep10K = 10000;
long int    Fstep100K = 100000;
                                          
//encoder 


unsigned  long  temp1;
unsigned  long  frequency;
unsigned  long  freq_result;  
unsigned  long  OPfreq;
//unsigned  long  IF_FREQ;
unsigned  long  RX_FREQ; 
unsigned  long  RITtemp; 
unsigned  long  temp ; 
unsigned  long  cal_value;
 //********************************************** 

// registers for limit testing

unsigned long  low_band_limit;  //low limit, band tuning
unsigned long  high_band_limit; //high limit, band tuning 
unsigned long  low_absolute_limit = 500000;
unsigned long  high_absolute_limit = 30000000;

#if ENABLE_VOICE_SSB_TX
volatile uint8_t gTick4000 = 0;
#endif //ENABLE_VOICE_SSB_TX

// EEPROM adderess 
// 0 = band pointer
// 4-7 = cal data
// 8-11 = IF offset


void setup() {
    //switch inputs

    DDRB = 0xff;
    DDRD = 0Xff; 
    
#if MULTIDC_VFO
    pinMode(A3, INPUT_PULLUP);
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
#endif

#if FT8_VFO
pinMode(A0, INPUT_PULLUP); 
pinMode(A1, OUTPUT); 
pinMode(A3, INPUT); 
digitalWrite(A1, HIGH); 
#endif 

    noInterrupts();
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;

    OCR2A = 125;    // 1ms interval
    TCCR2A = 0x02;  // WGM21=1, WGM20=0
    TCCR2B = 0x05;  // WGM22=0, CS=0b100 (16MHz / 128 = 2MHz)
    TIFR2 = 0x07;   // clear interrupt flags
    TIMSK2 |= (1 << OCIE2A);  // enable Output Compare Match A Interrupt
    interrupts();   

#if MULTIDC_VFO
    digit5 = LED_d;
    digit4 = LED_C;
    digit3 = LED_BLANK;
    digit2 = LED_N_1;
    digit1 = LED_N_3;
#endif    

#if FT8_VFO
    digit5 = LED_F;
    digit4 = LED_N_8;
    digit3 = LED_BLANK;
#if !ENABLE_VOICE_SSB_TX
    digit2 = LED_N_1;
    digit1 = LED_N_3; 
#else //ENABLE_VOICE_SSB_TX
    digit2 = LED_question;
    digit1 = LED_question;
#endif //!ENABLE_VOICE_SSB_TX
#endif 

delay(1000);



    si5351bx_init(); 
    cal_data();    //load calibration data  
  
#if MULTIDC_VFO 
    stepK =Fstep100;
    stepSize = 2;
    shift_display = 1;   
    int_band();
#endif

#if FT8_VFO
    stepK = Fstep1K; //default tuning rate
    stepSize = 3; 
    FT8_band_select(); 
#endif  

#if FT8_VFO
     
#if !ENABLE_VOICE_SSB_TX
    // initialize analog comparator
    ADCSRA = 0x00;          // ADEN=0
    ADCSRB = (1 << ACME);   // enable Analog Comparator Multiplexer
    ADMUX = 0x02;           // MUX=0b0010 (ADC2)
    ACSR = (1 << ACBG) | (1 << ACI) | (1 << ACIS1) | (0 << ACIS0);   // enable Bandgap Voltage Reference, clear Analog Comparator Interrupt flag, ACIS=0b10 (Comparator Interrupt on Falling Output Edge)
    //ACSR |= (1 << ACIE);   // enable Analog Comparator Interrupt

    // initialize TIMER1
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1B = (1 << ICNC1) | (0 << ICES1) | (1 << CS10);// enable Input Capture Noise Canceler, Input Capture Edge Select (Falling), CS=0b001 (16MHz / 1)
    ICR1 = 0;
    ACSR |= (1 << ACIC);      // enable Analog Comparator Input Capture
    TIFR1 = 0x027;            // clear interrupt flags
    TIMSK1 |= (1 << ICIE1) | (1 << TOIE1);   // enable Input Capture Interrupt, enable Timer1 Overflow Interrupt
    interrupts();
#else //ENABLE_VOICE_SSB_TX
    initSSBTX();
#endif //!ENABLE_VOICE_SSB_TX
#endif 
  

    delay(1000); //let things settle down a bit
    displayfreq();
    PLLwrite();
}    
void loop() {

#if FT8_VFO  
    static uint16_t sPrevTimer1 = 0;
    bool processUI = false;
#if !ENABLE_VOICE_SSB_TX
    if ((TCNT1 - sPrevTimer1) >= (CPU_CLOCK_FREQ / 1000)) {
        sPrevTimer1 = TCNT1;
        processUI = true;
    }
    else {
        goto SKIP_UI;
    }
#else //ENABLE_VOICE_SSB_TX
    static uint8_t sPrevTick4000 = 0;
    uint8_t currTick4000 = gTick4000;
    if (((uint8_t)(currTick4000 - sPrevTick4000)) >= 4) {
        sPrevTick4000 = currTick4000;
        processUI = true;
    }
    else {
        goto SKIP_UI;
    }
    if (isTransmitting()) {
        refreshDisplay();
        goto SKIP_UI;
    }
#endif //!ENABLE_VOICE_SSB_TX
#endif

    if (bitRead(sw_inputs, E_sw) == LOW) {  // test for switch closed
        timedswitch();
        debounceE();
    }
    if (bitRead(sw_inputs, U_sw) == LOW) {  // test tune up flag
        Tune_UP();
    } 
    if (bitRead(sw_inputs, D_sw) == LOW) {  // test tune down flag
        Tune_DWN();
    }

#if FT8_VFO
    if (digitalRead(A0) == LOW) {transmit();}    
#endif

#if MULTIDC_VFO
    if (digitalRead(A3)==LOW) {
        transmit();
    }

    if (EncoderFlag == 1) {     // test tune up flag
        Tune_UP();
    }
    if (EncoderFlag == 2) {     // test tune down flag 
        Tune_DWN();
    }
#endif

#if FT8_VFO
SKIP_UI:
    processAudioInput(processUI);
#endif

}


//**************************************************************
//end of switch polling loop
//**************************************************************

void debounceE(){
    while (bitRead(sw_inputs, E_sw) == LOW) { }
}

void debounceU(){
    while (bitRead(sw_inputs, U_sw) == LOW) { }
}

void debounceD(){
    while (bitRead(sw_inputs, D_sw) == LOW) { } 
}

#if FT8_VFO
void transmit() { 
        digitalWrite(A1, LOW);
        si5351bx_setfreq(1, OPfreq); 
        while (digitalRead(A0) == LOW){loop;} 
        si5351bx_setfreq(0, OPfreq); //update clock chip with VFO 
        digitalWrite(A1, HIGH); 
}
#endif

#if MULTIDC_VFO

void transmit() {
 // if (ritflag == 0) {RITtemp = OPfreq;} 
        si5351bx_setfreq(1, RITtemp); 
        while (digitalRead(A3) == LOW){loop;} 
        si5351bx_setfreq(1, OPfreq); //update clock chip with VFO 
}
#endif

void timedswitch() {  
    unsigned long time0;
    unsigned long duration;

    time0 = tcount;

    do {
        duration = (unsigned long)tcount - time0;   // calculate how long the button has been pushed
   
        if (duration == 12000) { //cal mode
            digit5 = LED_C;
            digit4 = LED_A;
            digit3 = LED_L;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }

        if (duration == 2000) {
            digit5 = LED_d;
            digit4 = LED_N_5;
            digit3 = LED_BLANK;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
        
               
#if MULTIDC_VFO  

        if (duration == 3000) {// band
            digit5 = LED_N_6;
            digit4 = LED_n;
            digit3 = LED_BLANK;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }

        if (duration == 1000) { //rit mode
            digit5 = LED_r;
            digit4 = LED_BLANK;
            digit3 = LED_BLANK;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        } 

  #endif       
#if ENABLE_VOICE_SSB_TX
        if (duration == 4000) { // TX mode
            digit5 = LED_N_5;
            digit4 = LED_N_5;
            digit3 = LED_b;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
#endif //ENABLE_VOICE_SSB_TX

        if (duration == 14000) { //clear eerom 
            digit5 = LED_r;
            digit4 = LED_E;
            digit3 = LED_N_5;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
    } while (bitRead(sw_inputs, E_sw) != 1);

    if (duration >= 14000) {
        reset_flash();
    }
    else if (duration >= 12000) {
        calibration();
    }
#if ENABLE_VOICE_SSB_TX
    else if (duration >= 4000) {
        changeTXMode();
    }
#endif //ENABLE_VOICE_SSB_TX

#if MULTIDC_VFO 
    else if (duration >= 3000) {
        changeBand();
    }
#endif

    else if (duration >=2000) {
      toggleDS();
    }

#if MULTIDC_VFO
    else if (duration >= 1000)  {
        RIT();
    }    
#endif

    else if (duration >= 10) {
        nextFstep();
    }
}


void Tune_UP() {
    FREQ_incerment();
    delay(150);
}

void Tune_DWN() {    
    FREQ_decerment();
    delay(150); 
}


// adjust the operating frequency
void FREQ_incerment() {
    EncoderFlag = 0;
    if (OPfreq >= high_absolute_limit) {
        return;
    } 

    OPfreq += stepK;   // add frequenc tuning step to frequency word

    outofband = 0;
    if (OPfreq > high_band_limit) {  // band tuning limits
        outofband = 1;
    }
    if (OPfreq < low_band_limit) {
      outofband = 1;
    }

#if MULTIDC_VFO
    if (ritflag & RIT_ON) {
        RITdisplay();
    }
    else {
        displayfreq();
    }
#endif    

#if FT8_VFO
    displayfreq();
#endif    
    
    PLLwrite();    
}

void FREQ_decerment() {
    EncoderFlag = 0; 
    if (OPfreq <= low_absolute_limit) {
        return;
    } 

    OPfreq -= stepK;

    outofband = 0;
    if (OPfreq < low_band_limit) {  // band tuning limits
        outofband = 1;
    }
    if (OPfreq > high_band_limit){
      outofband = 1; 
    }

#if MULTIDC_VFO
    if (ritflag & RIT_ON) {
        RITdisplay();
    }
    else {
        displayfreq();
    }
#endif

#if FT8_VFO
    displayfreq();
#endif 

    PLLwrite(); 
}


//toggle tuning step rate
void  nextFstep () {

#if MULTIDC_VFO 
    if (ritflag == 1) {
        flip_sideband();
    }
    else {
        incStep();
    }
#endif

#if FT8_VFO
    incStep();
#endif      
}

void incStep() { 
    if (shift_display == 1) {
        allsteps();
    } 
    else {
        bigsteps();
    }
}


void allsteps() {
    ++stepSize; 
   if (stepSize == 7){ stepSize = 1;} 
  

    switch(stepSize) {
    case 1:
        stepK = Fstep10;
        d1temp = digit1;
        digit1 = LED_BLANK;    
        delay(100); 
        digit1 = d1temp;
        break;

    case 2:
        stepK = Fstep100;
        d1temp = digit2;
        digit2 = LED_BLANK;
        delay(100); 
        digit2 = d1temp;
        break; 

    case 3:
        stepK = Fstep1K;
        d1temp = digit3;
        digit3 = LED_BLANK;  
        delay(100); 
        digit3 = d1temp;
        break; 

    case 4:
        stepK = Fstep5K;
        d1temp = digit3;
        digit3 = LED_BLANK;  
        delay(100); 
        digit3 = d1temp;
        delay(100);
        digit3 = LED_BLANK;
        delay(100);
        digit3 = d1temp;
        break;    

    case 5:
        stepK = Fstep10K;
        d1temp = digit4;
        digit4 = LED_BLANK;  
        delay(100); 
        digit4 = d1temp;
        break;

    case 6:
        stepK = Fstep100K;
        d1temp = digit5;
        digit5 = LED_neg;  
        delay(100); 
        digit5 = d1temp;
        break;
    }
}

void bigsteps() {
    if (++stepSize > 6) {
        stepSize = 3;
    }
    
    switch(stepSize) {
    case 1:
        break;

    case 2:
        break;

    case 3:  stepK = Fstep1K;
        d1temp = digit1;
        digit1 = LED_BLANK;  
        delay(100); 
        digit1 = d1temp;
        break; 

    case 4:
        stepK = Fstep5K;
        d1temp = digit1;
        digit1 = LED_BLANK;  
        delay(100); 
        digit1 = d1temp;
        delay(100);
        digit1 = LED_BLANK;
        delay(100);
        digit1 = d1temp;
        break;    

    case 5:
        stepK = Fstep10K;
        d1temp = digit2;
        digit2 = LED_BLANK;  
        delay(100); 
        digit2 = d1temp;
        break;

    case 6:
        stepK = Fstep100K;
        d1temp = digit3;
        digit3 = LED_BLANK; 
        delay(100); 
        digit3 = d1temp;
        break;
    }
}


void  toggleDS(){

    if (shift_display == 1) {shift_display = 0; stepSize = 3; stepK = Fstep1K;}
    else shift_display = 1;
    displayfreq();
    debounceE(); 
}


#if MULTIDC_VFO 
void flip_sideband() {
    
    if (OPfreq > RITtemp) {
        OPfreq = RITtemp - 600;
        digit4 = LED_neg;
    }
    else {
        OPfreq = RITtemp + 600;
        digit4 = LED_BLANK;
    }
    PLLwrite(); 
    debounceE(); 
}


void RIT() {
    if (ritflag == 1) {
        RIText();
    }
    else {
        RITenable();
    }
}

void RITenable(){
    ritflag = 1;
    tempSS = stepSize;
    tempSK = stepK; 
    stepSize = 1;
    stepK = 10; 
    RITtemp = OPfreq;
    OPfreq = RITtemp +600;
    
    PLLwrite();
    RITdisplay();
    debounceE();
}

void RIText() {
    ritflag =0;
    stepK = tempSK;
    stepSize = tempSS; 
    OPfreq = RITtemp;
    PLLwrite();
    displayfreq();
    debounceE();
}

void RITdisplay() {
    unsigned long RITresult;

    if (RITtemp >= OPfreq) {
        RITresult = RITtemp - OPfreq;
        digit4 = LED_neg;
    }
    else {
        RITresult = OPfreq - RITtemp; 
        digit4 = LED_BLANK;   
    }

    frequency = RITresult;
    freq_result = frequency%10000;
    freq_result = freq_result/1000;
    hex2seg(); 
    digitX = digitX + LED_point;
    digit3 = digitX;
    freq_result = frequency%1000;
    freq_result = freq_result/100;
    hex2seg(); 
    digit2 = digitX;
    freq_result = frequency%100;
    freq_result = freq_result/10;
    hex2seg();
    digit1 = digitX;    
}

#endif 

////////////////////////////////////////////////
//This breaks up the frequency value into decades and then converts the result 
//hex value into the LED 7 seg map. 
///////////////////////////////////////////////

void    displayfreq()
{
   if (shift_display == 1){FREQdisplay(); return;}
   else {OFB_F_display();}
}

   void OFB_F_display()

{
          frequency = OPfreq;    
          freq_result = frequency/10000000;
          hex2seg();
          if (freq_result == 0){digitX= LED_BLANK;}
          digit5 = digitX;

          freq_result = frequency%10000000;
          freq_result = freq_result/1000000;
          hex2seg();
          digitX = digitX + LED_point;
          digit4 = digitX;
         
        freq_result = frequency%1000000;     //get the 100,000 kHz digit by first getting the remainder 
        freq_result = freq_result / 100000;  //divide the remainder by 100,000 to get the MSD
        hex2seg();                           //convert the result to the 7 segment code
        digit3 = digitX;  
        
        freq_result = frequency%100000;      //repeat the process for 10K, 1K and 100 Hz digits
        freq_result = freq_result/10000;
        hex2seg();
        digit2 = digitX;
        
        freq_result = frequency%10000;
        freq_result = freq_result/1000;
        hex2seg(); 
        digit1 = digitX;
}


void FREQdisplay(){
        frequency = OPfreq;              
        freq_result = frequency%1000000;     //get the 100,000 kHz digit by first getting the remainder 
        freq_result = freq_result / 100000;  //divide the remainder by 100,000 to get the MSD
        hex2seg();                           //convert the result to the 7 segment code
        if (freq_result == 0){digitX = LED_BLANK;}
        digit5 = digitX;  
        
        freq_result = frequency%100000;      //repeat the process for 10K, 1K and 100 Hz digits
        freq_result = freq_result/10000;
        hex2seg();
        digit4 = digitX;
        
        freq_result = frequency%10000;
        freq_result = freq_result/1000;
        hex2seg(); 
        digitX = digitX + LED_point;
        digit3 = digitX;
        
        freq_result = frequency%1000;
        freq_result = freq_result/100;
        hex2seg();
        digit2 = digitX;
        
        freq_result = frequency%100;
        freq_result = freq_result/10;
        hex2seg();
        digit1 = digitX;
}

void  hex2seg()
{
        if (freq_result == 0) {digitX = LED_N_0;} //this is the conversion table
        if (freq_result == 1) {digitX = LED_N_1;}
        if (freq_result == 2) {digitX = LED_N_2;}
        if (freq_result == 3) {digitX = LED_N_3;}
        if (freq_result == 4) {digitX = LED_N_4;}
        if (freq_result == 5) {digitX = LED_N_5;}
        if (freq_result == 6) {digitX = LED_N_6;}
        if (freq_result == 7) {digitX = LED_N_7;}
        if (freq_result == 8) {digitX = LED_N_8;}
        if (freq_result == 9) {digitX = LED_N_9;}
}

/*
 * 
 * timer outside of the normal Ardinu timers
 * does keyer timing and port D mulitplexing for display and 
 * switch inputs. 
 */

ISR(TIMER2_COMPA_vect)
{  
    ++tcount;

    if (++digit_counter > 4) {
        digit_counter = 0;
    }

    switch(digit_counter) {
    

    case 0: 
        digitalWrite(SLED5, HIGH);
        readswitches(); 
        PORTD = digit1;
        digitalWrite(SLED1, LOW);
        break;

    case 1: 
        digitalWrite(SLED1, HIGH);
        readswitches(); 
        PORTD = digit2;
        digitalWrite(SLED2, LOW);
        break;

    case 2:  
        digitalWrite(SLED2, HIGH);
        readswitches();  
        PORTD = digit3;
        digitalWrite(SLED3, LOW);
        break;

    case 3: 
        digitalWrite(SLED3, HIGH);
        readswitches(); 
        PORTD = digit4;
        digitalWrite(SLED4, LOW);
        break;

    case 4: 
        digitalWrite(SLED4, HIGH);
        readswitches(); 
        PORTD = digit5;
        digitalWrite(SLED5, LOW);
        break;
    }
}

void readswitches() {
    //read the switches and encoder here
    DDRD = 0x00;
    PORTD = 0xff;
    digitalWrite(8, LOW);
    sw_inputs = PIND;
    digitalWrite(8, HIGH);
    DDRD = 0xff;
    
#if MULTIDC_VFO
    c = digitalRead(ENC_A);  // read encoder clock bit
    if (c != cLast) {   // call if changed
        encoder();
    }
#endif
}
                                 
// encoder, test for direction only on 0 to 1 clock state change

#if MULTIDC_VFO
void encoder() {
    if (cLast ==0) {
        d = digitalRead(ENC_B);    
        if (d == LOW) {     // if low
            EncoderFlag = 2;
        }
        else {              // if high
            EncoderFlag = 1;
        }
    }
    cLast = c;  // store new state of clock  
}
#endif


/*
 * output the frequency data to the clock chip.
 */

void PLLwrite() {   

#if FT8_VFO
    si5351bx_setfreq(0, OPfreq); //update clock chip with VFO frequency. 
#endif
       
#if MULTIDC_VFO    
    si5351bx_setfreq(1, OPfreq); //update clock chip with VFO
#endif      
}

/*
 * Ref oscillator frequency is calibrated first
 * 10MHz signal outputted on CLOCK 0 
 * tune to equal exactly 10.000,000 MHz
 * adjusted in 1 Hz steps. 
 */

 
void calibration(){
    displayfreq();
    debounceE();
    digit5 = LED_C;

    while (bitRead(sw_inputs, E_sw) == HIGH) {
        if (bitRead(sw_inputs, U_sw) == LOW) {
            ADJ_UP();
        }
        if (bitRead(sw_inputs, D_sw) == LOW) {
            ADJ_DWN();
        } 
    }

    temp = si5351bx_vcoa;  //store the cal value 
    cal_value = temp;  //store the cal value 
    EEPROM.write(7, temp);
    temp = cal_value >>8;
    EEPROM.write(6, temp); 
    temp = cal_value >>16;
    EEPROM.write(5, temp);
    temp  = cal_value >>24;
    EEPROM.write(4,temp); 

    displayfreq();  
    debounceE(); 
}

void ADJ_UP(){
      si5351bx_vcoa = si5351bx_vcoa - (SI5351BX_CRYSTAL_CAL_STEP * SI5351BX_MSA);
      calwrite();
}

void ADJ_DWN() {
      si5351bx_vcoa = si5351bx_vcoa + (SI5351BX_CRYSTAL_CAL_STEP * SI5351BX_MSA);
      calwrite();
}

void calwrite() {
#if ENABLE_VOICE_SSB_TX
      setCalibratedXtalFrequency(SI5351_CALIBRATED_CRYSTAL_FREQ);
#endif //ENABLE_VOICE_SSB_TX
      si5351bx_setfreq(0, OPfreq);
#if  MULTIDC_VFO
      si5351bx_setfreq(1, OPfreq);
#endif       

    delay (100); 
}

void cal_data(){
      temp1 = si5351bx_vcoa;
      temp =0; 
      temp = EEPROM.read(4);
      cal_value = temp;
      cal_value = cal_value << 8;
      temp = EEPROM.read(5);
      cal_value = cal_value + temp;
      cal_value = cal_value << 8;
      temp = EEPROM.read(6);
      cal_value = cal_value + temp;
      cal_value = cal_value << 8;
      temp = EEPROM.read(7);
      cal_value = cal_value + temp;
      si5351bx_vcoa = cal_value;
      if (cal_value == 0xffffffff) si5351bx_vcoa = temp1;
}


#if MULTIDC_VFO
void changeBand(){
      digit5 = LED_N_6;
      digit4 = LED_n; 
      digit3 = LED_BLANK; 
      get_band_MDC();
      outofband = 0;
      shift_display = 0;
    debounceE(); 
      
    do {
      if (bitRead(sw_inputs,U_sw) !=1) {nextband();}
      if (bitRead(sw_inputs,D_sw) !=1) {nextbandD();}
    }
    while (bitRead(sw_inputs,E_sw) !=0);

    EEPROM.write(0,BANDpointer); 
    
    debounceE(); 
    displayfreq();
    PLLwrite();  
}

void nextband () {
    ++BANDpointer ;
    if (BANDpointer == 4) {(BANDpointer = 1);}
    get_band_MDC();
    debounceU(); 
}

void nextbandD() {
   --BANDpointer ;
    if (BANDpointer == 0) {(BANDpointer = 3);}
    get_band_MDC();
    debounceD(); 
}

#endif

#if FT8_VFO

void FT8_band_select() {
  
  Bvoltage = analogRead(3);
  if (Bvoltage < 20)   {BANDpointer = 8; get_band(); return;} 
  if (Bvoltage < 50)   {BANDpointer = 1; get_band(); return;} 
  if (Bvoltage < 125)   {BANDpointer = 9; get_band(); return;}
  if (Bvoltage < 200)  {BANDpointer = 6; get_band(); return;}
  if (Bvoltage < 350)  {BANDpointer = 5; get_band(); return;}
  if (Bvoltage < 550)  {BANDpointer = 4; get_band(); return;}
  if (Bvoltage < 630)  {BANDpointer = 3; get_band(); return;}
  if (Bvoltage < 725)  {BANDpointer = 7; get_band(); return;}
  if (Bvoltage < 800)  {BANDpointer = 10; get_band(); return;}
  if (Bvoltage < 900)  {BANDpointer = 2;  get_band(); return;}
}
#endif

#if MULTIDC_VFO
void int_band() {
  BANDpointer = EEPROM.read(0);
  if (BANDpointer == 0xff) {BANDpointer= 1;} 
  get_band_MDC(); 
}

#endif


#if FT8_VFO

void get_band() {
  digit5 = LED_N_6;
  digit4 = LED_n; 
  digit3 = LED_BLANK;
switch(BANDpointer) {
case 1: 
      BAND16();
      break;
      
case 2:
      BAND80();
      break;

case 3: 
      BAND60() ;
      break ;
      
case 4: 
      BAND40() ;
      break ;
      
case 5: 
      BAND30() ;
      break ;
      
case 6: 
      BAND20() ;
      break ;
      
case 7: 
      BAND17() ;
      break ;

case 8:
      BAND15() ;
      break;

case 9:
      BAND12();
      break;

case 10:
      BAND10();
      break;
}
}

void BAND16() {  
    digit2 = LED_N_1;
    digit1 = LED_N_6;
    low_band_limit = 1800000;
    high_band_limit = 2000000;
    OPfreq = 1840000;    
}

void BAND80() {  
    digit2 = LED_N_8;
    digit1 = LED_N_0;
    low_band_limit = 3500000;
    high_band_limit = 4000000;
    OPfreq = 3573000;    
}

void BAND60() {  
    digit2 = LED_N_6;
    digit1 = LED_N_0;
    low_band_limit = 5351500;
    high_band_limit = 5366500;
    OPfreq = 5357000;    
}

void BAND40() { 
    digit2 = LED_N_4;  
    digit1 = LED_N_0; 
    low_band_limit = 7000000;
    high_band_limit = 7300000;
    OPfreq = 7074000;
}

void BAND30() {  
    digit2 = LED_N_3;
    digit1 = LED_N_0;
    low_band_limit = 10100000;
    high_band_limit = 10150000;
    OPfreq = 10136000; 
}

void BAND20() {  
    digit2 = LED_N_2;
    digit1 = LED_N_0;
    low_band_limit = 14000000;
    high_band_limit = 14500000;
    OPfreq = 14074000;
}

void BAND17() {  
    digit2 = LED_N_1;
    digit1 = LED_N_7; 
    low_band_limit = 18068000;
    high_band_limit = 18500000;
    OPfreq = 18100000;
}

void BAND15() {  
    digit2 = LED_N_1;
    digit1 = LED_N_5; 
    low_band_limit = 21000000;
    high_band_limit = 21500000;
    OPfreq = 21074000;
}

void BAND12() {  
    digit2 = LED_N_1;
    digit1 = LED_N_2; 
    low_band_limit = 24890000;
    high_band_limit = 24990000;
    OPfreq = 24915000;
}

void BAND10() {  
    digit2 = LED_N_1;
    digit1 = LED_N_0; 
    low_band_limit = 28000000;
    high_band_limit = 28500000;
    OPfreq = 28074000;
}

#endif

#if MULTIDC_VFO

void get_band_MDC() {
  digit5 = LED_N_6;
  digit4 = LED_n; 
  digit3 = LED_BLANK;
  
switch(BANDpointer) 
      {
case 1: 
      MDC_BAND15();
      break;
      
case 2:
      MDC_BAND12();
      break;

case 3: 
      MDC_BAND10() ;
      break ;
      }     
}

void MDC_BAND15() {  
    digit2 = LED_N_1;
    digit1 = LED_N_5; 
    low_band_limit = 21000000;
    high_band_limit = 21500000;
    OPfreq = 21060000;
    RITtemp = OPfreq;
}

void MDC_BAND12() {  
    digit2 = LED_N_1;
    digit1 = LED_N_2; 
    low_band_limit = 24890000;
    high_band_limit = 24990000;
    OPfreq = 24906000;
    RITtemp = OPfreq;
}

void MDC_BAND10() {  
    digit2 = LED_N_1;
    digit1 = LED_N_0; 
    low_band_limit = 28000000;
    high_band_limit = 28500000;
    OPfreq = 28060000;
    RITtemp = OPfreq;
}

#endif



//this compact and stand alone si5351 frequency calculation routines 
//are Copyright 2017, Jerry Gaffke, KE7ER, under the GNU General Public License 3.0 


#if !ENABLE_VOICE_SSB_TX
void si5351bx_init() {                  // Call once at power-up, start PLLA
    uint8_t reg;  uint32_t msxp1;
    Wire.begin();  
    i2cWrite(149, 0);                   // SpreadSpectrum off
    i2cWrite(3, si5351bx_clken);        // Disable all CLK output drivers
    i2cWrite(183, SI5351BX_XTALPF<<6 | 0x12);  // Set 25mhz crystal load capacitance
    msxp1 = 128*SI5351BX_MSA - 512;     // and msxp2=0, msxp3=1, not fractional
    uint8_t  vals[8] = {0, 1, BB2(msxp1), BB1(msxp1), BB0(msxp1), 0, 0, 0};
    i2cWriten(26, vals, 8);             // Write to 8 PLLA msynth regs
    i2cWrite(177, 0x20);                // Reset PLLA  (0x80 resets PLLB)
    // for (reg=16; reg<=23; reg++) i2cWrite(reg, 0x80);    // Powerdown CLK's
    // i2cWrite(187, 0);                // No fannout of clkin, xtal, ms0, ms4
}

void si5351bx_setfreq(uint8_t clknum, uint32_t fout) {  // Set a CLK to fout Hz
    uint32_t  msa, msb, msc, msxp1, msxp2, msxp3p2top;
    if ((fout<500000) || (fout>109000000))  // If clock freq out of range
        si5351bx_clken |= 1<<clknum;        //  shut down the clock
    else {
        msa = si5351bx_vcoa / fout;     // Integer part of vco/fout
        msb = si5351bx_vcoa % fout;     // Fractional part of vco/fout 
        msc = fout;             // Divide by 2 till fits in reg
        while (msc & 0xfff00000) {msb=msb>>1; msc=msc>>1;}
        msxp1 =(128*msa + 128*msb/msc - 512) | (((uint32_t)si5351bx_rdiv)<<20);
        msxp2 = 128*msb - 128*msb/msc * msc;    // msxp3 == msc;        
        msxp3p2top = (((msc & 0x0F0000) <<4) | msxp2);      // 2 top nibbles
        uint8_t vals[8] = { BB1(msc), BB0(msc), BB2(msxp1), BB1(msxp1), 
            BB0(msxp1), BB2(msxp3p2top), BB1(msxp2), BB0(msxp2) };
        i2cWriten(42+(clknum*8), vals, 8);  // Write to 8 msynth regs
        i2cWrite(16+clknum, 0x0C|si5351bx_drive[clknum]);   // use local msynth
        
    }

#if MULTIDC_VFO    
    si5351bx_clken &= ~(1<<clknum);     // Clear bit to enable clock
     i2cWrite(3, si5351bx_clken);        // Enable/disable clock
#endif

#if FT8_VFO    
        if (clknum == 0) {si5351bx_clken = 0xfe;} //turn on the approperate clock output. 
       if (clknum == 1) {si5351bx_clken = 0xfd;} 
       i2cWrite(3, si5351bx_clken);        // Enable/disable clock
#endif
}

void i2cWrite(uint8_t reg, uint8_t val){    // write reg via i2c
    Wire.beginTransmission(SI5351BX_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt){   // write array
    Wire.beginTransmission(SI5351BX_ADDR);
    Wire.write(reg);
    while (vcnt--) Wire.write(*vals++);
    Wire.endTransmission();
}
#endif //!ENABLE_VOICE_SSB_TX

void reset_flash(void) {
  EEPROM.write(11, 0xff);
  EEPROM.write(10, 0xff);
  EEPROM.write(9, 0xff);
  EEPROM.write(8, 0xff);
    EEPROM.write(7, 0xff);
    EEPROM.write(6, 0xff);
    EEPROM.write(5, 0xff);
    EEPROM.write(4, 0xff);

    EEPROM.write(0, 0xff);

    void (*cold_reset)(void) = 0;
    cold_reset();
}



#if !ENABLE_VOICE_SSB_TX
/*
    Si5351A related functions
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
 */

#define SI5351A_CLK1_CONTROL            17
#define SI5351A_PLLB_BASE               34
#define SI5351A_MULTISYNTH1_BASE        50
#define SI5351A_PLL_RESET               177

#define SI5351A_CLK1_MS1_INT            (1 << 6)
#define SI5351A_CLK1_MS1_SRC_PLLB       (1 << 5)
#define SI5351A_CLK1_SRC_MULTISYNTH_1   (3 << 2)
#define SI5351A_CLK1_IDRV_2MA           (0 << 0)

#define SI5351A_PLL_RESET_PLLB_RST      (1 << 7)

#define PLL_CALCULATION_PRECISION       4

static uint8_t s_regs[8];

inline void si5351a_setup_PLLB(uint8_t mult, uint32_t num, uint32_t denom)
{
    uint32_t p1 = 128 * mult + ((128 * num) / denom) - 512;
    uint32_t p2 = 128 * num - denom * ((128 * num) / denom);
    uint32_t p3 = denom;

    s_regs[0] = (uint8_t)(p3 >> 8);
    s_regs[1] = (uint8_t)p3;
    s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    s_regs[3] = (uint8_t)(p1 >> 8);
    s_regs[4] = (uint8_t)p1;
    s_regs[5] = ((uint8_t)(p3 >> 12) & 0xf0) | ((uint8_t)(p2 >> 16) & 0x0f);
    s_regs[6] = (uint8_t)(p2 >> 8);
    s_regs[7] = (uint8_t)p2;
    i2cWriten(SI5351A_PLLB_BASE, s_regs, 8);
}

// div must be even number
inline void si5351a_setup_multisynth1(uint32_t div)
{
    uint32_t p1 = 128 * div - 512;

    s_regs[0] = 0;
    s_regs[1] = 1;
    s_regs[2] = (uint8_t)(p1 >> 16) & 0x03;
    s_regs[3] = (uint8_t)(p1 >> 8);
    s_regs[4] = (uint8_t)p1;
    s_regs[5] = 0;
    s_regs[6] = 0;
    s_regs[7] = 0;
    i2cWriten(SI5351A_MULTISYNTH1_BASE, s_regs, 8);

    i2cWrite(SI5351A_CLK1_CONTROL, (SI5351A_CLK1_MS1_INT | 
                                    SI5351A_CLK1_MS1_SRC_PLLB | 
                                    SI5351A_CLK1_SRC_MULTISYNTH_1 | 
                                    SI5351A_CLK1_IDRV_2MA));
}

inline void si5351a_reset_PLLB(void)
{
    i2cWrite(SI5351A_PLL_RESET, SI5351A_PLL_RESET_PLLB_RST);
}

// freq is in 28.4 fixed point number, 0.0625Hz resolution
void si5351a_set_freq(uint32_t freq)
{
    #define PLL_MAX_FREQ        900000000
    #define PLL_MIN_FREQ        600000000
    #define PLL_MID_FREQ        ((PLL_MAX_FREQ + PLL_MIN_FREQ) / 2)
    #define PLL_DENOM_MAX       0x000fffff

    uint32_t ms_div = PLL_MID_FREQ / (freq >> PLL_CALCULATION_PRECISION) + 1;
    ms_div &= 0xfffffffe;   // make it even number

    uint32_t pll_freq = ((uint64_t)freq * ms_div) >> PLL_CALCULATION_PRECISION;

    uint32_t calibrated_crystal_freq = SI5351_CALIBRATED_CRYSTAL_FREQ;
    uint32_t pll_mult   = pll_freq / calibrated_crystal_freq;
    uint32_t pll_remain = pll_freq - (pll_mult * calibrated_crystal_freq);
    uint32_t pll_num    = (uint64_t)pll_remain * PLL_DENOM_MAX / calibrated_crystal_freq;
    si5351a_setup_PLLB(pll_mult, pll_num, PLL_DENOM_MAX);

    static uint32_t prev_ms_div = 0;
    if (ms_div != prev_ms_div) {
        prev_ms_div = ms_div;
        si5351a_setup_multisynth1(ms_div);
        si5351a_reset_PLLB();
    }
}



/*
    Audio Wave Length Measurement related functions
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
 */

#define MIN_INPUT_AUDIO_FREQ            190                         // minimum input audio frequency limit is 200Hz  - 5%
#define MAX_INPUT_AUDIO_FREQ            4200                        // maximum input audio frequency limit is 4000Hz + 5%
#define UPDATE_VFO_PERIOD               (CPU_CLOCK_FREQ / 250)      // update FSK frequency 250 times/sec. (every 4ms)
#define NO_SIGNAL_PERIOD_THRESHOLD      (CPU_CLOCK_FREQ / 20)       // no signal detection threshold (50ms)
#define MIN_SAMPLE_COUNT_FOR_AVERAGING  2                           // minimum sample counts for averaging filter

volatile uint8_t  gMeasuredFullWaveCount = 0;
volatile uint16_t gTimer1OverflowCounter = 0;
volatile uint32_t gCurrentTimer1InputCaptureValue = 0;
volatile uint32_t gUpperHalfLenSum = 0;
volatile uint32_t gLowerHalfLenSum = 0;

inline void resetMeasuredValues(void)
{
    noInterrupts();

    // reset values
    gMeasuredFullWaveCount = 0;
    gUpperHalfLenSum       = 0;
    gLowerHalfLenSum       = 0;

    interrupts();
}

inline void readAndResetMeasuredValues(uint32_t *currentInputCaptureValue, uint8_t *fullWaveCount, uint32_t *upperHalfWaveLenSum, uint32_t *lowerHalfWaveLenSum)
{
    noInterrupts();

    *currentInputCaptureValue = gCurrentTimer1InputCaptureValue;

    *fullWaveCount         = gMeasuredFullWaveCount;
    *upperHalfWaveLenSum   = gUpperHalfLenSum;
    *lowerHalfWaveLenSum   = gLowerHalfLenSum;

    // reset values
    gMeasuredFullWaveCount = 0;
    gUpperHalfLenSum       = 0;
    gLowerHalfLenSum       = 0;

    interrupts();
}

inline uint32_t readCurrentTimer1Value(void)
{
    noInterrupts();
    uint16_t counterValue = TCNT1;
    uint32_t currentTimer1Value = ((uint32_t)gTimer1OverflowCounter << 16) | counterValue;
    if ((TIFR1 & (1 << TOV1)) && (counterValue & 0x8000) == 0x0000) {
        // timer1 overflow happened and hasn't handled it yet
        currentTimer1Value += 0x10000;
    }
    interrupts();
    return currentTimer1Value;
}

void processAudioInput(bool checkNoSignal)
{
    static bool sIsTransmitting = false;

    // read the length of the last measured audio wave
    uint32_t currentInputCaptureValue;
    uint8_t  inputCaptureEvents;
    uint32_t upperWaveLenSum;
    uint32_t lowerWaveLenSum;
    readAndResetMeasuredValues(&currentInputCaptureValue, &inputCaptureEvents, &upperWaveLenSum, &lowerWaveLenSum);

    static uint32_t sLastVFOUpdatedInputCaptureValue  = 0;
    static uint16_t sCapturedWaveCount                = 0;
    static uint32_t sUpperWaveLenTotal                = 0;
    static uint32_t sLowerWaveLenTotal                = 0;
    static uint32_t sLastValidSignalInputCaptureValue = 0;

    if (inputCaptureEvents > 0) {
        sCapturedWaveCount += inputCaptureEvents;
        sUpperWaveLenTotal += upperWaveLenSum;
        sLowerWaveLenTotal += lowerWaveLenSum;

        if (sLastVFOUpdatedInputCaptureValue == 0) {
            sLastVFOUpdatedInputCaptureValue = currentInputCaptureValue;
        }

        uint32_t totalWaveLength = currentInputCaptureValue - sLastVFOUpdatedInputCaptureValue;
        if (totalWaveLength >= UPDATE_VFO_PERIOD && sCapturedWaveCount >= MIN_SAMPLE_COUNT_FOR_AVERAGING) {

            // measured audio wave length
            uint32_t averageWaveLength = ((sUpperWaveLenTotal << PLL_CALCULATION_PRECISION) + (sCapturedWaveCount / 2)) / sCapturedWaveCount +
                                         ((sLowerWaveLenTotal << PLL_CALCULATION_PRECISION) + (sCapturedWaveCount / 2)) / sCapturedWaveCount;
            // measured audio frequency
            uint32_t audioFreq = (CPU_CLOCK_FREQ << (PLL_CALCULATION_PRECISION * 2)) / averageWaveLength;   // frequency is in 28.4 fixed point number, 0.0625Hz resolution

            if (((uint32_t)MIN_INPUT_AUDIO_FREQ << PLL_CALCULATION_PRECISION) <= audioFreq && audioFreq <= ((uint32_t)MAX_INPUT_AUDIO_FREQ << PLL_CALCULATION_PRECISION) &&
                sLowerWaveLenTotal < sUpperWaveLenTotal && sUpperWaveLenTotal < (sLowerWaveLenTotal << 1))  // sLowerWaveLenTotal < sUpperWaveLenTotal < sLowerWaveLenTotal * 2
            {
                // found audio signal
                sLastValidSignalInputCaptureValue = currentInputCaptureValue;

                if (sIsTransmitting) {
                    digitalWrite(A1, LOW);              // disable RX
                    si5351a_set_freq((OPfreq << PLL_CALCULATION_PRECISION) + audioFreq);    // update CLK1 frequency
                    if (si5351bx_clken != 0xfd) {
                        si5351bx_clken = 0xfd;
                        i2cWrite(3, si5351bx_clken);    // enable CLK1 output
                    }
                }

                sIsTransmitting = true;     // set this flag at here so we can ignore the first detected frequency which might include some error
            }

            sLastVFOUpdatedInputCaptureValue = currentInputCaptureValue;
            sCapturedWaveCount = 0;
            sUpperWaveLenTotal = 0;
            sLowerWaveLenTotal = 0;
        }
    }

    if (checkNoSignal && sIsTransmitting) {
        uint32_t currentTimer1Value = readCurrentTimer1Value();
        uint32_t noSignalPeriod = currentTimer1Value - sLastValidSignalInputCaptureValue;
        if (noSignalPeriod > NO_SIGNAL_PERIOD_THRESHOLD) {
            // detected no signal period
            sLastVFOUpdatedInputCaptureValue = 0;
            sCapturedWaveCount = 0;
            sUpperWaveLenTotal = 0;
            sLowerWaveLenTotal = 0;

            resetMeasuredValues();

            // disable CLK1 (TX) and enable CLK0 (RX), switch to RX mode
            si5351bx_clken = 0xfe;
            i2cWrite(3, si5351bx_clken);
            digitalWrite(A1, HIGH);         // enable RX

            sIsTransmitting = false;
        }
    }
}   

inline uint32_t readTimer1InputCaptureValue(void)
{
    uint16_t counterValue = ICR1;
    uint32_t currentTimer1Value = ((uint32_t)gTimer1OverflowCounter << 16) | counterValue;
    if ((TIFR1 & (1 << TOV1)) && (counterValue & 0x8000) == 0x0000) {
        // timer1 overflow happened and hasn't handled it yet
        currentTimer1Value += 0x10000;
    }
    return currentTimer1Value;
}

// ISR priority 11
ISR(TIMER1_CAPT_vect) {
    static uint32_t sPrevInputCaptureValue;
    static uint32_t sUpperWaveLen;

    uint32_t currentInputCaptureValue = readTimer1InputCaptureValue();
    uint32_t halfWaveLen = currentInputCaptureValue - sPrevInputCaptureValue;

    uint8_t currTCCR1B = TCCR1B;
    if (currTCCR1B & (1 << ICES1)) {
        // detected Falling Audio Signal Edge (Rising Input Capture Edge)
        static uint32_t sAveUpperHalfWaveLen = 0;
        sAveUpperHalfWaveLen = (sAveUpperHalfWaveLen + sAveUpperHalfWaveLen + sAveUpperHalfWaveLen + halfWaveLen) >> 2;  // (sAveUpperHalfWaveLen * 3 + halfWaveLen) / 4;
        if (halfWaveLen < ((sAveUpperHalfWaveLen >> 2) + (sAveUpperHalfWaveLen >> 4))) {    // (sAveUpperHalfWaveLen * 0.3125)
            // ignore ripple
            return;
        }

        sUpperWaveLen = halfWaveLen;
    }
    else {
        // detected Rising Audio Signal Edge (Falling Input Capture Edge)
        static uint32_t sAveLowerHalfWaveLen = 0;
        sAveLowerHalfWaveLen = (sAveLowerHalfWaveLen + sAveLowerHalfWaveLen + sAveLowerHalfWaveLen + halfWaveLen) >> 2;  // (sAveLowerHalfWaveLen * 3 + halfWaveLen) / 4;
        if (halfWaveLen < ((sAveLowerHalfWaveLen >> 2) + (sAveLowerHalfWaveLen >> 4))) {    // (sAveLowerHalfWaveLen * 0.3125)
            // ignore ripple
            return;
        }

        gUpperHalfLenSum += sUpperWaveLen;
        gLowerHalfLenSum += halfWaveLen;
        gCurrentTimer1InputCaptureValue = currentInputCaptureValue;
        gMeasuredFullWaveCount++;
    }
    sPrevInputCaptureValue = currentInputCaptureValue;

    TCCR1B = currTCCR1B ^ (1 << ICES1);     // flip edge selection
    TIFR1 = (1 << ICF1);                    // clear Input Capture interrupt Flag (flipping the edge selection causes the unwanted interrupt)
}

// ISR priority 14
ISR(TIMER1_OVF_vect) {
    gTimer1OverflowCounter++;
}
#endif //!ENABLE_VOICE_SSB_TX



#if ENABLE_VOICE_SSB_TX
/*
    SSB modulator code on ATmega328P
    Developed by Guido Ten Dolle PE1NNZ https://github.com/threeme3/QCX-SSB

    commit 4fc60f5c8d74ba7364cf891e008b920ab5e5c82d (HEAD -> feature-rx-improved, origin/feature-rx-improved)
    Author: guido <pe1nnz@amsat.org>
    Date:   Sun Aug 22 22:46:17 2021 +0200

    All the modifications to the origina code are wrapped by "#if AFSK_TO_FSK_VFO"
*/

#define F_XTAL        SI5351BX_XTAL
#define SI5351_ADDR   SI5351BX_ADDR
#define F_MCU 16000000  // 16MHz ATMEGA328P crystal
#define RS_HIGH_ON_IDLE   1   // Experimental LCD support where RS line is high on idle periods to comply with SDA I2C standard.

/* begin of Guido's code */

//line#40
#define VOX_ENABLE       1   // Voice-On-Xmit which is switching the transceiver into transmit as soon audio is detected (above noise gate level)

//line#177
#if(ARDUINO < 10810)
   #error "Unsupported Arduino IDE version, use Arduino IDE 1.8.10 or later from https://www.arduino.cc/en/software"
#endif
#if !(defined(ARDUINO_ARCH_AVR))
   #error "Unsupported architecture, select Arduino IDE > Tools > Board > Arduino AVR Boards > Arduino Uno."
#endif
#if(F_CPU != 16000000)
   #error "Unsupported clock frequency, Arduino IDE must specify 16MHz clock; alternate crystal frequencies may be specified with F_MCU."
#endif
#undef F_CPU
#if !AFSK_TO_FSK_VFO
#define F_CPU 20007000  // Actual crystal frequency of 20MHz XTAL1, note that this declaration is just informative and does not correct the timing in Arduino functions like delay(); hence a 1.25 factor needs to be added for correction.
#else //AFSK_TO_FSK_VFO
#define F_CPU   F_MCU
#endif //!AFSK_TO_FSK_VFO
#ifndef F_MCU
#define F_MCU 20000000  // 20MHz ATMEGA328P crystal
#endif

//line#261
volatile uint8_t vox = 0;

//line#1211
// I2C communication starts with a START condition, multiple single byte-transfers (MSB first) followed by an ACK/NACK and stops with a STOP condition;
// during data-transfer SDA may only change when SCL is LOW, during a START/STOP condition SCL is HIGH and SDA goes DOWN for a START and UP for a STOP.
// https://www.ti.com/lit/an/slva704/slva704.pdf
class I2C {
public:
#if(F_MCU > 20900000)
  #define I2C_DELAY   6
#else
#if !AFSK_TO_FSK_VFO
  #define I2C_DELAY   4    // Determines I2C Speed (2=939kb/s (too fast!!); 3=822kb/s; 4=731kb/s; 5=658kb/s; 6=598kb/s). Increase this value when you get I2C tx errors (E05); decrease this value when you get a CPU overload (E01). An increment eats ~3.5% CPU load; minimum value is 3 on my QCX, resulting in 84.5% CPU load
#else //AFSK_TO_FSK_VFO
  #define I2C_DELAY         6
  #define I2C_DELAY_RISE    7
#endif //!AFSK_TO_FSK_VFO
#endif
  #define I2C_DDR DDRC     // Pins for the I2C bit banging
  #define I2C_PIN PINC
  #define I2C_PORT PORTC
  #define I2C_SDA (1 << 4) // PC4
  #define I2C_SCL (1 << 5) // PC5
  #define DELAY(n) for(uint8_t i = 0; i != n; i++) asm("nop");
  #define I2C_SDA_GET() I2C_PIN & I2C_SDA
  #define I2C_SCL_GET() I2C_PIN & I2C_SCL
  #define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
  #define I2C_SDA_LO() I2C_DDR |=  I2C_SDA;
#if !AFSK_TO_FSK_VFO
  #define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
  #define I2C_SCL_LO() I2C_DDR |=  I2C_SCL; DELAY(I2C_DELAY);
#else //AFSK_TO_FSK_VFO
  #define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY_RISE);
  #define I2C_SCL_LO() I2C_DDR |=  I2C_SCL;
#endif //!AFSK_TO_FSK_VFO

  I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_SCL_HI();
    I2C_SDA_HI();
#ifndef RS_HIGH_ON_IDLE
    suspend();
#endif
  }
  ~I2C(){
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_DDR &= ~( I2C_SDA | I2C_SCL );
  }  
  inline void start(){
#ifdef RS_HIGH_ON_IDLE
    I2C_SDA_LO();
#else
    resume();  //prepare for I2C
#endif
    I2C_SCL_LO();
    I2C_SDA_HI();
  }
  inline void stop(){
    I2C_SDA_LO();   // ensure SDA is LO so STOP-condition can be initiated by pulling SCL HI (in case of ACK it SDA was already LO, but for a delayed ACK or NACK it is not!)
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_DDR &= ~(I2C_SDA | I2C_SCL); // prepare for a start: pull-up both SDA, SCL
#ifndef RS_HIGH_ON_IDLE
    suspend();
#endif
  }
  #define SendBit(data, mask) \
    if(data & mask){ \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();
  /*#define SendByte(data) \
    SendBit(data, 1 << 7) \
    SendBit(data, 1 << 6) \
    SendBit(data, 1 << 5) \
    SendBit(data, 1 << 4) \
    SendBit(data, 1 << 3) \
    SendBit(data, 1 << 2) \
    SendBit(data, 1 << 1) \
    SendBit(data, 1 << 0) \
    I2C_SDA_HI();  // recv ACK \
    DELAY(I2C_DELAY);     \
    I2C_SCL_HI();         \
    I2C_SCL_LO();*/
  inline void SendByte(uint8_t data){
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI();  // recv ACK
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SCL_LO();
  }
  inline uint8_t RecvBit(uint8_t mask){
    I2C_SCL_HI();
    uint16_t i = 60000;
    for(;!(I2C_SCL_GET()) && i; i--);  // wait util slave release SCL to HIGH (meaning data valid), or timeout at 3ms
    //if(!i){ lcd.setCursor(0, 1); lcd.print(F("E07 I2C timeout")); }
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return (data) ? mask : 0;
  }
  inline uint8_t RecvByte(uint8_t last){
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
    if(last){
      I2C_SDA_HI();  // NACK
    } else {
      I2C_SDA_LO();  // ACK
    }
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SDA_HI();    // restore SDA for read
    I2C_SCL_LO();
    return data;
  }
  inline void resume(){
#ifdef LCD_RS_PORTIO
    I2C_PORT &= ~I2C_SDA; // pin sharing SDA/LCD_RS mitigation
#endif
  }
  inline void suspend(){
    I2C_SDA_LO();         // pin sharing SDA/LCD_RS: pull-down LCD_RS; QCXLiquidCrystal require this for any operation
  }

  void begin(){};
  void beginTransmission(uint8_t addr){ start(); SendByte(addr << 1);  };
  bool write(uint8_t byte){ SendByte(byte); return 1; };
  uint8_t endTransmission(){ stop(); return 0; };
};

//#define log2(n) (log(n) / log(2))
uint8_t log2(uint16_t x){
  uint8_t y = 0;
  for(; x>>=1;) y++;
  return y;
}

// /*
I2C i2c;
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  //volatile uint32_t _mod;
  volatile uint8_t pll_regs[8];

  #define BB0(x) ((uint8_t)(x))           // Bash byte x of int32_t
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define FAST __attribute__((optimize("Ofast")))

  volatile uint32_t fxtal = F_XTAL;

#define NEW_TX 1
#ifdef NEW_TX
  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  {
    #define _MSC  0x10000
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;

    uint16_t msp1 = _msa128min512 + msb128 / _MSC; // = 128 * _msa + msb128 / _MSC - 512;
    uint16_t msp2 = msb128; // = msb128 % _MSC;  assuming MSC is covering exact uint16_t so the mod operation can dissapear (and the upper BB2 byte) // = msb128 - msb128/_MSC * _MSC;

    //pll_regs[0] = BB1(msc);  // 3 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    //pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))/*|BB2(msp2)*/; // top nibble MUST be same as top nibble of _MSC !  assuming that BB2(msp2) is always 0 -> so reg is constant
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }

  inline void SendPLLRegisterBulk(){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26+0*8 + 4);  // Write to PLLA
    //i2c.SendByte(26+1*8 + 4);  // Write to PLLB
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }
#else  // !NEW_TX
  inline void FAST freq_calc_fast(int16_t df)  // note: relies on cached variables: _msb128, _msa128min512, _div, _fout, fxtal
  { 
    #define _MSC  0x80000  //0x80000: 98% CPU load   0xFFFFF: 114% CPU load
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;
    //uint32_t msb128 = ((int64_t)(_div * (int32_t)df + _mod) * _MSC * 128) / fxtal; // @pre: 14<=_div<=144, |df|<=5000, _mod<=1800e3 (for fout<30M), _MSC=524288

    //#define _MSC  (F_XTAL/128)   // MSC exact multiple of F_XTAL (and maximized to fit in max. span 1048575)
    //uint32_t msb128 = (_div * (int32_t)df + _mod);

    //#define _MSC  0xFFFFF  // Old algorithm 114% CPU load, shortcut for a fixed fxtal=27e6
    //register uint32_t xmsb = (_div * (_fout + (int32_t)df)) % fxtal;  // xmsb = msb * fxtal/(128 * _MSC);
    //uint32_t msb128 = xmsb * 5*(32/32) - (xmsb/32);  // msb128 = xmsb * 159/32, where 159/32 = 128 * 0xFFFFF / fxtal; fxtal=27e6

    //#define _MSC  (F_XTAL/128)  // 114% CPU load  perfect alignment
    //uint32_t msb128 = (_div * (_fout + (int32_t)df)) % fxtal;

    uint32_t msp1 = _msa128min512 + msb128 / _MSC;  // = 128 * _msa + msb128 / _MSC - 512;
    uint32_t msp2 = msb128 % _MSC;  // = msb128 - msb128/_MSC * _MSC;
    //uint32_t msp1 = _msa128min512;  // = 128 * _msa + msb128 / _MSC - 512;  assuming msb128 < _MSC, so that msp1 is constant
    //uint32_t msp2 = msb128;  // = msb128 - msb128/_MSC * _MSC, assuming msb128 < _MSC

    //pll_regs[0] = BB1(msc);  // 3 regs are constant
    //pll_regs[1] = BB0(msc);
    //pll_regs[2] = BB2(msp1);
    pll_regs[3] = BB1(msp1);
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4))|BB2(msp2); // top nibble MUST be same as top nibble of _MSC !
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }

  inline void SendPLLRegisterBulk(){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26+0*8 + 3);  // Write to PLLA
    //i2c.SendByte(26+1*8 + 3);  // Write to PLLB
    i2c.SendByte(pll_regs[3]);
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }
#endif // !NEW_TX
  
  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n){
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    while (n--) i2c.SendByte(*data++);
    i2c.stop();      
  }
  void SendRegister(uint8_t reg, uint8_t val){ SendRegister(reg, &val, 1); }
  int16_t iqmsa; // to detect a need for a PLL reset
///*
  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };
  
  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA, uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0){
    uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
    msa = div_nom / div_denom;     // integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS
    if(msa == 4) _int = 1;  // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3 which basically says that for MS divider a value of 4 and integer mode must be used
    msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom); // fractional part
    msc = (_int) ? 1 : _MSC;
    //lcd.setCursor(0, 0); lcd.print(n); lcd.print(":"); lcd.print(msa); lcd.print(" "); lcd.print(msb); lcd.print(" "); lcd.print(msc); lcd.print(F("    ")); delay(500);
    msp1 = 128*msa + 128*msb/msc - 512;
    msp2 = 128*msb - 128*msb/msc * msc;
    msp3 = msc;
    uint8_t ms_reg2 = BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C);
    uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };

    SendRegister(n*8+42, ms_regs, 8); // Write to MSx
    if(n < 0){
      SendRegister(n+16+8, 0x80|(0x40*_int)); // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    } else {
      //SendRegister(n+16, ((pll)*0x20)|0x0C|0|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 0=2mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+16, ((pll)*0x20)|0x0C|3|(0x40*_int));  // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+165, (!_int) * phase * msa / 90);      // when using: make sure to configure MS in fractional-mode, perform reset afterwards
    }
  }

  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase){ SendRegister(n+165, phase * (div_nom / div_denom) / 90); }  // when using: make sure to configure MS in fractional-mode!, perform reset afterwards

  void reset(){ SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB

  void oe(uint8_t mask){ SendRegister(3, ~mask); } // output-enable mask: CLK2=4; CLK1=2; CLK0=1

  void freq(int32_t fout, uint16_t i, uint16_t q){  // Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
      uint8_t rdiv = 0; // CLK pin sees fout/(2^rdiv)
      if(fout > 300000000){ i/=3; q/=3; fout/=3; }  // for higher freqs, use 3rd harmonic
      if(fout < 500000){ rdiv = 7; fout *= 128; } // Divide by 128 for fout 4..500kHz
      uint16_t d; if(fout < 30000000) d = (16 * fxtal) / fout; else d = (32 * fxtal) / fout;  // Integer part  .. maybe 44?
      if(fout < 3500000) d = (7 * fxtal) / fout;  // PLL at 189MHz to cover 160m (freq>1.48MHz) when using 27MHz crystal
      if(fout > 140000000) d = 4; // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d += 2; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      // si5351 spectral purity considerations: https://groups.io/g/QRPLabs/message/42662

      ms(MSNA, fvcoa, fxtal);                   // PLLA in fractional mode
      //ms(MSNB, fvcoa, fxtal);
#if !AFSK_TO_FSK_VFO
      ms(MS0,  fvcoa, fout, PLLA, 0, i, rdiv);  // Multisynth stage with integer divider but in frac mode due to phase setting
      ms(MS1,  fvcoa, fout, PLLA, 0, q, rdiv);
#ifdef F_CLK2
      freqb(F_CLK2);
#else
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
#endif
#else //AFSK_TO_FSK_VFO
      ms(MS0,  fvcoa, fout, PLLA, 0, 0, rdiv);  // RX LO
      ms(MS1,  fvcoa, fout, PLLA, 0, 0, rdiv);  // TX carrier
#endif //!AFSK_TO_FSK_VFO
      if(iqmsa != (((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90)){ iqmsa = ((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90; reset(); }
#if !AFSK_TO_FSK_VFO
      oe(0b00000011);  // output enable CLK0, CLK1
#endif //!AFSK_TO_FSK_VFO

#ifdef x
      ms(MSNA, fvcoa, fxtal);
      ms(MSNB, fvcoa, fxtal);
      #define F_DEV 4
      ms(MS0,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS1,  fvcoa, (fout + F_DEV), PLLA, 0, 0, rdiv);
      ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
      reset();
      ms(MS0,  fvcoa, fout, PLLA, 0, 0, rdiv);
      delayMicroseconds(F_MCU/16000000 * 1000000UL/F_DEV);  // Td = 1/(4 * Fdev) phase-shift   https://tj-lab.org/2020/08/27/si5351%e5%8d%98%e4%bd%93%e3%81%a73mhz%e4%bb%a5%e4%b8%8b%e3%81%ae%e7%9b%b4%e4%ba%a4%e4%bf%a1%e5%8f%b7%e3%82%92%e5%87%ba%e5%8a%9b%e3%81%99%e3%82%8b/
      ms(MS1,  fvcoa, fout, PLLA, 0, 0, rdiv);
      oe(0b00000011);  // output enable CLK0, CLK1
#endif
      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
      //_mod = fvcoa % fxtal;
  }

  void freqb(uint32_t fout){  // Set a CLK2 to fout Hz (on PLLB)
      uint16_t d = (16 * fxtal) / fout;
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz

      ms(MSNB, fvcoa, fxtal);
      ms(MS2,  fvcoa, fout, PLLB, 0, 0, 0);
  }
  
//*/
/*
  void freq(uint32_t fout, uint16_t i, uint16_t q){  // Set a CLK0,1 to fout Hz with phase i, q
      uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
      uint8_t rdiv = 0;             // CLK pin sees fout/(2^rdiv)
      if(fout > 300000000){ i/=3; q/=3; fout/=3; }  // for higher freqs, use 3rd harmonic
      if(fout < 500000){ rdiv = 7; fout *= 128; } // Divide by 128 for fout 4..500kHz

      uint16_t d = (16 * fxtal) / fout;  // Integer part
      //if(fout > 7000000) d = (33 * fxtal) / fout;
      if(fout < 3500000) d = (7 * fxtal) / fout;  // PLL at 189MHz to cover 160m (freq>1.48MHz) when using 27MHz crystal

      if( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d++; // Test if multiplier remains same for freq deviation +/- 5kHz, if not use different divider to make same
      if(d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
      bool divby4 = 0; if(fout > 140000000){ d = 4; divby4 = 1; } // for f=140..300MHz; AN619; 4.1.3
      uint32_t fvcoa = d * fout;  // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
      msa = fvcoa / fxtal;     // Integer part of vco/fxtal. msa must be in range 15..90
      msb = ((uint64_t)(fvcoa % fxtal)*_MSC) / fxtal; // fractional part
      msc = _MSC;
      
      msp1 = 128*msa + 128*msb/msc - 512;
      msp2 = 128*msb - 128*msb/msc * msc;
      msp3 = msc;
      uint8_t pll_regs[8] = { BB1(msp3), BB0(msp3), BB2(msp1), BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };
      SendRegister(26+0*8, pll_regs, 8); // Write to PLLA
      SendRegister(26+1*8, pll_regs, 8); // Write to PLLB
      SendRegister(16+6, 0x80); // PLLA in fractional mode; 0x40=FBA_INT; 0x80=CLK6_PDN
      SendRegister(16+7, 0x80); // PLLB in fractional mode; 0x40=FBB_INT; 0x80=CLK7_PDN

      msa = fvcoa / fout;     // Integer part of vco/fout. msa must be in range 6..127 (support for integer and initial phase offset)
      //lcd.setCursor(0, 0); lcd.print(fvcoa/fxtal); lcd.print(" "); lcd.print(msb); lcd.print(" "); lcd.print(msa); lcd.print(F("     "));
      msp1 = (divby4) ? 0 : (128*msa - 512);     // msp1 and msp2=0, msp3=1, integer division
      msp2 = 0;
      msp3 = 1;
      uint8_t ms_regs[8] = { BB1(msp3), BB0(msp3), BB2(msp1) | (rdiv<<4) | (divby4*0x0C), BB1(msp1), BB0(msp1), BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };
      SendRegister(42+0*8, ms_regs, 8); // Write to MS0
      SendRegister(42+1*8, ms_regs, 8); // Write to MS1
      SendRegister(42+2*8, ms_regs, 8); // Write to MS2
      SendRegister(16+0, 0x0C|3|(0x40*divby4));  // CLK0: 0x0C=PLLA local msynth; 3=8mA; 0x40=MS0_INT; 0x80=CLK0_PDN
      SendRegister(16+1, 0x0C|3|(0x40*divby4));  // CLK1: 0x0C=PLLA local msynth; 3=8mA; 0x40=MS1_INT; 0x80=CLK1_PDN
      SendRegister(16+2, 0x2C|3|(0x40*divby4));  // CLK2: 0x2C=PLLB local msynth; 3=8mA; 0x40=MS2_INT; 0x80=CLK2_PDN
      SendRegister(165, i * msa / 90);  // CLK0: I-phase (on change -> Reset PLL)
      SendRegister(166, q * msa / 90);  // CLK1: Q-phase (on change -> Reset PLL)
      if(iqmsa != ((i-q)*msa/90)){ iqmsa = (i-q)*msa/90; SendRegister(177, 0xA0); } // 0x20 reset PLLA; 0x80 reset PLLB
      SendRegister(3, 0b11111100);      // Enable/disable clock

      _fout = fout;  // cache
      _div = d;
      _msa128min512 = fvcoa / fxtal * 128 - 512;
      _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
  }
*/
  uint8_t RecvRegister(uint8_t reg){
    i2c.start();  // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(true);
    i2c.stop();
    return data;
  }
  void powerDown(){
    SendRegister(3, 0b11111111); // Disable all CLK outputs
    SendRegister(24, 0b00000000); // Disable state: LOW state when disabled
    SendRegister(25, 0b00000000); // Disable state: LOW state when disabled
    for(int addr = 16; addr != 24; addr++) SendRegister(addr, 0b10000000);  // Conserve power when output is disabled
    SendRegister(187, 0);        // Disable fanout (power-safe)
    // To initialise things as they should:
    SendRegister(149, 0);        // Disable spread spectrum enable
    SendRegister(183, 0b11010010);  // Internal CL = 10 pF (default)
  }
  #define SI_CLK_OE 3

};
static SI5351 si5351;
// */

//line#1951
enum mode_t { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

inline void _vox(bool trigger)
{
  if(trigger){
    tx = (tx) ? 254 : 255; // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on when not triggered again). tx == 255 when triggered first, 254 follows for subsequent triggers, until tx is off.
  } else {
    if(tx) tx--;
  }
}

#if !AFSK_TO_FSK_VFO
#define F_SAMP_TX 4800 //4810 //4805 // 4402 // (Design) ADC sample-rate; is best a multiple of _UA and fits exactly in OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1 , should not exceed CPU utilization
#if(F_MCU != 20000000)
const int16_t _F_SAMP_TX = (F_MCU * 4800LL / 20000000);  // Actual ADC sample-rate; used for phase calculations
#else
#define _F_SAMP_TX  F_SAMP_TX
#endif
#define _UA  600 //=(_FSAMP_TX)/8 //(_F_SAMP_TX)      //360  // unit angle; integer representation of one full circle turn or 2pi radials or 360 degrees, should be a integer divider of F_SAMP_TX and maximized to have higest precision
#else //AFSK_TO_FSK_VFO
#define F_SAMP_TX   4000
#define _F_SAMP_TX  F_SAMP_TX
#define _UA         (_F_SAMP_TX / 8)
#endif //!AFSK_TO_FSK_VFO
#define MAX_DP  ((filt == 0) ? _UA : (filt == 3) ? _UA/4 : _UA/2)     //(_UA/2) // the occupied SSB bandwidth can be further reduced by restricting the maximum phase change (set MAX_DP to _UA/2).
#define CARRIER_COMPLETELY_OFF_ON_LOW  1    // disable oscillator on low amplitudes, to prevent potential unwanted biasing/leakage through PA circuit
#define MULTI_ADC  1  // multiple ADC conversions for more sensitive (+12dB) microphone input
//#define QUAD  1       // invert TX signal for phase changes > 180

inline int16_t arctan3(int16_t q, int16_t i)  // error ~ 0.8 degree
{ // source: [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
//#define _atan2(z)  (_UA/8 + _UA/44) * z  // very much of a simplification...not accurate at all, but fast
#define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z  //derived from (5) [1]   note that atan2 can overflow easily so keep _UA low
//#define _atan2(z)  (_UA/8 + _UA/24 - _UA/24 * z) * z  //derived from (7) [1]
  int16_t r;
  if(abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

uint8_t lut[256];
volatile uint8_t amp;
#define MORE_MIC_GAIN   1       // adds more microphone gain, improving overall SSB quality (when speaking further away from microphone)
#ifdef MORE_MIC_GAIN
volatile uint8_t vox_thresh = (1 << 2);
#else
volatile uint8_t vox_thresh = (1 << 1); //(1 << 2);
#endif
volatile uint8_t drive = 2;   // hmm.. drive>2 impacts cpu load..why?

volatile uint8_t quad = 0;
#if AFSK_TO_FSK_VFO
static int16_t v[16];
#endif //AFSK_TO_FSK_VFO

inline int16_t ssb(int16_t in)
{
  static int16_t dc, z1;

  int16_t i, q;
  uint8_t j;
#if !AFSK_TO_FSK_VFO
  static int16_t v[16];
#endif //!AFSK_TO_FSK_VFO
  for(j = 0; j != 15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
//#define DIG_MODE  // optimization for digital modes: for super flat TX spectrum, (only down < 100Hz to cut-off DC components)
#ifdef DIG_MODE
  int16_t ac = in;
  dc = (ac + (7) * dc) / (7 + 1);  // hpf: slow average
  v[15] = (ac - dc) / 2;           // hpf (dc decoupling)  (-6dB gain to compensate for DC-noise)
#else
  int16_t ac = in * 2;             //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif //DIG_MODE
  i = v[7] * 2;  // 6dB gain for i, q  (to prevent quanitization issues in hilbert transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]); // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
  //dc += (in - dc) / 2;       // fast moving average
  dc = (in + dc) / 2;        // average
  int16_t ac = (in - dc);   // DC decoupling
  //v[15] = ac;// - z1;        // high-pass (emphasis) filter
  v[15] = (ac + z1);// / 2;           // low-pass filter with notch at Fs/2
  z1 = ac;

  i = v[7];
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2; // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i, q);
#endif  // MORE_MIC_GAIN

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
  _vox(_amp > vox_thresh);
#else
  if(vox) _vox(_amp > vox_thresh);
#endif
  //_amp = (_amp > vox_thresh) ? _amp : 0;   // vox_thresh = 4 is a good setting
  //if(!(_amp > vox_thresh)) return 0;

  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp; // clip or when drive=8 use max output
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference and restriction
  //dp = (amp) ? dp : 0;  // dp = 0 when amp = 0
  prev_phase = phase;

  if(dp < 0) dp = dp + _UA; // make negative phase shifts positive: prevents negative frequencies and will reduce spurs on other sideband
#ifdef QUAD
  if(dp >= (_UA/2)){ dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if(dp > MAX_DP){ // dp should be less than half unit-angle in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if(mode == USB)
    return dp * ( _F_SAMP_TX / _UA); // calculate frequency-difference based on phase-difference
  else
    return dp * (-_F_SAMP_TX / _UA);
}

#define MIC_ATTEN  0  // 0*6dB attenuation (note that the LSB bits are quite noisy)
volatile int8_t mox = 0;
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB phase-differences, calculation of SI5351 frequency registers and send the registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx()
{ // jitter dependent things first
#ifdef MULTI_ADC  // SSB with multiple ADC conversions:
  int16_t adc;                         // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
#ifdef QUAD
#ifdef TX_CLK0_CLK1
  si5351.SendRegister(16, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK0 in case of a huge phase-change
  si5351.SendRegister(17, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK1 in case of a huge phase-change
#else
  si5351.SendRegister(18, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK2 in case of a huge phase-change
#endif
#endif //QUAD
  OCR1BL = amp;                      // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += ADC;
ADCSRA |= (1 << ADSC);  // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  int16_t df = ssb(_adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  //_adc = (adc/4 - 512);
#define AF_BIAS   32
  _adc = (adc/4 - (512 - AF_BIAS));        // now make sure that we keep a postive bias offset (to prevent the phase swapping 180 degrees and potentially causing negative feedback (RFI)
#else  // SSB with single ADC conversion:
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
#endif

#ifdef CARRIER_COMPLETELY_OFF_ON_LOW
#if AFSK_TO_FSK_VFO
  if(tx == 1){ si5351.oe(0); }   // disable carrier
  if(tx == 255){ si5351.oe(1 << 1); } // enable carrier
#else //!AFSK_TO_FSK_VFO
  if(tx == 1){ OCR1BL = 0; si5351.SendRegister(SI_CLK_OE, TX0RX0); }   // disable carrier
  if(tx == 255){ si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier
#endif //AFSK_TO_FSK_VFO
#endif

#ifdef MOX_ENABLE
  if(!mox) return;
  OCR1AL = (adc << (mox-1)) + 128;  // TX audio monitoring
#endif
}

//line#2155
void dummy()
{
}

//line#2190
void dsp_tx_fm()
{ // jitter dependent things first
#if !AFSK_TO_FSK_VFO
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  OCR1BL = lut[255];                   // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
#else //AFSK_TO_FSK_VFO

// do same SSB processing for ADC bias and VOX to work properly
#ifdef MULTI_ADC  // SSB with multiple ADC conversions:
  int16_t adc;                         // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
#ifdef QUAD
#ifdef TX_CLK0_CLK1
  si5351.SendRegister(16, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK0 in case of a huge phase-change
  si5351.SendRegister(17, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK1 in case of a huge phase-change
#else
  si5351.SendRegister(18, (quad) ? 0x1f : 0x0f);  // Invert/non-invert CLK2 in case of a huge phase-change
#endif
#endif //QUAD
  OCR1BL = amp;                      // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  adc += ADC;
ADCSRA |= (1 << ADSC);  // causes RFI on QCX-SSB units (not on units with direct biasing); ENABLE this line when using direct biasing!!
  ssb(_adc >> MIC_ATTEN); // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(v[15] << drive);           // calculate SI5351 registers based on frequency shift and carrier frequency
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  //_adc = (adc/4 - 512);
#define AF_BIAS   32
  _adc = (adc/4 - (512 - AF_BIAS));        // now make sure that we keep a postive bias offset (to prevent the phase swapping 180 degrees and potentially causing negative feedback (RFI)
#else  // SSB with single ADC conversion:
  ADCSRA |= (1 << ADSC);    // start next ADC conversion (trigger ADC interrupt if ADIE flag is set)
  //OCR1BL = amp;                        // submit amplitude to PWM register (actually this is done in advance (about 140us) of phase-change, so that phase-delays in key-shaping circuit filter can settle)
  si5351.SendPLLRegisterBulk();       // submit frequency registers to SI5351 over 731kbit/s I2C (transfer takes 64/731 = 88us, then PLL-loopfilter probably needs 50us to stabalize)
  OCR1BL = amp;                        // submit amplitude to PWM register (takes about 1/32125 = 31us+/-31us to propagate) -> amplitude-phase-alignment error is about 30-50us
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t df = ssb(adc >> MIC_ATTEN);  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  si5351.freq_calc_fast(df);           // calculate SI5351 registers based on frequency shift and carrier frequency
#endif

#endif //!AFSK_TO_FSK_VFO
}

//line#2909
typedef void (*func_t)(void);
volatile func_t func_ptr;

//line#3653
uint16_t analogSampleMic()
{
  uint16_t adc;
  noInterrupts();
  ADCSRA = (1 << ADEN) | (((uint8_t)log2((uint8_t)(F_CPU / 13 / (192307/1)))) & 0x07);  // hack: faster conversion rate necessary for VOX

#if !AFSK_TO_FSK_VFO
  if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, LOW);  // disable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, TX0RX0);
  uint8_t oldmux = ADMUX;
  for(;!(ADCSRA & (1 << ADIF)););  // wait until (a potential previous) ADC conversion is completed
  ADMUX = admux[2];  // set MUX for next conversion
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  for(;!(ADCSRA & (1 << ADIF)););  // wait until ADC conversion is completed
  ADMUX = oldmux;
  if((dsp_cap == SDR) && (vox_thresh >= 32)) digitalWrite(RX, HIGH);  // enable RF input, only for SDR mod and with low VOX threshold
  //si5351.SendRegister(SI_CLK_OE, TX0RX1);
#else //AFSK_TO_FSK_VFO
  for(;(ADCSRA & (1 << ADSC)););  // wait until (a potential previous) ADC conversion is completed
  ADCSRA |= (1 << ADSC);    // start next ADC conversion
  for(;(ADCSRA & (1 << ADSC)););  // wait until ADC conversion is completed
#endif //!AFSK_TO_FSK_VFO
  adc = ADC;
  interrupts();
  return adc;
}

//line#4210
static uint8_t vox_tx = 0;
static uint8_t vox_sample = 0;
static uint16_t vox_adc = 0;

/* end of Guido's code */
#endif //ENABLE_VOICE_SSB_TX



#if ENABLE_VOICE_SSB_TX
/*
    Glue code for SSB modulator
    Developed by Kazuhisa "Kazu" Terasaki AG6NS
*/

// ISR priority 12
ISR(TIMER1_COMPA_vect) {
    func_ptr();
    gTick4000++;
}

void initSSBTX(void) {
    func_ptr = dummy;

    #define F_ADC_CONV (192307/2)  //was 192307/1, but as noted this produces clicks in audio stream. Slower ADC clock cures this (but is a problem for VOX when sampling mic-input simulatanously).

    // initialize ADC
    DIDR0 = (1 << ADC2D);   // disable Digital Input Buffer on ADC2 pin
    ADCSRA = 0;
    ADCSRB = 0;
    ADMUX = 0x02;           // MUX=0b0010 (ADC2)
    ADMUX |= (1 << REFS0);  // AREF is AVcc (5V)
    uint32_t fs = F_ADC_CONV;
    ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;  // ADC Prescaler (for normal conversions non-auto-triggered): ADPS = log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp. conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
    ADCSRA |= (1 << ADEN);  // enable ADC

    // initialize TIMER1
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCNT1 = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11); // CTC mode, CS=0b011 (16MHz / 8)
    fs = F_SAMP_TX;
    uint16_t ocr = ((F_CPU / 8) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
    OCR1A = ocr;
    TIFR1 = 0x27;               // clear all interrupt flags
    TIMSK1 = (1 << OCIE1A);     // enable Output Compare A Match interrupt
    interrupts();

    setCalibratedXtalFrequency(SI5351_CALIBRATED_CRYSTAL_FREQ);

    vox = 1;
    drive = 6;  // only for FM, no affect to SSB
}

inline void si5351bx_init() {
}

inline void si5351bx_setfreq(uint8_t clknum, uint32_t fout) {
    si5351.freq(fout, 0, 0);
    si5351.oe(1 << clknum);
}

inline void setCalibratedXtalFrequency(uint32_t calibrated_xtal_freq) {
    si5351.fxtal = calibrated_xtal_freq;
}

inline void changeModeToTX(void) {
    TIMSK2 &= ~(1 << OCIE2A);       // disable TIMER2 Compare Match A Interrupt
    digitalWrite(A1, LOW);          // disable RX
    delay(10);
    si5351bx_setfreq(1, OPfreq);    // enable CLK1 (TX) and disable CLK0 (RX)
    switch (mode) {
    case LSB:
    case USB:
        func_ptr = dsp_tx;
        break;
    case FM:
        func_ptr = dsp_tx_fm;
        break;
    }
}

inline void changeModeToRX(void) {
    func_ptr = dummy;
    si5351bx_setfreq(0, OPfreq);    // disable CLK1 (TX) and enable CLK0 (RX)
    delay(10);
    digitalWrite(A1, HIGH);         // enable RX
    TIMSK2 |= (1 << OCIE2A);        // enable TIMER2 Compare Match A Interrupt
}

inline bool isTransmitting(void) {
    return (func_ptr != dummy);
}

void switch_rxtx(uint8_t tx_enable) {
  noInterrupts();
  tx = tx_enable;
  if(tx_enable){ // tx
    changeModeToTX();
  } else {  // rx
    changeModeToRX();
  }
  interrupts();
}

void processAudioInput(bool every1ms) {
#ifdef VOX_ENABLE
  if((vox) && ((mode == LSB) || (mode == USB) || (mode == FM))){  // If VOX enabled (and in LSB/USB mode), then take mic samples and feed ssb processing function, to derive amplitude, and potentially detect cross vox_threshold to detect a TX or RX event: this is expressed in tx variable
    if(!vox_tx){ // VOX not active
#ifdef MULTI_ADC
      if(vox_sample++ == 16){  // take N sample, then process
        ssb(((int16_t)(vox_adc/16) - (512 - AF_BIAS)) >> MIC_ATTEN);   // sampling mic
        vox_sample = 0;
        vox_adc = 0;
      } else {
        vox_adc += analogSampleMic();
      }
#else
      ssb(((int16_t)(analogSampleMic()) - 512) >> MIC_ATTEN);   // sampling mic
#endif
      static uint8_t sInitialVOXDelay = 255;
      if (sInitialVOXDelay > 0) {
        if (every1ms) sInitialVOXDelay--;
        tx = 0;
      }

      if(tx){  // TX triggered by audio -> TX
        vox_tx = 1;
        switch_rxtx(255);
        //for(;(tx);) wdt_reset();  // while in tx (workaround for RFI feedback related issue)
        //delay(100); tx = 255;
      }
    } else if(!tx){  // VOX activated, no audio detected -> RX
      switch_rxtx(0);
      vox_tx = 0;
      delay(32); //delay(10);
      //vox_adc = 0; for(i = 0; i != 32; i++) ssb(0); //clean buffers
      //for(int i = 0; i != 32; i++) ssb((analogSampleMic() - 512) >> MIC_ATTEN); // clear internal buffer
      //tx = 0; // make sure tx is off (could have been triggered by rubbish in above statement)
    }
  }
#endif //VOX_ENABLE
}

void refreshDisplay(void) {
    if (++digit_counter > 4) {
        digit_counter = 0;
    }

    switch(digit_counter) {
    case 0: 
        digitalWrite(SLED5, HIGH);
        PORTD = digit1;
        digitalWrite(SLED1, LOW);
        break;

    case 1: 
        digitalWrite(SLED1, HIGH);
        PORTD = digit2;
        digitalWrite(SLED2, LOW);
        break;

    case 2:  
        digitalWrite(SLED2, HIGH);
        PORTD = digit3;
        digitalWrite(SLED3, LOW);
        break;

    case 3: 
        digitalWrite(SLED3, HIGH);
        PORTD = digit4;
        digitalWrite(SLED4, LOW);
        break;

    case 4: 
        digitalWrite(SLED4, HIGH);
        PORTD = digit5;
        digitalWrite(SLED5, LOW);
        break;
    }
}

void changeTXMode(uint8_t newMode) {
    if (newMode == mode) return;
    mode = newMode;
}

void displayCurrentTXMode(void) {
    switch (mode) {
    case LSB:
        digit5 = LED_L;
        digit4 = LED_N_5;
        digit3 = LED_b;
        digit2 = LED_BLANK;
        digit1 = LED_BLANK;
        break;
    case USB:
        digit5 = LED_U;
        digit4 = LED_N_5;
        digit3 = LED_b;
        digit2 = LED_BLANK;
        digit1 = LED_BLANK;
        break;
    case FM:
        digit5 = LED_F;
        digit4 = LED_r;
        digit3 = LED_n;
        digit2 = LED_BLANK;
        digit1 = LED_BLANK;
        break;
    }
}

void changeTXMode(void) {
    do {
        if (bitRead(sw_inputs, U_sw) == LOW) {
            switch (mode) {
            case LSB:
                mode = USB;
                break;
            case USB:
                mode = FM;
                break;
            case FM:
                mode = LSB;
                break;
            }
            delay(150);
        }
        if (bitRead(sw_inputs, D_sw) == LOW) {
            switch (mode) {
            case FM:
                mode = USB;
                break;
            case USB:
                mode = LSB;
                break;
            case LSB:
                mode = FM;
                break;
            }
            delay(150);
        }
        displayCurrentTXMode();
    }
    while (bitRead(sw_inputs,E_sw) == HIGH);
    displayfreq();
    debounceE(); 
}
#endif //ENABLE_VOICE_SSB_TX
