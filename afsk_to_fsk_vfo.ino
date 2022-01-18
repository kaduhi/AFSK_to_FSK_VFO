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

#include <EEPROM.h>
#include "Wire.h"
//*********************************************************************
//USE this configuration for MULTI-DC VFO
//#define MULTIDC_VFO                     1
//#define FT8_VFO                         0
//*********************************************************************
// Use this configuration for FT-8 VFO
#define MULTIDC_VFO                     0
#define FT8_VFO                         1
//*********************************************************************

#define ENABLE_CAT_INTERFACE            1


#if ENABLE_CAT_INTERFACE

#include "src/IC746CAT/IC746.h"

IC746 radio = IC746();

bool gCATPttOn = false;
bool gCATSplitOn = false;
unsigned long gCATOtherVfoFreq = 0L;
bool gCATSsbModeLsb = false;
bool gCATVfoBActive = false;

#endif //ENABLE_CAT_INTERFACE


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
#define   LED_N_6  0xcb
#define   LED_N_7  0x16
#define   LED_N_8  0xdf
#define   LED_N_9  0x1f
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
#define   LED_BLANK 0x00
#define   LED_question 0xbd
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
    digit2 = LED_N_1;
    digit1 = LED_N_3; 
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
#endif 
  

    delay(1000); //let things settle down a bit
    displayfreq();
    PLLwrite();

#if ENABLE_CAT_INTERFACE
    init_IC746CAT_library();
#endif //ENABLE_CAT_INTERFACE
}    
void loop() {

#if FT8_VFO  
    static uint16_t sPrevTimer1 = 0;
    bool processUI = false;
    if ((TCNT1 - sPrevTimer1) >= (CPU_CLOCK_FREQ / 1000)) {
        sPrevTimer1 = TCNT1;
        processUI = true;
    }
    else {
        goto SKIP_UI;
    }
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

#if ENABLE_CAT_INTERFACE
    radio.check();
#endif //ENABLE_CAT_INTERFACE

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
#if ENABLE_CAT_INTERFACE
    sw_inputs = PIND | 0x03;    // only reads S1 MENU switch
#else //!ENABLE_CAT_INTERFACE
    sw_inputs = PIND;
#endif //!ENABLE_CAT_INTERFACE
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
#if ENABLE_CAT_INTERFACE
    if (gCATOtherVfoFreq == 0L) {
        gCATOtherVfoFreq = OPfreq;
    }
#endif //ENABLE_CAT_INTERFACE
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

#if ENABLE_CAT_INTERFACE
                if (!gCATPttOn) {
                    // disable FSK output even still detecting AFSK signal
                    // disable CLK1 (TX) and enable CLK0 (RX), switch to RX mode
                    si5351bx_clken = 0xfe;
                    i2cWrite(3, si5351bx_clken);
                    digitalWrite(A1, HIGH);         // enable RX
                }
                else
                    // do not enable FSK output even CAT PTT is ON unless detecting AFSK signal
#endif //ENABLE_CAT_INTERFACE
                if (sIsTransmitting) {
                    digitalWrite(A1, LOW);              // disable RX
#if ENABLE_CAT_INTERFACE
                    unsigned long freq = OPfreq;
                    if (gCATSplitOn) {
                        freq = gCATOtherVfoFreq;
                    }
                    freq <<= PLL_CALCULATION_PRECISION;
                    if (gCATSsbModeLsb) {
                        // LSB
                        freq -= audioFreq;
                    }
                    else {
                        // USB
                        freq += audioFreq;
                    }
                    si5351a_set_freq(freq);    // update CLK1 frequency
#else //!ENABLE_CAT_INTERFACE
                    si5351a_set_freq((OPfreq << PLL_CALCULATION_PRECISION) + audioFreq);    // update CLK1 frequency
#endif //!ENABLE_CAT_INTERFACE
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



#if ENABLE_CAT_INTERFACE
/*
    CAT interface related glue code
    Developed by Kazuhisa "Kazu" Terasaki AG6NS

    Using the "IC746CAT" library in https://github.com/kk4das/IC746CAT developed by Dean Souleles, KK4DAS
 */

void init_IC746CAT_library(void)
{
    radio.addCATPtt(catSetPtt);
    radio.addCATGetPtt(catGetPtt);
    radio.addCATAtoB(catVfoAtoB);
    radio.addCATSwapVfo(catSwapVfo);
    radio.addCATsplit(catSetSplit);
    radio.addCATFSet(catSetFreq);
    radio.addCATMSet(catSetMode);
    radio.addCATVSet(catSetVFO);
    radio.addCATGetFreq(catGetFreq);
    radio.addCATGetMode(catGetMode);

    radio.begin(19200, SERIAL_8N1);
}

void catSetPtt(boolean catPTT)
{
    gCATPttOn = (bool)catPTT;
}

boolean catGetPtt(void)
{
    return (boolean)gCATPttOn;
}

void catSetSplit(boolean catSplit)
{
    gCATSplitOn = (bool)catSplit;
}

void catSwapVfo()
{
    unsigned long temp = OPfreq;
    OPfreq = gCATOtherVfoFreq;
    gCATOtherVfoFreq = temp;

    gCATVfoBActive = !gCATVfoBActive;

    PLLwrite();
    displayfreq();
}

void catSetFreq(long f)
{
    OPfreq = (unsigned long)f;

    PLLwrite();
    displayfreq();
}

void catSetMode(byte m)
{
    gCATSsbModeLsb = (m == CAT_MODE_LSB);
}

void catSetVFO(byte v)
{
    if (v == CAT_VFO_A) {
        // VFO A
        if (gCATVfoBActive) {
            catSwapVfo();   // this will flip the gCATVfoBActive
        }
    }
    else {
        // VFO B
        if (!gCATVfoBActive) {
            catSwapVfo();   // this will flip the gCATVfoBActive
        }
    }
}

void catVfoAtoB()
{
    gCATOtherVfoFreq = OPfreq;
}

long catGetFreq() {
    return (long)OPfreq;
}

// function to pass the mode to the cat library
byte catGetMode() {
    return gCATSsbModeLsb ? CAT_MODE_LSB : CAT_MODE_USB;
}

#endif //ENABLE_CAT_INTERFACE
