
#include <EEPROM.h>
#include "Wire.h"


#define USE_ROTARY_ENCODER            0
#define USE_A3_AS_TX_SIGNAL           0

#define TEST_TIMER1_INPUT_CAPTURE     0


#define CPU_CLOCK_FREQ                16000000                    // 16MHz
#define MIN_INPUT_CAPTURE_VALUE       (CPU_CLOCK_FREQ / 4000)     // maximum capture frequency is 4000Hz
#define MAX_INPUT_CAPTURE_VALUE       (CPU_CLOCK_FREQ / 150)      // minimum capture frequency is 150Hz
#define UPDATE_VFO_PERIOD_IN_CLK      (CPU_CLOCK_FREQ / 100)      // update VFO frequency 100 time/sec.


// si5451 definations 
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351BX_XTALPF 3             // 1:6pf  2:8pf  3:10pf

#define SI5351BX_XTAL 25004000          // Crystal freq in Hz
#define SI5351BX_MSA  35                // VCOA is at 25mhz*35 = 875mhz

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
byte        ritflag = 0;
byte        EncoderFlag = 0;
byte        c = 0;
byte        d = 0;
byte        cLast; 

volatile unsigned long tcount;

int                Eadr;
volatile byte      sw_inputs ; 


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

long int    Fstep10  = 10 ;                // 10 Hz step
long int    Fstep100 = 100;
long int    Fstep1K =  1000;
long int    Fstep5K =  5000;
long int    Fstep10K = 10000;
long int    Fstep100K = 100000;
                                          
//encoder 


unsigned  long  temp1;
unsigned  long frequency;
unsigned  long freq_result;  
unsigned  long OPfreq;
unsigned  long  RITtemp; 
unsigned long   temp ; 
unsigned long   cal_value;
 //********************************************** 

// registers for limit testing

unsigned long  low_band_limit;  //low limit, band tuning
unsigned long  high_band_limit; //high limit, band tuning 
unsigned long  low_absolute_limit = 500000;
unsigned long  high_absolute_limit = 30000000;


volatile uint8_t gTimer1CaptureEvents = 0;
volatile uint16_t gTimer1OverflowCounter = 0;
volatile uint32_t gCurrentTimer1CaptureValue = 0;
uint32_t gLastTimer1CaptureValue = 0;
uint16_t gTimer1CaptureCount = 0;


void setup() {
    //switch inputs

    DDRB = 0xff;
    DDRD = 0Xff; 
#if USE_A3_AS_TX_SIGNAL
    pinMode(A3, INPUT_PULLUP);
#endif
#if USE_ROTARY_ENCODER
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
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

    si5351bx_init(); 
    cal_data();    //load calibration data  
    stepK = Fstep100 ; //default tuning rate
    stepSize = 2; 
    delay(1000); 
    int_band();
    delay(1000); //let things settle down a bit

    displayfreq();
    PLLwrite();

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

    // disable CLK1
    si5351bx_setfreq(1, 499999);

#if TEST_TIMER1_INPUT_CAPTURE
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A3, OUTPUT);
#endif
}


void loop() {
    static uint16_t sPrevTimer1 = 0;
    bool processUI = false;
    if ((TCNT1 - sPrevTimer1) >= (CPU_CLOCK_FREQ / 1000)) {
        sPrevTimer1 = TCNT1;
        processUI = true;
    }
    else {
        goto SKIP_UI;
    }

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

#if USE_A3_AS_TX_SIGNAL
    if (A3==LOW) {
        transmit();
    }
#endif

#if USE_ROTARY_ENCODER
    if (EncoderFlag == 1) {     // test tune up flag
        Tune_UP();
    }
    if (EncoderFlag == 2) {     // test tune down flag 
        Tune_DWN();
    }
#endif

SKIP_UI:

#if TEST_TIMER1_INPUT_CAPTURE
    static uint8_t sCnt1;
    digitalWrite(A1, ((++sCnt1 & 0x01) ? HIGH : LOW));
#endif

    noInterrupts();
    uint32_t currentTimer1CaptureValue = gCurrentTimer1CaptureValue;
    uint8_t timer1CaptureEvents = gTimer1CaptureEvents;
    gTimer1CaptureEvents = 0;
    interrupts();
    static bool sIsTransmitting = false;
    if (timer1CaptureEvents > 0) {
#if TEST_TIMER1_INPUT_CAPTURE
        static uint8_t sCnt3;
        digitalWrite(A3, ((++sCnt3 & 0x01) ? HIGH : LOW));
#endif
        gTimer1CaptureCount += timer1CaptureEvents;
        uint32_t totalWaveLength = currentTimer1CaptureValue - gLastTimer1CaptureValue;
        if (totalWaveLength >= UPDATE_VFO_PERIOD_IN_CLK) {
            uint32_t averageWaveLength = (totalWaveLength + (gTimer1CaptureCount >> 1)) / gTimer1CaptureCount;
            gLastTimer1CaptureValue = currentTimer1CaptureValue;
            gTimer1CaptureCount = 0;
            if (MIN_INPUT_CAPTURE_VALUE <= averageWaveLength && averageWaveLength <= MAX_INPUT_CAPTURE_VALUE) {
                // found audio signal
                if (sIsTransmitting) {
                    uint32_t audioFreq = CPU_CLOCK_FREQ / averageWaveLength;
                    si5351bx_setfreq(1, OPfreq + audioFreq);    // update CLK1 frequency
                }
                sIsTransmitting = true;
            }
        }
    }
    else if (processUI && sIsTransmitting) {
        noInterrupts();
        uint16_t capturedValue = TCNT1;
        uint32_t currentTimer1Value = ((uint32_t)gTimer1OverflowCounter << 16) | capturedValue;
        if ((TIFR1 & (1 << TOV1)) && (capturedValue & 0x8000) == 0x0000) {
            // timer1 overflow happened and hasn't handled it yet
            currentTimer1Value += 0x10000;
        }
        interrupts();
        uint32_t noSignalPeriod = currentTimer1Value - gLastTimer1CaptureValue;
        if (noSignalPeriod > (UPDATE_VFO_PERIOD_IN_CLK * 10)) {
            gLastTimer1CaptureValue = currentTimer1Value;
            gTimer1CaptureCount = 0;
            gTimer1CaptureEvents = 0;

            // disable CLK1
            si5351bx_setfreq(1, 499999);
            sIsTransmitting = false;
        }
    }
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

#if USE_A3_AS_TX_SIGNAL
void transmit() {
        si5351bx_setfreq(0, RITtemp);
        //si5351bx_setfreq(1, RITtemp); 
        while (A3 == LOW){loop;} 
      si5351bx_setfreq(0, OPfreq); //update clock chip with VFO frequency.
      //si5351bx_setfreq(1, OPfreq); //update clock chip with VFO 
}
#endif


void timedswitch() {  
    unsigned long time0;
    unsigned long duration;

    time0 = tcount;

    do {
        duration = (unsigned long)tcount - time0;   // calculate how long the button has been pushed
        if (duration == 1000) {
            digit5 = LED_N_6;
            digit4 = LED_n;
            digit3 = LED_BLANK;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
        if (duration == 5000) {
            digit5 = LED_C;
            digit4 = LED_A;
            digit3 = LED_L;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
        if (duration == 200) {
            digit5 = LED_r;
            digit4 = LED_BLANK;
            digit3 = LED_BLANK;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        } 
        if (duration == 10000) {
            digit5 = LED_r;
            digit4 = LED_E;
            digit3 = LED_N_5;
            digit2 = LED_BLANK;
            digit1 = LED_BLANK;
        }
    } while (bitRead(sw_inputs, E_sw) != 1);

    if (duration >= 10000) {
        reset_flash();
    }
    else if (duration >= 5000) {
        calibration();
    }
    else if (duration >= 1000) {
        changeBand();
    }
    else if (duration >= 200)  {
        RIT();
    }
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

    if (ritflag & RIT_ON) {
        RITdisplay();
    }
    else {
        displayfreq();
    }
    
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

    if (ritflag & RIT_ON) {
        RITdisplay();
    }
    else {
        displayfreq();
    }

    PLLwrite(); 
}

//toggle tuning step rate
void  nextFstep () {
    if (ritflag == 1) {
        flip_sideband();
    }
    else {
        incStep();
    }
}

void incStep() { 
    if (outofband == 1) {
        bigsteps();
    }
    else {
        allsteps();
    }
}


void allsteps() {
    if (++stepSize > 6) {
        stepSize = 1;
    }

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
        digit5 = LED_BLANK;  
        delay(100); 
        digit5 = d1temp;
        break;
    }
}

void bigsteps() {
    if (++stepSize > 6) {
        stepSize = 4;
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


void flip_sideband() {
    if (OPfreq > RITtemp) {
        OPfreq = RITtemp - 600;
        digit4 = LED_neg;
    }
    else {
        OPfreq = RITtemp + 600;
        digit4 = LED_BLANK;
    }
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
    if (outofband == 0) {
        OPfreq = RITtemp +600;
    } 
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


////////////////////////////////////////////////
//This breaks up the frequency value into decades and then converts the result 
//hex value into the LED 7 seg map. 
///////////////////////////////////////////////

void    displayfreq()
{

   if (outofband == 1) {OFB_F_display(); return;}
   else {FREQdisplay();}
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

    if (++digit_counter > 5) {
        digit_counter = 0;
    }

    switch(digit_counter) {
    case 0:
        digitalWrite(SLED5, HIGH); 
        readswitches(); 
        break;

    case 1: 
        PORTD = digit1;
        digitalWrite(SLED1, LOW);
        break;

    case 2: 
        digitalWrite(SLED1, HIGH);
        PORTD = digit2;
        digitalWrite(SLED2, LOW);
        break;

    case 3:  
        digitalWrite(SLED2, HIGH); 
        PORTD = digit3;
        digitalWrite(SLED3, LOW);
        break;

    case 4: 
        digitalWrite(SLED3, HIGH); 
        PORTD = digit4;
        digitalWrite(SLED4, LOW);
        break;

    case 5: 
        digitalWrite(SLED4, HIGH);
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
    
#if USE_ROTARY_ENCODER
    c = digitalRead(ENC_A);  // read encoder clock bit
    if (c != cLast) {   // call if changed
        encoder();
    }
#endif
}
                                 
// encoder, test for direction only on 0 to 1 clock state change

#if USE_ROTARY_ENCODER
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
    si5351bx_setfreq(0, OPfreq); //update clock chip with VFO frequency.
    //si5351bx_setfreq(1, OPfreq); //update clock chip with VFO 
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
            debounceU();
        }
        if (bitRead(sw_inputs, D_sw) == LOW) {
            ADJ_DWN();
            debounceD();
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
      si5351bx_vcoa = si5351bx_vcoa -175;
      calwrite();
}

void ADJ_DWN() {
      si5351bx_vcoa = si5351bx_vcoa +175;
      calwrite();
}

void calwrite() {
      si5351bx_setfreq(0, OPfreq); 

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



void changeBand(){
      digit5 = LED_N_6;
      digit4 = LED_n; 
      digit3 = LED_BLANK; 
      get_band();
    
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
    if (BANDpointer == 11) {(BANDpointer = 1);}
    get_band();
    debounceU(); 
}

void nextbandD() {
   --BANDpointer ;
    if (BANDpointer == 0) {(BANDpointer = 10);}
    get_band();
    debounceD(); 
}

void int_band() {
  BANDpointer = EEPROM.read(0);
  if (BANDpointer == 0xff) {BANDpointer= 1; changeBand();}
  digit5 = LED_N_6;
  digit4 = LED_n; 
  digit3 = LED_BLANK;
  get_band(); 
}


void get_band() {
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
    OPfreq = 535700;    
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
    OPfreq = 10130000; 
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
    OPfreq = 21060000;
}

void BAND12() {  
    digit2 = LED_N_1;
    digit1 = LED_N_2; 
    low_band_limit = 24890000;
    high_band_limit = 24990000;
    OPfreq = 24900000;
}

void BAND10() {  
    digit2 = LED_N_1;
    digit1 = LED_N_0; 
    low_band_limit = 28000000;
    high_band_limit = 28500000;
    OPfreq = 28060000;
}

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
        si5351bx_clken &= ~(1<<clknum);     // Clear bit to enable clock
    }
 //       if (clknum == 0) {si5351bx_clken = 0xfe;} //turn on the approperate clock output. 
 //       if (clknum == 1) {si5351bx_clken = 0xfd;} 
        i2cWrite(3, si5351bx_clken);        // Enable/disable clock
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
    EEPROM.write(7, 0xff);
    EEPROM.write(6, 0xff);
    EEPROM.write(5, 0xff);
    EEPROM.write(4, 0xff);

    EEPROM.write(0, 0xff);

    void (*cold_reset)(void) = 0;
    cold_reset();
}

ISR(TIMER1_CAPT_vect) {
#if TEST_TIMER1_INPUT_CAPTURE
    static uint8_t sCnt0;
    digitalWrite(A0, ((++sCnt0 & 0x01) ? HIGH : LOW));
#endif
    uint16_t capturedValue = ICR1;
    gCurrentTimer1CaptureValue = ((uint32_t)gTimer1OverflowCounter << 16) | capturedValue;
    if ((TIFR1 & (1 << TOV1)) && (capturedValue & 0x8000) == 0x0000) {
        // timer1 overflow happened and hasn't handled it yet
        gCurrentTimer1CaptureValue += 0x10000;
    }
    gTimer1CaptureEvents++;
}

ISR(TIMER1_OVF_vect) {
    gTimer1OverflowCounter++;
}
