
/*Sketch for QRPguys.com  Basic Si5351 Digital VFO    
 * Firmware offerned as open source GNU rules apply. 
 * Written by KD1JV for QRPguys copywrite 2020 
 * Si5153 routine by Jerry Gaffke, KE7ER,
 * set board type to UNO when compiling. 
 * 
 * 
 * 
 */



#include <EEPROM.h>
#include "Wire.h"


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

#define  SLED5  9
#define  SLED4  10
#define  SLED3  11
#define  SLED2  12
#define  SLED1  13


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



//flag definitions 


#define     E_sw    2
#define     U_sw    0
#define     D_sw    1

// register names 
byte        SRtemp;
byte        Etemp; 
byte        BANDpointer; 
byte        outofband =0; 

volatile unsigned long tcount;
unsigned long        duration=0;

int                Eadr;
volatile byte      sw_inputs ; 

//temp storage of 7 segment display data 
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
long int    stepK;      // temp storage of freq tuning step

long int    Fstep10  = 10 ;                // 10 Hz step
long int    Fstep100 = 100;
long int    Fstep1K =  1000;
long int    Fstep5K =  5000;
long int    Fstep100K = 100000;
                                        
unsigned long  temp1;
unsigned long  frequency;
unsigned long  freq_result;  
unsigned long  OPfreq;
volatile unsigned long   time1;
volatile unsigned long   time0;
unsigned long   temp ; 
unsigned long   cal_value;
 //********************************************** 

// registers for limit testing

unsigned long  low_band_limit;  //low limit, band tuning
unsigned long  high_band_limit; //high limit, band tuning 
unsigned long  low_absolute_limit = 500000;
unsigned long  high_absolute_limit = 30000000;

ISR (TIMER1_COMPA_vect) {TIMER1_SERVICE_ROUTINE();}

void setup() {
 //switch inputs

DDRB = 0xff;
DDRD = 0Xff; 
   
      noInterrupts();
      TCCR1A = 0;
      TCCR1B = 0;
      TCNT1 = 0;
      
      OCR1A = 238;
      TCCR1B = 0x0a; 
      TIMSK1 |= (1 << OCIE1A);
      interrupts();   

      
      si5351bx_init(); //initialize the Si5351 chip 
      cal_data();    //load calibration data  
      stepK = Fstep1K ; //default tuning rate
      stepSize = 3; 
      delay(1000); 
      int_band();  //get the intial band
      delay(1000); //let things settle down a bit
 
      displayfreq();
      PLLwrite();
      time0 = tcount; //clear the counter
}


void loop() {
  // test for switch closed
       
        if (bitRead(sw_inputs, E_sw) == LOW) {timedswitch(); debounceE();}
           
        if (bitRead(sw_inputs, U_sw) == LOW) {Tune_UP();} //test tune up flag
        if (bitRead(sw_inputs, D_sw) == LOW) {Tune_DWN();} //test tune down flag   
}   

//**************************************************************
//end of switch polling loop
//**************************************************************

void debounceE(){
    while (bitRead(sw_inputs, E_sw) == LOW) {delay(1);} 
    }

void debounceU(){
    while (bitRead(sw_inputs, U_sw) == LOW) {delay(1);} 
}

void debounceD(){
    while (bitRead(sw_inputs, D_sw) == LOW) {delay(1);} 
}

void timedswitch(){  
    duration = 0;
    time0 = tcount;
  do {time1 = tcount; duration = time1- time0; //calculate how long the button has been pushed
   if (duration == 10000) {digit5 = LED_N_6; digit4 = LED_n; digit3 = 0x00; digit2 = 0x00; digit1 = 0x00;}
   if (duration == 50000) {digit5 = LED_C; digit4 = LED_A; digit3 = LED_L; digit2= 0x00; digit1 = 0x00;}
   
  }
  while (bitRead(sw_inputs,E_sw) !=1);
    duration = time1 - time0;
  
    if (duration > 50000) {calibration();duration = 0;}
    if (duration > 10000) {changeBand(); duration = 0;}
    if (duration > 500)  {nextFstep(); duration = 0;}
}


void  Tune_UP() {
      FREQ_incerment();
      delay(250);
}

void Tune_DWN() {    
      FREQ_decerment();
      delay(250);  
}


// adjust the operating frequency
void FREQ_incerment() {
      if (OPfreq >= high_absolute_limit) {return;} 
      OPfreq  = OPfreq + stepK;  //add frequenc tuning step to frequency word
      outofband = 0;
      if (OPfreq > high_band_limit) {outofband =1;} //band tuning limits
      if (OPfreq < low_band_limit) {outofband = 1;}
      displayfreq();
      PLLwrite();    
}

void FREQ_decerment() {
      if (OPfreq <= low_absolute_limit) {return;} 
      OPfreq  = OPfreq - stepK;
      outofband = 0;
      if (OPfreq < low_band_limit) {outofband = 1;}
      if (OPfreq > high_band_limit) {outofband =1;} 
      displayfreq();
      PLLwrite(); 
}

//toggle tuning step rate
void  nextFstep () {
    ++ stepSize ;
    if (stepSize == 5) {(stepSize = 1);}
    
switch(stepSize) {
     case 1:
          stepK = Fstep10;
          d1temp = digit1;
          digit1 = 0x00;    
          delay(100); 
          digit1 = d1temp;
          break;
           
     case 2:
          stepK = Fstep100;
          d1temp = digit2;
          digit2 = 0x00;
          delay(100); 
          digit2 = d1temp;
          break; 

    case 3:
          stepK = Fstep1K;
          d1temp = digit3;
          digit3 = 0x00;  
          delay(100); 
          digit3 = d1temp;
          break; 

       case 4:
          stepK = Fstep5K;
          d1temp = digit4;
          digit4 = 0x00;  
          delay(100); 
          digit4 = d1temp;
          break;    

 case 5:
          stepK = Fstep100K;
          d1temp = digit5;
          digit5 = 0x00;  
          delay(100); 
          digit5 = d1temp;
          break;              
                }
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
          if (freq_result == 0){digitX= 0x00;}
          digit5 = digitX;

          freq_result = frequency%10000000;
          freq_result = freq_result/1000000;
          hex2seg();
          digitX = digitX + 0x20;
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
        if (freq_result == 0){digitX = 0x00;}
        digit5 = digitX;  
        
        freq_result = frequency%100000;      //repeat the process for 10K, 1K and 100 Hz digits
        freq_result = freq_result/10000;
        hex2seg();
        digit4 = digitX;
        
        freq_result = frequency%10000;
        freq_result = freq_result/1000;
        hex2seg(); 
        digitX = digitX+ 0x20;
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

void TIMER1_SERVICE_ROUTINE() 
{  
     byte cREG;
     cREG = SREG;
          ++tcount;
        
        ++ digit_counter;
        if (digit_counter == 7 ) {(digit_counter = 1);}
       
switch(digit_counter) {
  
            case 1: 
            digitalWrite(8, HIGH); 
            DDRD = 0xff;
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
              
         
          case 6:                         //read the switches and encoder here
             digitalWrite(SLED5, HIGH);   
              DDRD = 0x00;
              PORTD= 0Xff; 
            digitalWrite(8, LOW);   
       for (int i = 0; i < 5; i++) {sw_inputs = PIND;} //read the switches 4 times to debounce 
            digitalWrite(8, HIGH); 
            
         break;  
        }
        SREG= cREG; 
}


/*
 * output the frequency data to the clock chip.
 */

void PLLwrite() {   
      si5351bx_setfreq(0,OPfreq); //update clock chip with VFO frequency. 
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
 
while (bitRead (sw_inputs,E_sw)== HIGH)
{
  if   (bitRead(sw_inputs, U_sw) == LOW){ADJ_UP();debounceU();}
  if   (bitRead(sw_inputs, D_sw) == LOW) {ADJ_DWN(); debounceD();} 
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
   
} //end of calibration routine



void ADJ_UP(){
      si5351bx_vcoa = si5351bx_vcoa -175;
      calwrite();
}

void ADJ_DWN() {
      si5351bx_vcoa = si5351bx_vcoa +175;
      calwrite();
}

void calwrite() {
      si5351bx_setfreq(0,OPfreq); 

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
      digit3 = 0x00; 
      get_band();
    
    debounceE(); 
      
    do {
      if (bitRead(sw_inputs,U_sw) !=1) {nextband();}
    }
    while (bitRead(sw_inputs,E_sw) !=0);

    EEPROM.write(0,BANDpointer); 
    
    debounceE(); 
    displayfreq();
    PLLwrite();  
}


void nextband () {
    ++BANDpointer ;
    if (BANDpointer == 8) {(BANDpointer = 1);}
    get_band();
    debounceU(); 
}


void int_band() {
  BANDpointer = EEPROM.read(0);
  if (BANDpointer == 0xff) {BANDpointer= 1; changeBand();}
  digit5 = LED_N_6;
  digit4 = LED_n; 
  digit3 = 0x00;
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
}

}

void BAND16() {  
    digit2 = LED_N_1;
    digit1 = LED_N_6;
    low_band_limit = 1800000;
    high_band_limit = 1820000;
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
  //      si5351bx_clken &= ~(1<<clknum);     // Clear bit to enable clock
    }
        if (clknum == 0) {si5351bx_clken = 0xfe;} //turn on the approperate clock output. 
        if (clknum == 1) {si5351bx_clken = 0xfd;} 
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
