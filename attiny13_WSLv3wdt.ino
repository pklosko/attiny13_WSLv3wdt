/**
 * ATTiny13 - Heltec WSLv3 WatchDog
 * 
 *  - battery voltage - ADC2
 *  - WLS "alive" scan 
 *  - UART debug and info
 *  
 * Arduino IDE settings:
 *   ATtiny13
 *   1.2MHz internal
 *   LTO disabled
 *   BOD none
 *   
 * (c) 2025 Petr KLOSKO
 * https://www.klosko.net/

 *      
 * Versions:
 * 
 * v.20250210
 *  - first release
 *  - LoRaThermo based log via UART
 *  - E32-900 as a Comm module - ADC pin used as M01 switch
 *  - Compiled /w MocroCore v.2.4.1
 * 
 * v.20250212
 *  - Remove delay before E32 sleep mode
 * 
 **/ 
#ifndef F_CPU
# define        F_CPU           (1200000UL) // 0.6 MHz
#endif  /* !F_CPU */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "uart.h"  // use £ukasz Marcin Podkalicki <lpodkalicki@gmail.com> Software UART for ATtiny13


#define INTERVAL       (16U)       // 16times miss watchdog = approx. 5 min
#define BMS_INTERVAL   (96U)       // 96times miss IRQ counter = approx. 30 min 96
#define BATT_INTERVAL  (2U)        // 96times miss IRQ counter = approx. 1 h

#define ADC_REF        (1<<REFS0) // internal reference 1.1 V 
#define ADC_CH2_BAT    2          // ADC channel - ADC2/PB4

#define ADC_CTRL       PB1
#define WSL_RST        PB0
#define WSL_USR        PB2
#define E32_M01        PB4

#define UTHR_BAT_CH    (4200U)     // 3.10V - Battery fuse is se to 3V
#define UTHR_BAT_OK    (3750U)     // 3.75V - Battery is in OK state
#define UTHR_BAT_LO    (3150U)     // 3.10V - Battery fuse is se to 3V
#define COEFFICIENT    (57U)       // coeficient to calculate Battery Voltage from ADC value, see and use CoeficientCalculator.xlsx 
#define BAT43VDROP     (50U)       // V drop of input BAT43 diode

volatile uint8_t c  __attribute__((section(".noinit")));   // Watchdog counter
volatile uint8_t r  __attribute__((section(".noinit")));   // IRQ counter, WSL Reading batt voltage
volatile uint8_t b  __attribute__((section(".noinit")));   // Battery Info Msg Counter
volatile uint8_t sf  __attribute__((section(".noinit")));  // Status Flaf

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

ISR(WDT_vect){
  c++;
}

ISR(INT0_vect){
  c=0;
  sf=0;
}

void WSLReset(){
  PORTB ^= (1 << WSL_RST);
  _delay_ms(1000);
  PORTB &= ~(1 << WSL_RST);
  sf = 0;
}

void WSLSleep(){
  PORTB ^= (1 << WSL_USR);
  _delay_ms(5000);
  PORTB &= ~(1 << WSL_USR);
}

void E32_TXmode(){
  ADCSRA &= ~(1<<ADEN);    //Disable ADC
  ACSR = (1<<ACD);         //Disable the analog comparator
  DDRB |= (1 << E32_M01);  //Output: E32 M0 M1  
  PORTB ^= (1 << E32_M01); //Set to HIGH
  _delay_ms(500);
}

void E32_SLEEPmode(){
  _delay_ms(100);         // Not necessary - see E32 manual / mode operation
  PORTB &= ~(1 << E32_M01);
  DDRB &= ~(1 << E32_M01);  //Input: E32 M0 M1  
}

// Init ADC
void ADC_init(void){
   ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

// Read Voltage from ADC channel
uint16_t ADCvoltage_read(uint8_t ch){
  uint8_t low, high;
  ADMUX = ADC_REF | ch;          // set reference and channel
  for(uint8_t i=0; i<8; i++){
    ADCSRA |= (1<<ADSC);         // start conversion 8x for value stabilization 
    while(ADCSRA & (1<<ADSC)){}  // wait for conversion complete
//    Vbatt += ADC; 
  }
  low = ADCL;
  high = ADCH;
  return (uint16_t)(((((high << 8) | low) * COEFFICIENT) / 10) + BAT43VDROP);
}

void sleep(){
  DIDR0 = 0x3F;            //Disable digital input buffers on all unused ADC0-ADC5 pins.
  ADCSRA &= ~(1<<ADEN);    //Disable ADC
  ACSR = (1<<ACD);         //Disable the analog comparator
  sleep_mode(); 
}


int
main(void)
{
  DDRB &= ~(1 << ADC_CTRL);  //Input: INT0, where a button is attached (pulled up)
  DDRB |= (1 << WSL_RST);    //Output: WSL Reset
  DDRB |= (1 << WSL_USR);    //Output: WSL USER Button - Long press to sleep
  PORTB &= ~(1 << WSL_RST);  //Make sure the Reset is DOWN
  PORTB &= ~(1 << WSL_USR);  //Make sure the USER is DOWN

  MCUCR &= ~(1 << ISC00);    //Set low level Interrupt
  MCUCR &= ~(1 << ISC01);    //Set low level Interrupt
  GIMSK |= (1<<INT0);        // Enable Pin Change Interrupts

  WDTCR |= (1<<WDP3 )|(0<<WDP2 )|(0<<WDP1)|(1<<WDP0)|(1<<WDE); // 8s   
  WDTCR |= (1<<WDTIE);       // enable Watchdog Timer interrupt
  sei();                     // enable global interrupts

  if (c == INTERVAL){
    c = 0;
    ADC_init();
    uint16_t v = ADCvoltage_read(ADC_CH2_BAT);
    if(v > UTHR_BAT_OK){
      sf = 0; // Set POWER OFF flag      
      WSLReset();
    }
  } 
  if (c > INTERVAL){     //FUSE!! This shouldn't happen
    c = 0;
  }
  
  r++;
  if (r > BMS_INTERVAL){
    r = 0;
    b++;
    ADC_init();
    uint16_t v = ADCvoltage_read(ADC_CH2_BAT);

    if (b >= BATT_INTERVAL){
      b = 0;
      sf = 0;
      if (v > UTHR_BAT_CH){
        sf = 4;
      }
    }
  
    if((v < UTHR_BAT_LO) && (sf == 0)){
      sf = 0x60; // Set POWER OFF flag
      WSLSleep();
    }
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep();

 /* loop */
  while(1);
}
