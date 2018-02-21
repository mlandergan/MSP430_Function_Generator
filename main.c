/************** ECE2049 Lab 4 ******************/
/**************  10/4/2017   ******************/
/***************************************************/
#include <stdlib.h>
#include <msp430.h>
#include "peripherals.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#define DAC_PORT_LDAC_SEL   P3SEL
#define DAC_PORT_LDAC_DIR   P3DIR
#define DAC_PORT_LDAC_OUT   P3OUT
#define DAC_PORT_CS_SEL     P8OUT
#define DAC_PORT_CS_DIR     P8DIR
#define DAC_PORT_CS_OUT     P8OUT

#define DAC_PIN_CS      BIT2
#define DAC_PIN_LDAC        BIT7

enum LAB_STATE {WAITING = 0, DC = 1, SQUAREWAVE = 2, SAWTOOTHWAVE =3, TRIANGLEWAVE = 4};


// Function Prototypes
void runtimerA2Square(void);
void runtimerA2Saw(void);
void configureADC();
unsigned int movingAverage(char);
unsigned int wheelToTicks(unsigned int);
char button_state();
void configure_buttons();
void DACSetValue(unsigned int);
void DACInit(void);

// Declare globals variables
long int timer = 0, lastcount = 0;
unsigned int in_pot, DAC_vals[2], analog1Value;
char count = 0;
unsigned int moving_avg_array[256];//creating the array for the moving avgd
unsigned int scroll_tick;
char currButton;
unsigned long timerSaw = 0;
unsigned int code = 0 ,code2 = 0, step = 186, k = 0;
unsigned int increasing = 1, previous = 1;
volatile bool sample = false;

#pragma vector=TIMER2_A0_VECTOR
__interrupt void Timer_A2_ISR(void)
{
    timer++;
    sample = true;
}

// Main
void main(void)


{
    _BIS_SR(GIE); // Global interrupt enable
    WDTCTL = WDTPW | WDTHOLD;      // Stop watchdog timer
    configDisplay(); // configure display
    configure_buttons(); // configure buttons
    DACInit();

    configureADC();
    enum LAB_STATE state = WAITING; // create an instance of GAME_STATE and intiliaze it to WAITING case
    int i;


    //Code setup
    Graphics_clearDisplay(&g_sContext); // Clear the display

    while (1)    // Forever loop
    {
        switch (state){
            case WAITING:
               Graphics_drawStringCentered(&g_sContext, "Choose Button:", AUTO_STRING_LENGTH, 48, 15, TRANSPARENT_TEXT);
               Graphics_drawStringCentered(&g_sContext, "1-DC", AUTO_STRING_LENGTH, 48, 25, TRANSPARENT_TEXT);
               Graphics_drawStringCentered(&g_sContext, "2-SQUARE", AUTO_STRING_LENGTH, 48, 35, TRANSPARENT_TEXT);
               Graphics_drawStringCentered(&g_sContext, "3-SAWTOOTH", AUTO_STRING_LENGTH, 48, 45, TRANSPARENT_TEXT);
               Graphics_drawStringCentered(&g_sContext, "4-TRIANGLE", AUTO_STRING_LENGTH, 48, 55, TRANSPARENT_TEXT);

               Graphics_flushBuffer(&g_sContext);

              currButton = button_state();
              if (currButton == 0b1000){     // if button 1 is pressed
                   state = DC;
              }
                else if(currButton == 0b0100) { // if button 2 is pressed
                runtimerA2Square(); // configure A2 timer
                state = SQUAREWAVE;
                }
                else if(currButton == 0b0010) { // if button 3 is pressed
                runtimerA2Saw(); // reconfigure A2 timer
                state = SAWTOOTHWAVE;
                }
                else if(currButton == 0b0001) { // if button 4 is pressed
                 runtimerA2Saw(); // reconfigure A2 timer
                 state = TRIANGLEWAVE;
                }
            break;

            case DC:
                runtimerA2Square();
                while (1)    // Forever loop
                    {
                        if(sample) // only sample every tint
                        {
                           // ADC12CTL0 &= ~ADC12SC; //clear start bit
                           // ADC12CTL0 |= ADC12SC;
                            ADC12CTL0 |= ADC12SC + ADC12ENC;

                            // Poll busy bit waiting for conversion to complete
                             while (ADC12CTL1 & ADC12BUSY)
                             __no_operation();

                            in_pot = ADC12MEM0 & 0x0FFF; // read in ADC value for pot
                            DACSetValue(in_pot);
                            analog1Value = ADC12MEM1 & 0x0FFF;
                            sample = false;
                        }
                    }  // end while (1)

            break;

            case SQUAREWAVE:
                lastcount = timer;
                DAC_vals[0] = 0;

                while(1){
                    ADC12CTL0 |= ADC12SC + ADC12ENC;

                    // Poll busy bit waiting for conversion to complete
                 while ((ADC12CTL1 & ADC12BUSY) || (timer == lastcount)){
                     __no_operation();
                }
                   in_pot = ADC12MEM0 & 0x0FFF; // read in ADC value for po
                   DAC_vals[1] = in_pot;
                   DACSetValue(DAC_vals[timer & 1]);
                   lastcount = timer;
                }
                break;

            case SAWTOOTHWAVE:

                while(1){
                if(code > 4090){
                    code = 0;
                }
                DACSetValue(code);
                if ((timer % 22) < 22){
                    code += step;
                   }
                 }

                break;

            case TRIANGLEWAVE:
                code = 0;
                while(1){
                k = timer % 44;
                if (k <= 22){
                    code = k*step;
                   }
                else {
                    code = (44-k) *step;
                   }

                DACSetValue(code);
                }
                break;
        }
    }
}


/** **********************************************
 * Initialize the DAC and its associated SPI bus,
 * using parameters defined in peripherals.h
 ************************************************/
void DACInit(void)
{
    // Configure LDAC and CS for digital IO outputs
    DAC_PORT_LDAC_SEL &= ~DAC_PIN_LDAC;
    DAC_PORT_LDAC_DIR |=  DAC_PIN_LDAC;
    DAC_PORT_LDAC_OUT |= DAC_PIN_LDAC; // Deassert LDAC

    DAC_PORT_CS_SEL   &= ~DAC_PIN_CS;
    DAC_PORT_CS_DIR   |=  DAC_PIN_CS;
    DAC_PORT_CS_OUT   |=  DAC_PIN_CS;  // Deassert CS
}



 void DACSetValue(unsigned int dac_code)
    {
        // Start the SPI transmission by asserting CS (active low)
        // This assumes DACInit() already called
        DAC_PORT_CS_OUT &= ~DAC_PIN_CS;

        // Write in DAC configuration bits. From DAC data sheet
        // 3h=0011 to highest nibble.
        // 0=DACA, 0=buffered, 1=Gain=1, 1=Out Enbl
        dac_code |= 0x3000;     // Add control bits to DAC word

        uint8_t lo_byte = (unsigned char)(dac_code & 0x00FF);
        uint8_t hi_byte = (unsigned char)((dac_code & 0xFF00) >> 8);

        // First, send the high byte
        DAC_SPI_REG_TXBUF = hi_byte;

        // Wait for the SPI peripheral to finish transmitting
        while(!(DAC_SPI_REG_IFG & UCTXIFG)) {
            _no_operation();
        }

        // Then send the low byte
        DAC_SPI_REG_TXBUF = lo_byte;

        // Wait for the SPI peripheral to finish transmitting
        while(!(DAC_SPI_REG_IFG & UCTXIFG)) {
            _no_operation();
        }

        // We are done transmitting, so de-assert CS (set = 1)
        DAC_PORT_CS_OUT |=  DAC_PIN_CS;

        // This DAC is designed such that the code we send does not
        // take effect on the output until we toggle the LDAC pin.
        // This is because the DAC has multiple outputs. This design
        // enables a user to send voltage codes to each output and
        // have them all take effect at the same time.
        DAC_PORT_LDAC_OUT &= ~DAC_PIN_LDAC;  // Assert LDAC
         __delay_cycles(10);                 // small delay
        DAC_PORT_LDAC_OUT |=  DAC_PIN_LDAC;  // De-assert LDAC
    }


// convert scroll wheel value to ACLK tick count
unsigned int wheelToTicks(unsigned int in_pot){
    unsigned int tick = 0;
    tick = ((in_pot * (0.008547)) + 40);
    return tick;
}


void configureADC(void){
    // Reset REFMSTR to hand over control of internal reference
    // voltages to ADC12_A control registers
    REFCTL0 &= ~REFMSTR;

     // SHT0 = 1001b = 384 cycles = middle of the road, example value
     // ADC12ON = Turn on ADC
     // ADC12REFON = Internal reference on and set to 1.5V
     ADC12CTL0 = ADC12SHT0_9 | ADC12ON | ADC12REFON | ADC12MSC;
     ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1;

     // configure ADC for potentiometer
     // Vref+ = 3.3 v, Vref- = GND
     ADC12MCTL0 = ADC12SREF_0 | ADC12INCH_0;


     // configure ADC for Analog 1
     // Vref+ = 3.3 v, Vref- = GND
     ADC12MCTL1 = ADC12SREF_0 | ADC12INCH_1 | ADC12EOS; //end of sequence;

     P6SEL |= (BIT0 | BIT1); // configure A0
     ADC12CTL0 &= ~ADC12SC; // clear start bit
}


void runtimerA2Square(void)
{
    // This function configures and starts Timer A2
    // Use ACLK, 16 Bit, up mode, 1 divider
     TA2CTL = TASSEL_1 + MC_1 + ID_0;
     TA2CCR0 = 163; // 163+1 = 164 ACLK tics = 0.005 second
     TA2CCTL0 = CCIE; // TA2CCR0 interrupt enabled
}

void runtimerA2Saw(void)
{
    // This function configures and starts Timer A2
    // Use SMCLK, 16 Bit, up mode, 1 divider
     TA2CTL = TASSEL_2 + MC_1 + ID_0;
     TA2CCR0 = 475; // 0.45 ms
     TA2CCTL0 = CCIE; // TA2CCR0 interrupt enabled
}



void configure_buttons(){
  //P7.0, P3.6, P2.2, P7.4 are the buttons
  P7SEL = P7SEL &~ (BIT0 | BIT4); //DIGITAL I/O P7.0 AND 7.4
  P3SEL = P3SEL &~ (BIT6); // DIGITAL I/O P3.6
  P2SEL = P2SEL &~ (BIT2); //DIGITAL I/O P2.2

  //SET ALL TO INPUTS
  P7DIR = P7DIR &~ (BIT0 | BIT4);
  P3DIR = P3DIR &~ (BIT6);
  P2DIR = P2DIR &~ (BIT2);

  //ENABLE PULLUP/DOWN RESISTORS
  P7REN = P7REN | (BIT0 | BIT4);
  P3REN = P3REN | (BIT6);
  P2REN = P2REN | (BIT2);

  //PULL UP RESISTORS
  P7OUT = P7OUT | (BIT0 | BIT4);
  P3OUT = P3OUT | (BIT6);
  P2OUT = P2OUT | (BIT2);
}


char button_state(){
  //define char for each button and for all together
  char button1_pressed;
  char button2_pressed;
  char button3_pressed;
  char button4_pressed;
  char buttons_pressed;

  //just in case clear all of the bits in the button#_pressed chars
  button1_pressed = button1_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button2_pressed = button2_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button3_pressed = button3_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  button4_pressed = button4_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
  buttons_pressed = buttons_pressed &~ (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);

  //SET button1_pressed BIT 0 to 7.0
  button1_pressed = ~P7IN & BIT0; //in bit 0 position
  button1_pressed = button1_pressed << 3; //moved to bit 3 position


  //SET button2_pressed BIT 1 to P3.6
  button2_pressed = ~P3IN & BIT6; //x1xxxxxx
  button2_pressed = button2_pressed >> 4; //moved to bit 2 position

  //SET button3_pressed BIT 2 to P2.2
  button3_pressed = ~P2IN & BIT2; //in but 2 position
  button3_pressed = button3_pressed >> 1; //moved to bit 1 position

  //SET button4_pressed BIT 3 to P7.4
  button4_pressed = ~P7IN & BIT4;
  button4_pressed = button4_pressed >> 4; //moved to the bit 0 position

  //or the button#_pressed together to get the result
  buttons_pressed = button1_pressed | button2_pressed | button3_pressed | button4_pressed; //combine the statements
  return buttons_pressed;
}







