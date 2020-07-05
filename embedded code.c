#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"


/* macros*/
//#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_MASK 8
//#define RED_LED_MASK 2
#define MAX_CHAR 40
#define MAX_FIELDS 10
#define AIN0_MASK 8
#define AIN1_MASK 4
#define TRUE 1
#define FALSE 0
// PortD masks
#define TX_MASK 128
//#define A0_MASK 4
#define FSS_MASK 32
#define CLK_MASK 16

// Set pixel arguments
#define CLEAR  0
#define SET    1
#define INVERT 2

//#define CS (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //f1


/* global variables*/
uint8_t posi[]={0};
uint8_t maxarg=0;
char str[MAX_CHAR+1]={0};
uint8_t* argcount=0;
uint8_t argnum=0;
uint16_t rawvalue;
float avgvoltage;

uint16_t rawval;
float avolt;
//char str2[20];
float volta;
uint8_t output;
uint16_t rvalue1=0;
uint16_t rvalue2=0;
uint8_t mode = 0;

uint32_t sfreq;
uint32_t freq1;
uint32_t freq2;
float svolt;
uint32_t soff;
uint32_t sdacs;
float sduty;
uint32_t phasea;
uint32_t phaseb;
uint32_t accumulatora;
uint32_t accumulatorb;
uint32_t tablea[][4096]={0};
uint16_t tableb[][4096]={0};
uint32_t sinewave;
uint32_t out1;
uint32_t out2;
uint32_t index=0;
uint8_t cycle=0;
uint8_t ctrls = 0;
uint8_t ctrlsq = 0;
uint8_t ctrlsw = 0;
uint8_t ctrlt = 0;
uint8_t ctrlsd=0;
uint32_t loff = 0;


uint32_t *displaycyclesa=0;
uint32_t p=0;
uint32_t counter=0;
uint32_t r;

uint8_t test;


// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1



// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, F peripherals
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOF| SYSCTL_RCGC2_GPIOE ;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB  |SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA |SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD |SYSCTL_RCGC2_GPIOB  ;
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    //SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOB  | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    // Enable clocks
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD;


    // Configure LED pin
    GPIO_PORTF_DIR_R = GREEN_LED_MASK|0X02; //| RED_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK;// | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK|0X02; //| RED_LED_MASK;


    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enable TX, RX, and module

    // Configure SSI2 pins for SPI configuration
    GPIO_PORTB_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
    GPIO_PORTB_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB5_SSI2FSS | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI1
    GPIO_PORTB_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
    GPIO_PORTB_PUR_R |= CLK_MASK| FSS_MASK ;                      // SCLK must be enabled when SPO=1 (see 15.4)

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                        // turn off SSI1 to allow re-configuration
    SSI2_CR1_R = 0;                                    // select master mode
    SSI2_CC_R = 0;                                     // select system clock as the clock source
    SSI2_CPSR_R = 10;                                  // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_SPH | SSI_CR0_SPO | SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;
    LDAC=1;

    //Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;                            // set load value to 400 (40MHz *10us) for 100kHz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;              // turn-on timer

}


void timer1Isr()
{

    if (mode == 1)
    {
        LDAC=0;
        LDAC=1;
        if(sdacs==0)
        {
            SSI2_DR_R = rvalue1;
            LDAC=0;
            LDAC=1;
        }
        if(sdacs==1)
        {
            SSI2_DR_R = rvalue2;
            LDAC=0;
            LDAC=1;
        }
    }
    if (mode == 0)
    {
        if(cycle == 0)
                {
            if(sdacs==0)
            {
                LDAC=0;
                LDAC=1;
                accumulatora= accumulatora+ phasea;
                out1 = tablea[0][accumulatora >> 20];
                SSI2_DR_R = out1;
                LDAC=0;
                LDAC=1;
            }
            if(sdacs==1)
            {
                LDAC=0;
                LDAC=1;
                accumulatorb= accumulatorb+ phaseb;
                out1 = tablea[0][accumulatorb >> 20];
                SSI2_DR_R = out1;
                LDAC=0;
                LDAC=1;
            }
                }
    }
    if (cycle == 1)
    {
        if (counter == 0)
        {
            //mode=1;
            //dontdoanything();
            SSI2_DR_R=0;
             cycle = 0;
        }
        else
        {
            if(sdacs==0)
            {

                LDAC=0;
                __asm("             NOP");                  // 5
                __asm("             NOP");
                __asm("             NOP");                  // 5
                __asm("             NOP");
                LDAC=1;
                accumulatora= accumulatora+ phasea;
                out1 = tablea[0][accumulatora >> 20];
                SSI2_DR_R = out1;
                counter--;
                //LDAC=0;
                //LDAC=1;
            }
            if(sdacs==1)
            {
                counter--;
                LDAC=0;
                __asm("             NOP");                  // 5
                __asm("             NOP");
                __asm("             NOP");                  // 5
                __asm("             NOP");
                LDAC=1;
                accumulatorb= accumulatorb+ phaseb;
                out1 = tablea[0][accumulatorb >> 20];
                SSI2_DR_R = out1;
                //LDAC=0;
                //LDAC=1;
            }
        }

    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

void outdac(uint16_t command)
{
    mode = 1;
    SSI2_DR_R = command;               // write command
    while (SSI2_SR_R & SSI_SR_BSY);
    LDAC=0;
    LDAC=1;
}

chann1(void)
{
    // Configure AIN0 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN0_MASK;                 // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;                 // turn on analog operation on pin PE0

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
}

chann2(void)
{
    // Configure AIN0 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;                 // select alternative functions for AN1 (PE2)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;                 // turn on analog operation on pin PE0

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 1;                               // set first sample to AIN1
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;
}

// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0 ()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}


void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}
/*GET A STRING*/
char* get_string(char str[] , uint8_t maxcharacters )
{
    char c;
    uint8_t count=0;
    while(count < maxcharacters)
    {
        a: c= getcUart0();

        if( (c==8) || (c==127) ) /*check if its a backspace*/
        {
            if(count> 0)
            {
                count--;
                //GREEN_LED ^= 1;
                goto a;
            }
            else
            {
                goto a;
            }
        }
        else if( (c==13) || (c==10) )  /*check if its a /r or /n*/
        {
            b: str[count]=0;
            break;
        }

        else if (c >=' ')  /*check if its a backspace*/
        {

            str[count++] = c;
            if(count == maxcharacters)  /*check if max chars are reached*/
            {
                //str[count]=0;
                goto b;
            }
            else
            {
                goto a;
            }
        }
        else
        {
            goto a;
        }
    }
    return str ;
}

parsestring(char str[], uint8_t position[], uint8_t maxfields, uint8_t* argcount)

{

    uint8_t indexofposition=0;

    uint8_t index;
    for(index=0; index<=20;index++ )
    {

        if((str[index]>=65 && str[index]<=90) || (str[index]>=97 && str[index]<=122)||(str[index]>=48 && str[index]<=57) ||str[index]== 46)
        {
            str[index] = str[index];
        }
        else
        {
            str[index]='\0';
        }
        if(index>0)  /*position of first valid chars*/
        {

            if((str[index-1] =='\0') && (str[index]!=0))
            {

                position[indexofposition]= index;
                indexofposition++;

            }
        }
    }
    *argcount=indexofposition; /*number of 1st valid chars*/
    // *argcount=3;
}

char* getargstring(char strg[], uint8_t position[], uint8_t maxarg, uint8_t argcount, uint8_t argnum)
//char* getargstring(char strg[], uint8_t maxarg, uint8_t argcount, uint8_t argnum)
{
    return &strg[position[maxarg]] ;
}

Reset(void)
{
    //
    // Jump to the CCS C initialization routine.  This will enable the
    // floating-point unit as well, so that does not need to be done here.
    //
    __asm("    .global _c_int00\n"
            "    b.w     _c_int00");
}

Reset2(void)
{
    NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

bool iscommand (char strcm[], uint8_t minarg)
{
    if((10>=minarg) && (strcmp(&str[posi[0]], strcm)==0))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

uint32_t getargint(uint8_t name1)
{
    return atoi(&str[posi[name1]]);
}

float getargfloat(uint8_t name)
{
    return atof(&str[posi[name]]);
}

settingdcvolt(uint8_t out, float voltag)
{
    if(out==0)
    {
        rvalue1=12288+((-373*voltag)+2112);  /*DACOUT A*/
        outdac(rvalue1);
    }
    else if (out==1)
    {
        rvalue2=45056+((-373*voltag)+2112);
        outdac(rvalue2);
    }
    else
    {
        putsUart0("error in dac");
    }
}

setsine(uint32_t freqs, float volts, float offset, uint8_t sdac)
{

    mode=0;

    if (sdac == 0)
    {
        phasea=freqs * 4294967296/100000;
        putsUart0("for sine");
        putsUart0("\r\n");

        for(index=0;index<4096;index++)
        {
            sinewave =(2112+(-373*((volts)*sin(index*2*3.14/4096)+ offset)));
            tablea[0][index]=0x3000+sinewave;

        }
    }

    else if (sdac == 1)
    {
        phaseb=freqs * 4294967296/100000;

        putsUart0("for sine");
        putsUart0("\r\n");

        for(index=0;index<4096;index++)
        {
            sinewave =(2112+(-373*((volts)*sin(index*2*3.14/4096)+ offset)));
            tablea[0][index]=0xB000+sinewave;
        }
    }
    else
    {
        putsUart0("wrong dac");
        putsUart0("\r\n");
    }
}

sawtooth(uint32_t frequency, float voltage, float offset, uint8_t sdac)
{
    mode=0;
    test=sdac;
    if (test == 0)
    {
        phasea = frequency * 4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;

        putsUart0("sawtooth");
        putsUart0("\r\n");
        for (i=0;i<4096;i++)
        {
            tablea[0][i] =0x3000-373*(offset+voltage+0.2) +(373*voltage+2112)-((373*voltage*i)/4096);
        }
    }
    else if (test == 1)
    {
        phasea = frequency * 4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;

        putsUart0("sawtooth");
        putsUart0("\r\n");
        for (i=0;i<4096;i++)
        {
            tablea[0][i] =0xB000-373*(offset+voltage+0.2) +(373*voltage+2112)-((373*voltage*i)/4096);
        }
    }
    else
    {
        putsUart0("wrong dac");
        putsUart0("\r\n");
    }


}

void triangle(uint32_t frequency,float voltage,float offset,uint8_t sdac)
{
    if (sdac == 0)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("triangle");
        putsUart0("\r\n");
        for (i = 0; i < 1024; i++)
        {
            tablea[0][i] = 0x3000 - 373 * (offset+(voltage/2)+0.2)+  2112- (373 * (voltage/2) * i / 1024);
        }
        for (i = 1024; i < 3072; i++)
        {
            tablea[0][i] = 0x3000 - 373 * (offset+(voltage/2)+0.2)+ 2112- (373 * (voltage/2)* (2048 - i) / 1024);
        }
        for (i = 3072; i < 4096; i++)
        {
            tablea[0][i] = 0x3000 - 373 * (offset+(voltage/2)+0.2) + 2112+ (373 * (voltage/2) * (4096 - i) / 1024);
        }

    }

    else if (sdac == 1)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("triangle");
        putsUart0("\r\n");
        for (i = 0; i < 1024; i++)
        {
            tablea[0][i] = 0xB000 - 373 * (offset+(voltage/2)+0.2)+  2112- (373 * (voltage/2) * i / 1024);
        }
        for (i = 1024; i < 3072; i++)
        {
            tablea[0][i] = 0xB000 - 373 * (offset+(voltage/2)+0.2)+ 2112- (373 * (voltage/2)* (2048 - i) / 1024);
        }
        for (i = 3072; i < 4096; i++)
        {
            tablea[0][i] = 0xB000 - 373 * (offset+(voltage/2)+0.2) + 2112+ (373 * (voltage/2) * (4096 - i) / 1024);
        }


    }
    else
    {
        putsUart0("wrong dac");
        putsUart0("\r\n");
    }
}



void square(uint32_t frequency,float voltage,uint8_t offset,uint8_t sdac)
{
    if (sdac == 0)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("square");
        putsUart0("\r\n");
        for (i = 0; i < 2048; i++)
        {

            tablea[0][i] = 0x3000 -373 * (offset+(voltage/2)+0.2 ) +2112 + (373 * (voltage/2) * 1);

        }
        for (i = 2048; i < 4096; i++)
        {

            tablea[0][i] = 0x3000 -373 *( offset+(voltage/2) +0.2 )+2112 + (373* (voltage/2) * (-1));

        }


    }
    else if (sdac == 1)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("square");
        putsUart0("\r\n");
        for (i = 0; i < 2048; i++)
        {

            tablea[0][i] = 0xB000 -373 * (offset+(voltage/2)+0.2 ) +2112 + (373 * (voltage/2) * 1);

        }
        for (i = 2048; i < 4096; i++)
        {

            tablea[0][i] = 0xB000 -373 *( offset+(voltage/2)+0.3 )+2112 + (373* (voltage/2) * (-1));

        }
    }
    else
    {
        putsUart0("wrong dac");
        putsUart0("\r\n");
    }
}

squareduty(uint32_t frequency,float voltage,uint8_t offset,uint8_t sdac, float dutycycle)
{
    float du;
    du = 4096 - (4096 * dutycycle / 100);
    if (sdac == 0)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("squareduty");
        putsUart0("\r\n");
        for (i = 0; i <du; i++)
        {

            tablea[0][i] = 0x3000 -373 * (offset+(voltage/2)+0.2 ) +2112 + (373 * (voltage/2) * 1);

        }
        for (i = du; i < 4096; i++)
        {

            tablea[0][i] = 0x3000 -373 *( offset+(voltage/2) +0.2 )+2112 + (373* (voltage/2) * (-1));

        }


    }
    else if (sdac == 1)
    {

        phasea = frequency *4294967296/ 100000;
        uint16_t i = 0;
        mode = 0;
        putsUart0("squareduty");
        putsUart0("\r\n");
        for (i = 0; i < du; i++)
        {

            tablea[0][i] = 0xB000 -373 * (offset+(voltage/2)+0.2 ) +2112 + (373 * (voltage/2) * 1);

        }
        for (i =du; i < 4096; i++)
        {

            tablea[0][i] = 0xB000 -373 *( offset+(voltage/2)+0.2 )+2112 + (373* (voltage/2) * (-1));

        }
    }
    else
    {
        putsUart0("wrong dac");
        putsUart0("\r\n");
    }
}

dontdoanything()
{
    uint32_t i=0;
    if(sdacs==0)

    {
        for ( i = 0;i < 4096;i++)
        {
            tablea[0][i]=0;
        }
    }
    if(sdacs==1)

    {
        for ( i = 0;i < 4096;i++)
        {
            tablea[0][i]=0;
        }
    }
}

gaincal(uint32_t fre1, uint32_t fre2)
{

    uint32_t num;
    float gain;
    float avgvoltagea;
    float avgvoltageb;
    char strd[30];
    char stry[30];
    uint16_t disp_freq=0;



    uint16_t rawvaluea ;
    uint16_t rawvalueb ;

    uint32_t freqjumpsize = 0;
    freqjumpsize = ((fre2 - fre1) / 10);
    putsUart0("gain calculation");
    putsUart0("\r\n");
    if (fre1 < fre2)
    {
        putsUart0("  f          gain");
        putsUart0("\r\n");
        disp_freq=fre1;
        phasea = disp_freq *4294967296 / 100000;
        waitMicrosecond(400000);
        sprintf(strd, "%5u",disp_freq  );
        putsUart0(strd);
        putsUart0("      ");
        //to read ina
        chann1();
        rawvaluea = readAdc0Ss3();
        avgvoltagea = (3.3 * (rawvaluea + 0.5) / 4096);
        // to read inb
        chann2();
        rawvalueb = readAdc0Ss3();
        avgvoltageb = (3.3 * (rawvalueb + 0.5) / 4096);
        gain=avgvoltageb / avgvoltagea;
        //gain = -20 *( log (gain));
        sprintf(stry, "%3.6f ", gain);
        putsUart0(stry);
        putsUart0("\r\n");
        for (num = 0; num < (fre2 - fre1); num += freqjumpsize)
        {
            disp_freq = disp_freq + freqjumpsize;
            phasea = disp_freq *4294967296 / 100000;
            waitMicrosecond(400000);
            sprintf(strd, "%5u",disp_freq  );
            putsUart0(strd);
            putsUart0("      ");
            //to read ina
            chann1();
            rawvaluea = readAdc0Ss3();
            avgvoltagea = (3.3 * (rawvaluea + 0.5) / 4096);
            // to read inb
            chann2();
            rawvalueb = readAdc0Ss3();
            avgvoltageb = (3.3 * (rawvalueb + 0.5) / 4096);
            gain=avgvoltageb / avgvoltagea;
            //gain = 20 *( log10 (gain));
            sprintf(stry, "%3.6f ", gain);
            putsUart0(stry);
            putsUart0("\r\n");
        }


    }
}

/*main*/
int main(void)
{
    // Initialize hardware
    initHw();

    GREEN_LED= 1;
    waitMicrosecond(500000);
    GREEN_LED=0;
    waitMicrosecond(500000);
    while(1)
    {
        //uint32_t d=0x3000;
        //sendGraphicsLcdCommand(d);



        get_string(str, MAX_CHAR);
        //putsUart0(str);
        parsestring(str,posi, MAX_FIELDS, argcount);

        //putsUart0(str);
        getargstring(str,posi, maxarg, *argcount, argnum);
        for ( maxarg=0; maxarg<6;maxarg++)
        {
            putsUart0(getargstring(str,posi, maxarg, *argcount, argnum));
            putsUart0(" \n \r \n");
            putsUart0("\r\n");

        }
        if(iscommand ("reset", 1))
        {
            //RED_LED=1;
            //RED_LED=0;
            Reset();
        }
        else if(iscommand ("voltage", 2))
        {
            if(getargint(1)==1)  /*channel 1*/
            {
                char str1[30];
                chann1();
                rawvalue = readAdc0Ss3();
                avgvoltage = (3.3 * (rawvalue + 0.5) / 4096);
                sprintf(str1, "voltage= %2.5f", avgvoltage);
                putsUart0(str1);
                putsUart0("\r\n");
            }
            else if (getargint(1)==2)
            {
                chann2();
                char str1[30];
                rawvalue = readAdc0Ss3();
                avgvoltage = (3.3 * (rawvalue + 0.5) / 4096);
                sprintf(str1, "voltage= %2.5f", avgvoltage);
                putsUart0(str1);
                putsUart0("\r\n");
            }
            else
            {
                putsUart0("error");
            }
        }


        else if(iscommand ("dc", 3))
        {
            output=getargint(1);
            volta=getargfloat(2);
            settingdcvolt(output,volta);
        }
        else if(iscommand ("sine", 5))
        {
            ctrls = 1;
            ctrlsq = 0;
            ctrlsw = 0;
            ctrlt = 0;
            ctrlsd=0;
            sfreq=getargint(1);
            svolt=getargfloat(2);
            uint32_t lloff=getargint(3);
            soff = lloff;
            sdacs=getargint(4);
            //startsine:setsine(sfreq,svolt,soff,sdacs);

        }
        else if(iscommand("cycles",3))
        {
            //cycle = 1;
            sdacs=getargint(1);
            uint32_t q=getargint(2);
            p = q;
            // p=2;
            displaycyclesa=&p;

            counter=p*(100000/sfreq);
        }
        else if(iscommand ("stop", 1))
        {
            putsUart0("for stop");
            putsUart0("\r\n");
            dontdoanything();
        }
        else if(iscommand ("start", 1))
        {
            putsUart0("for start");
            putsUart0("\r\n");

            if (ctrls == 1 && p==0 )
            {
                //goto startsine;
                setsine(sfreq,svolt,soff,sdacs);
            }
            if (ctrls == 1 && p!=0)
            {
                cycle = 1;
                setsine(sfreq,svolt,soff,sdacs);
            }
            if (ctrlsq == 1 && p==0 )
            {
                square(sfreq,svolt,soff,sdacs);
            }
            if (ctrlsq == 1 && p!=0)
            {
                cycle = 1;
                square(sfreq,svolt,soff,sdacs);
            }
            if (ctrlt == 1 && p==0 )
            {
                triangle(sfreq,svolt,soff,sdacs);
            }
            if (ctrlt == 1 && p!=0)
            {
                cycle = 1;
                triangle(sfreq,svolt,soff,sdacs);
            }

            if (ctrlsw == 1 && p==0 )
            {
                sawtooth(sfreq,svolt,soff,sdacs);
            }
            if (ctrlsw == 1 && p!=0)
            {
                cycle = 1;
                sawtooth(sfreq,svolt,soff,sdacs);
            }
            if (ctrlsd == 1 && p==0 )
            {
                squareduty(sfreq,svolt,soff,sdacs,sduty);
            }
            if (ctrlsd == 1 && p!=0)
            {
                cycle = 1;
                squareduty(sfreq,svolt,soff,sdacs,sduty);
            }

            /* if (ctrlsq == 1)
                goto startsquare;
            if (ctrlsw == 1)
                goto startsawtooth;
            if (ctrlt == 1)
                goto starttriangle;*/
        }

        else if(iscommand ("square", 5))
        {
            ctrls = 0;
            ctrlsq = 1;
            ctrlsw = 0;
            ctrlt = 0;
            ctrlsd=0;
            sfreq=getargint(1);
            svolt=getargfloat(2);
            uint32_t lloff=getargint(3);
            soff = lloff;
            sdacs=getargint(4);
            //startsquare:square(sfreq,svolt,soff,sdacs);
        }
        else if(iscommand ("triangle", 5))
        {
            ctrls = 0;
            ctrlsq = 0;
            ctrlsw = 0;
            ctrlt = 1;
            ctrlsd=0;
            sfreq=getargint(1);
            svolt=getargfloat(2);
            uint32_t lloff=getargint(3);
            soff = lloff;
            sdacs=getargint(4);
            //starttriangle:triangle(sfreq,svolt,soff,sdacs);
        }

        else if(iscommand ("sawtooth", 5))
        {
            ctrls = 0;
            ctrlsq = 0;
            ctrlsw = 1;
            ctrlt = 0;
            ctrlsd=0;
            sfreq=getargint(1);
            svolt=getargfloat(2);
            uint32_t lloff=getargint(3);
            soff = lloff;
            sdacs=getargint(4);
            //startsawtooth:sawtooth(sfreq,svolt,soff,sdacs);
        }
        else if(iscommand ("sqduty", 6))
        {
            ctrls = 0;
            ctrlsq = 0;
            ctrlsw = 0;
            ctrlt = 0;
            ctrlsd=1;
            sfreq=getargint(1);
            svolt=getargfloat(2);
            uint32_t lloff=getargint(3);
            soff = lloff;
            sdacs=getargint(4);
            sduty=getargfloat(5);
            //startsawtooth:sawtooth(sfreq,svolt,soff,sdacs);
        }
        else if(iscommand ("gain", 3))
        {
            freq1=getargint(1);
            freq2=getargint(2);
            gaincal(freq1,freq2);
        }
        else if(iscommand ("alcon", 2))
        {
            if(getargint(1)==0)  /*channel 1*/
            {
                char str2[30];
                chann1();
                rawval = readAdc0Ss3();
                if ( ctrls == 1)
                {
                    avolt = (3.3 * (rawval + 0.5) / 4096)+(0.4*svolt)-(soff*0.13);
                    sprintf(str2, "voltage= %2.2f", avolt);
                    putsUart0(str2);
                    putsUart0("\r\n");
                }
                else
                {
                    avolt = (3.3 * (rawval + 0.5) / 4096)+(0.52*svolt)-(soff*0.85);
                    sprintf(str2, "voltage= %2.2f", avolt);
                    putsUart0(str2);
                    putsUart0("\r\n");
                }

            }
            else if (getargint(1)==1)
            {
                char str2[30];
                chann2();
                rawval = readAdc0Ss3();

                if ( ctrls == 1)
                {
                    avolt = (3.3 * (rawval + 0.5) / 4096)+(0.38*svolt)-(soff*0.13);
                    sprintf(str2, "voltage= %2.2f", avolt);
                    putsUart0(str2);
                    putsUart0("\r\n");
                }
                else
                {
                    avolt = (3.3 * (rawval + 0.5) / 4096)+(0.52*svolt)-(soff*0.95);
                    sprintf(str2, "voltage= %2.2f", avolt);
                    putsUart0(str2);
                    putsUart0("\r\n");
                }

            }
            else
            {
                putsUart0("error");
            }
        }
        else
        {
            putsUart0("wrong command");
            putsUart0("\r\n");
        }

    }
}
