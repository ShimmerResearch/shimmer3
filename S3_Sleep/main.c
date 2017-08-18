#include "msp430.h"
#include "shimmer3_common_source/shimmer_sd_include.h"
//#include "shimmer_btsd.h"
// Shimmer3 Sleep Firmware - v0.2.0
void Init(void);
// make dir for Sleep_FW files
uint8_t  daughtCardId[16];
void main(void) {
    Init();
    UCSCTL4 = SELA_1;
    // Disable SVS
      PMMCTL0_H = PMMPW_H;
      SVSMHCTL &= ~(SVMHE | SVSHE);
      SVSMLCTL &= ~(SVMLE | SVSLE);
while(1)
{
    __bis_SR_register(LPM4_bits);   //ACLK remains active
}
}
void Init(void)
{
    Board_init();
    P3OUT |= BIT3;
    P3DIR |= BIT3;         //set as input
    CAT24C16_init();
    CAT24C16_read(0, 16, daughtCardId);
    if (daughtCardId[DAUGHT_CARD_ID] == 47)
     {
        P6OUT &= ~(BIT6 + BIT7);
        P6REN |= BIT6 + BIT7;       //enable pull down resistor // ADC stops fluctuating
        P6DIR &= ~(BIT6 + BIT7);   //A6 and A7 as input
        P7REN |= BIT7;
        //Internal ADC expansion ports
        P6OUT |= BIT1;
        P6REN |= BIT1;              //enable pull down resistor
        P6DIR |= BIT1;             //A1 set as output /* to be considered for bridge amplifier+*/

        P7OUT &= ~(BIT4 + BIT5);
        P7DIR &= ~(BIT4 + BIT5 ); //A12, A13 and A14 set as input

        //Internal ADC14 expansion ports
         P7OUT |= BIT6;
         P7REN |= BIT6;              //enable pull down resistor
         P7DIR |= BIT6;             //A1 set as output
         //GPIO_INTERNAL1
          P2OUT &= ~BIT0;
          P2REN &= BIT0;              //enable pull down resistor
          P2DIR &= ~BIT0;             //set as input /* to be considered for bridge amplifier+*/
      }
    else
    {
    }
    CAT24C16_powerOff();
    P3DIR &= ~BIT3;            //set as input
    P5OUT &= ~(BIT2 + BIT3);  //XT1
    P5DIR |= BIT2 + BIT3;
    P7OUT &= ~(BIT0 + BIT1);  // XT2
    P7DIR |= BIT0 + BIT1;
    //Set Vcore to minimum
    SetVCore(0);
    //explicitly disable ADC and ADC ref
   ADC12CTL0 &= ~(ADC12ON + ADC12REFON);
}
