#include"adc.h"
/*******************************************
函数名称: adc_init()
功    能: AD相关端口变量初始化
参    数: 无
返回值  : 无
********************************************/
void adc_init()
{
   
  REFCTL0 &= ~REFMSTR; 
  ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_8+ADC12REFON+ADC12REF2_5V;         // Sampling time, ADC12 on

  

  P6SEL |= BIT0+BIT1+BIT2;                            // P6.0 ADC option select
  
  
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_3;       // Use sampling timer
  ADC12MCTL0 = ADC12INCH_2+ADC12SREF_1;                 // ref+=AVcc, channel = A0
  ADC12MCTL1 = ADC12INCH_2+ADC12SREF_1;                 // ref+=AVcc, channel = A1
  ADC12MCTL2 = ADC12INCH_2+ADC12SREF_1+ADC12EOS;                 // ref+=AVcc, channel = A2
     
  ADC12CTL0 |= ADC12ENC;                       
 
  
}
/*******************************************
函数名称: adc_start()
功    能: AD开始采集
参    数: 无
返回值  : ad采集的值
********************************************/
unsigned int adc_start()
{
  unsigned int ad_val;
  ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
  while (!(ADC12IFG & BIT0));
  ad_val = ADC12MEM0;
  return ad_val;	
}