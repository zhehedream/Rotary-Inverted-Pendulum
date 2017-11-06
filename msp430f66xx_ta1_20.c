#include <msp430.h>
#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include "LCD12864.h"
#include "delay.h"
#include "key.h"
#include "adc.h"

#define PMM_STATUS_OK     0
#define PMM_STATUS_ERROR  1
#define _HAL_PMM_SVMLE  (SVMLE)
#define _HAL_PMM_SVSLE  (SVSLE)
#define _HAL_PMM_SVSFP  (SVSLFP)
#define _HAL_PMM_SVMFP  (SVMLFP)
#define uchar unsigned char
#define uint unsigned int

#define EN_H P3OUT|=BIT1
#define EN_L P3OUT&=~BIT1

#define CS_4004_H P6OUT |= BIT5
#define CS_4004_L P6OUT &=~BIT5
#define MOSI_4004_H P6OUT |= BIT6
#define MOSI_4004_L P6OUT &=~BIT6
#define SCLK_4004_H P6OUT |= BIT7
#define SCLK_4004_L P6OUT &=~BIT7



#define SCL1 P2OUT |=BIT0	
#define SCL0 P2OUT &=~BIT0	

#define SDA1 P2OUT |=BIT1			//IIC数据引脚
#define SDA0 P2OUT &=~BIT1	
#define SDAIN P2DIR &=~BIT1
#define SDAOUT P2DIR |=BIT1	
#define SDADATA (P2IN & BIT1)
//****************************************
// 定义MPU6050内部地址
//****************************************
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据，+1为读取

//**************************************
//I2C起始信号
//**************************************
void I2C_Start()
{
  SDA1;                    //拉高数据线
  SCL1;                    //拉高时钟线
  delay_us(5);                 //延时
  SDA0;                    //产生下降沿
  delay_us(5);                  //延时
  SCL0;                    //拉低时钟线
}
/********************************************************************
函数名称：power_up（）
入口参数：无
出口参数：无
功能：ISD4004的上电指令
********************************************************************/
void power_up()
{
  uchar i,m;
  CS_4004_H;
  SCLK_4004_L;
  CS_4004_L;
  m = 0x20;
  for( i=0; i<8; i++ )
  {
    if( m&0x01 )MOSI_4004_H;
    else MOSI_4004_L;
    SCLK_4004_L;
    m>>=1;
    SCLK_4004_H;
  }
  CS_4004_H;
}

/*****************************************************************
函数名称：play_record（uint add）
入口参数：uint add
出口参数：无
功能：从add处开始放音
********************************************************************/
void play_record( uint add )
{
  add=add/2;
  uchar i,m;
  power_up();
  delay_ms( 25 );
  CS_4004_H;
  SCLK_4004_L;
  CS_4004_L;
  //--------上电--------
  m=0xe0;
  for(i=0;i<16;i++)
  {
    
    if(add&0x01) MOSI_4004_H;
    else MOSI_4004_L;
    SCLK_4004_L;
    add = add>>1;
    SCLK_4004_H;
  }
  for(i=0;i<8;i++)
  {
    
    if(m&0x01) MOSI_4004_H;
    else MOSI_4004_L;
    SCLK_4004_L;
    m=m>>1;
    SCLK_4004_H;
  }
  //---发送PLAY命令---
  CS_4004_H;
  SCLK_4004_L;
  m=0xf0;
  CS_4004_L;
  for(i=0;i<8;i++)
  {
    if(m&0x01) MOSI_4004_H;
    else MOSI_4004_L;
    SCLK_4004_L;
    m=m>>1;
    SCLK_4004_H;
  }
  CS_4004_H;
  
  /* INT_4004_L;
  while(INT_4004_IN) {;}//等待录音结束*/
}
//**************************************
//I2C停止信号
//**************************************
void I2C_Stop()
{
  SDA0;                    //拉低数据线
  SCL1;                    //拉高时钟线
  delay_us(5);                   //延时
  SDA1;                    //产生上升沿
  delay_us(5);                  //延时
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(uchar ack)
{
  SDAOUT;
  if(ack) SDA1;
  else SDA0;
  //    SDA = ack;                  //写应答信号
  SCL1;                    //拉高时钟线
  delay_us(5);                  //延时
  SCL0;                    //拉低时钟线
  delay_us(5);                  //延时
}
//**************************************
//I2C接收应答信号
//**************************************
uchar I2C_RecvACK()
{
  uchar cy;
  SCL1;                    //拉高时钟线
  SDAIN;
  delay_us(5);                 //延时
  if(SDADATA)
  {
    cy=1;
  }
  else 
  {
    cy=0;
  }
  //    cy = SDA;                   //读应答信号
  SCL0;                    //拉低时钟线
  delay_us(5);                //延时
  SDAOUT; 
  return cy;
  
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void I2C_SendByte(uchar dat)
{
  uchar i;
  for (i=0; i<8; i++)         //8位计数器
  {
    if((dat<<i)&0x80)
    {
      SDA1;
    }
    else 
    {
      SDA0;
    }
    // SDA = cy;               //送数据口
    SCL1;                //拉高时钟线
    delay_us(5);              //延时
    SCL0;                //拉低时钟线
    delay_us(5);              //延时
  }
  I2C_RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte()
{
  uchar i;
  uchar dat = 0,cy;
  SDA1;                    //使能内部上拉,准备读取数据,
  SDAIN;
  for (i=0; i<8; i++)         //8位计数器
  {
    
    dat <<= 1;
    SCL1;                //拉高时钟线
    delay_us(5);             //延时
    if(SDADATA) 
    {
      cy=1;
    }
    else 
    {
      cy=0;
    }
    dat |= cy;             //读数据             
    SCL0;                //拉低时钟线
    delay_us(5);             //延时
  }
  SDAOUT;
  return dat;
}
//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(uchar REG_Address,uchar REG_data)
{
  I2C_Start();                  //起始信号
  I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
  I2C_SendByte(REG_Address);    //内部寄存器地址，
  I2C_SendByte(REG_data);       //内部寄存器数据，
  I2C_Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uchar Single_ReadI2C(uchar REG_Address)
{
  uchar REG_data;
  I2C_Start();                   //起始信号
  I2C_SendByte(SlaveAddress);    //发送设备地址+写信号
  I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始	
  I2C_Start();                   //起始信号
  I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号
  REG_data=I2C_RecvByte();       //读出寄存器数据
  I2C_SendACK(1);                //接收应答信号
  I2C_Stop();                    //停止信号
  return REG_data;
}
//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
  Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
  Single_WriteI2C(SMPLRT_DIV, 0x04);
  Single_WriteI2C(CONFIG, 0x06);
  Single_WriteI2C(GYRO_CONFIG, 0x18);
  Single_WriteI2C(ACCEL_CONFIG, 0x01);
}


//**************************************
//合成数据
//**************************************
int GetData(uchar REG_Address)
{
  char H,L;
  H=Single_ReadI2C(REG_Address);
  L=Single_ReadI2C(REG_Address+1);
  return (H<<8)+L;   //合成数据
}


////定义变量

uint16_t SetVCore (uint8_t level);
static uint16_t SetVCoreUp (uint8_t level);


int pwm_current,pwm_set;
int Rec;
int pwm_table[15];
int i,j,k;
double Angle_Real[10],Angle[10],e[10],d[10],ipt=0;
int pt,pt2,EN,Feedback,count1,count2;
uchar mode,mode1,mode2,flag,flag1,flag2,flag3,flag_rst,flag_f,p,input[4];
uchar flag_mode4;
int tmp1,tmp2,tmp3;

#define left_direct 1
#define right_direct 0
#define speed_max 4000
float g_fControlSpeedP;
float g_fControlSpeedI;
float g_fPspeed=4;
float g_fIspeed=0;
float g_fspeedset_speed_New=0;
float g_fspeedset_speed_Old=0; //这两个量是用于平滑的
unsigned int g_nCntSeg;     //中断计数
float g_fspeedset_speed=0;
int speed_want=0,speed_out,zhili_err=0,speed_err=0;
uchar nSpeedCycle=0,speed_direct=0;
uchar Result[6]={'0','0','0','0','\0'};


int FB[10];
long FB_10;
int GYRO_X;


int AD_table[36];


int i;
int X,Y,H,AD,AD1,AD2,AD3;
uchar P2,P6,P7;
int count;

void int_to_char(unsigned int data)
{
  unsigned char i=5;
  while(i--)
  {
    Result[i] = data%10+'0';
    data /= 10;
  }
}

unsigned int get_mid(unsigned int a,unsigned int b,unsigned int c)
{
  unsigned int x=0;
  if(a>b){x=b;b=a;a=x;}
  if(b>c){x=c;c=b;b=x;}
  if(a>b){x=b;b=a;a=x;}
  return b ;
}

void Get_Angle()
{
  
  ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
  while (!(ADC12IFG&BIT0));
  X = ADC12MEM0;
  Y = ADC12MEM1;
  H = ADC12MEM2;
  AD1=get_mid(X,Y,H);
  
  ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
  while (!(ADC12IFG&BIT0));
  X = ADC12MEM0;
  Y = ADC12MEM1;
  H = ADC12MEM2;
  AD2=get_mid(X,Y,H);
  
  ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion
  while (!(ADC12IFG&BIT0));
  X = ADC12MEM0;
  Y = ADC12MEM1;
  H = ADC12MEM2;
  AD3=get_mid(X,Y,H);
  
  AD=get_mid(AD1,AD2,AD3);
  
  if (pt==0)
    pt2=9;
  else
    pt2=pt-1;
  
  
  pt++;
  if(pt==10) pt=0;
  
  if (AD>=690&&AD<=1760)
  {
    Angle[pt]=180+(AD-690)*9/107.0;
  }
  else
  {
    if (AD<=690 && AD>=0)
    {
      Angle[pt]=180-(690-AD)*13/138.0;
    }
    else
    {
      if (AD>=1760&&AD<=2830)
      {
        Angle[pt]=270+(AD-1760)*9/107.0;
      }
      else
      {
        if (AD>=2830&&AD<=4095)
        {
          Angle[pt]=(AD-2830)*21/253.0;
        }
        else
        {
          Angle[pt]=110.0;
        }
      }
    }
    
    //    
    //    if (Angle[pt]-Angle[pt2]>150||Angle[pt2]-Angle[pt]>150)
    //      Angle[pt]=Angle[pt2];
  }
}

void Turn(int n1,int n2)
{
  TA1CCR1 = TA1CCR0-n1;
  TA1CCR2 = TA1CCR0-n2; 
  
}

void timer_ini();






///////////////////////////////yzh++++++////////////////////////////////
void zhili_control(void)
{
  e[pt]=Angle[pt]-184;
  d[pt]=Angle[pt]-Angle[pt2];
  
  zhili_err=e[pt]*100+d[pt]*30;
  /* if (zhili_err>0)
  {
  speed_direct=right_direct;
  //zhili_err = zhili_err +700;
  //if (zhili_err>speed_max)  zhili_err=speed_max;
}
  else
  {
  if (zhili_err < 0)
  {
  speed_direct=left_direct;
  zhili_err = - zhili_err;
  //if (zhili_err>speed_max)  zhili_err=speed_max;
}
    else
  {
  zhili_err=0;
}
}*/
}     

////////////////////////////////////////变量//////////////////////////////////////////
void speedcontrol_set(void) 
{      
  ////////////////////////获得速度////////////////////////////////////////////// 
  GYRO_X=GetData(GYRO_YOUT_H)-20;
  g_fControlSpeedP=(speed_want+GYRO_X)*g_fPspeed; //P控制量
  g_fControlSpeedI+= (speed_want+GYRO_X)*g_fIspeed; //I控制量
  g_fspeedset_speed_Old= g_fspeedset_speed_New;//上一次的控制量，用于平滑
  g_fspeedset_speed_New= g_fControlSpeedP+g_fControlSpeedI; //计算速度控制量
}
//----------------------速度平滑函数--------------------//

void pinghua(void)
{    
  g_fspeedset_speed=(g_fspeedset_speed_New-g_fspeedset_speed_Old)*(nSpeedCycle+1)/20+g_fspeedset_speed_Old;  //平滑后的速度控制量
  speed_err=g_fspeedset_speed;
}

void speed_need(void){
  if(nSpeedCycle==0)//计算新的控制量
  {
    speedcontrol_set(); 
  }
  pinghua();
  if (nSpeedCycle<19) nSpeedCycle++;
  else nSpeedCycle=0;
}

void baidong(void)
{
  Turn(1300,0);
  delay_ms(200);
  Turn(0,400);
  delay_ms(250);
  Turn(0,1250);
  delay_ms(200);
  Turn(400,0);
  delay_ms(250);
  
  
}


int main(void)
{
  WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
  
l1:
  EN_L;
  EN=0;
  Turn(0,400);
  timer_ini();
  key_init();
  
  P2DIR|=0x03;
  
  InitMPU6050();	//初始化MPU6050//GetData(GYRO_XOUT_H);
  
  
  
  
  
  
  LCD12864_initial();
  LCD12864_clear();
  P3DIR |= BIT2+BIT3;                       // P3.2 and P3.3 output
  P3SEL |= BIT2+BIT3;                       // P3.2 and P3.3 options select
  TA1CCR0=4000;
  TA1CCTL1 = OUTMOD_6;                      // CCR1 toggle/set
  TA1CCTL2 = OUTMOD_6;                      // CCR2 toggle/set
  Turn(400,0);
  
  TA1CTL = TASSEL_2 + MC_3;                 // ACLK, up-down mode
  adc_init();
  
  TBCCTL0 = CCIE;                           // CCR0 interrupt enabled
  TBCCR0 = 50000;
  TBCTL = TBSSEL_2 + MC_2 + TBCLR;          // SMCLK, contmode, clear TBR
  
  P1DIR|=BIT0;
  P1IE|=BIT1;
  P1IES|=BIT1;
  _BIS_SR(GIE);
  
  P6DIR|=(BIT5+BIT6+BIT7);
  CS_4004_H ;  
  
  
  
  
  //参数初始化
  pt=0;
  flag1=0;
  flag2=0;
  FB_10=0;
  for (i=0;i<10;i++)
  {
    Angle[i]=0;
    e[i]=0;
    d[i]=0;
    FB[i]=0;
  }
  
  /*
  while(1)
  {
  Get_Angle();
  LCD12864_gotoXY(1,0);
  if (L&BIT7)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT6)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT5)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT4)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT3)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT2)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT1)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  if (L&BIT0)
  LCD12864_sendbyte(iDat,'1');
      else
  LCD12864_sendbyte(iDat,'0');
  LCD12864_gotoXY(2,0);
  LCD12864_sendstr("角度: ");
  LCD12864_sendbyte(iDat,(Angle[pt])/1000);
  LCD12864_sendbyte(iDat,((Angle[pt])/100)%10);
  LCD12864_sendbyte(iDat,((Angle[pt])%100)/10);
  LCD12864_sendbyte(iDat,'.');
  LCD12864_sendbyte(iDat,(Angle[pt])%10); 
  delay_ms(500);
}   
  
  /*
  LCD12864_clear();
  LCD12864_gotoXY(1,0);
  LCD12864_sendstr("按K16 继续");
  flag_f=0;
  while(flag_f==0)
  {
  key_event();
  if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
  {
  key_flag = 0;
  switch(key_val)
  {
case 15: flag_f=1;break;//退出第一步 
  default:break;  
}
}		
}
  LCD12864_clear();
  LCD12864_gotoXY(1,0);
  LCD12864_sendstr("现在开始");
  */
  /*
  while(1)
  {
  //Turn(2000,0);//上限
  //Turn(600,0);//死区
  //delay_ms(500);
  //Turn(0,700);//死区
  //Turn(0,2000);
  //delay_ms(500);
  Turn(800,0);
  delay_ms(500);
  Turn(0,800);
  delay_ms(500);
}
  */
l2:  
  LCD12864_clear();
  LCD12864_gotoXY(1,0);
  LCD12864_sendstr("****模式选择****");
  LCD12864_gotoXY(2,0);
  LCD12864_sendstr("1.基础部分");
  LCD12864_gotoXY(3,0);
  LCD12864_sendstr("2.提高部分");
  LCD12864_gotoXY(4,0);
  LCD12864_sendstr("3.显示角度");
  
  while(flag2==0)
  {
    key_event();
    if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
    {
      key_flag = 0;
      switch(key_val)
      {
      case 0:  LCD12864_clear();flag2=1;mode=1;play_record(135);break;//基础
      case 1:  LCD12864_clear();flag2=1;mode=2;play_record(267);break;//提高 
      case 2:  LCD12864_clear();flag2=1;mode=3;play_record(309);break;//
      default:break;  
      }     
    }
  }
  
  if (mode==1)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("1.摆动至60度");
    LCD12864_gotoXY(2,0);
    LCD12864_sendstr("2.圆周运动");
    LCD12864_gotoXY(3,0);
    LCD12864_sendstr("3.165 保持垂直");  
    LCD12864_gotoXY(4,0);
    LCD12864_sendstr("4.返回上一级");  
    while(flag2==1 && flag3==0)
    {
      key_event();
      if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
      {
        key_flag = 0;
        switch(key_val)
        {
        case 0:  LCD12864_clear();flag3=1;mode2=1;play_record(0);break;//60
        case 1:  LCD12864_clear();flag3=1;mode2=2;play_record(398);break;//圆周 
        case 2:  LCD12864_clear();flag3=1;mode2=3;play_record(352);break;//165
        case 3:  LCD12864_clear();flag2=0;goto l2;break;//返回 
        default:break;  
        }     
      }
      
    }
    
    
  }
  
  if (mode==2)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("1.自由挂起");
    LCD12864_gotoXY(2,0);
    LCD12864_sendstr("2.圆周运动并保持");
    LCD12864_gotoXY(3,0);
    LCD12864_sendstr("3.挂起并保持圆周");
    LCD12864_gotoXY(4,0);
    LCD12864_sendstr("4.返回上一级"); 
    while(flag2==1 && flag3==0)
    {
      key_event();
      if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
      {
        key_flag = 0;
        switch(key_val)
        {
        case 0:  LCD12864_clear();flag3=1;mode2=4;play_record(487);break;//自由挂起
        case 1:  LCD12864_clear();flag3=1;mode2=5;play_record(441);break;//圆周运动
        case 2:  LCD12864_clear();flag3=1;mode2=6;play_record(89);break;//圆周运动
        case 3:  LCD12864_clear();flag2=0;goto l2;break;//返回 
        default:break;  
        }     
      }
    }    
  }
  if (mode==3)
  {
    while(1)
    {
      LCD12864_gotoXY(1,0);
      LCD12864_sendstr("角度显示");
      Get_Angle();
      GYRO_X=GetData(GYRO_YOUT_H)-20;
      /*
      int_to_char(AD);
      LCD12864_gotoXY(2,0);
      LCD12864_sendstr("AD值:");
      LCD12864_sendstr(Result);
      */
      int_to_char(Angle[pt]);
      LCD12864_gotoXY(2,0);
      LCD12864_sendstr("当前角度:");
      tmp2=floor(Angle[pt]);
      LCD12864_sendbyte(iDat,'0'+tmp2/100);
      LCD12864_sendbyte(iDat,'0'+(tmp2/10)%10);
      LCD12864_sendbyte(iDat,'0'+tmp2%10);
      LCD12864_sendstr("度");   
      /*
      LCD12864_gotoXY(3,0);
      LCD12864_sendstr("角速度:");
      if  (GYRO_X<0)
      {
      LCD12864_sendbyte(iDat,'-');
      int_to_char(-GYRO_X);
    }
      else
      {
      LCD12864_sendbyte(iDat,' ');
      int_to_char(GYRO_X);
    }
      LCD12864_sendstr(Result);
      */
      delay_ms(500);
    }
    
  }
  
  
  if (mode2==1)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("摆动至60度");
    
    for (i=0;i<1;i++)
    {    
      Turn(1300,0);
      delay_ms(200);
      Turn(0,400);
      delay_ms(250);
      Turn(0,1250);
      delay_ms(200);
      Turn(400,0);
      delay_ms(250);
    }
    while(1)
    {    
      Turn(1300,0);
      delay_ms(125);
      Turn(0,400);
      delay_ms(300);
      Turn(0,1250);
      delay_ms(125);
      Turn(400,0);
      delay_ms(300);
      
    } 
    
    
  }
  
  if (mode2==2)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("圆周运动");
    LCD12864_gotoXY(2,0);
    LCD12864_sendstr("1.快速摆动至圆周");
    LCD12864_gotoXY(3,0);
    LCD12864_sendstr("2.甩动至圆周");
    
    
    flag_f=0;
    tmp1=0;
    while(flag_f==0)
    {
      key_event();
      if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
      {
        key_flag = 0;
        switch(key_val)
        {
        case 0:  flag_f=1;tmp1=1;play_record(177);break;//60
        case 1:  flag_f=1;tmp1=2;play_record(223);break;//圆周 
        default:break;  
        }     
      }
    }
    
    if (tmp1==1)
    {
      while(1)
      {
        Turn(1700,0);
        delay_ms(170);
        Turn(0,400);
        delay_ms(250);
        Turn(0,1650);
        delay_ms(150);
        Turn(400,0);
        delay_ms(350);
        Turn(1700,0);
        delay_ms(170);
        Turn(0,400);
        delay_ms(250);
        Turn(0,1650);
        delay_ms(150);
        Turn(400,0);
        delay_ms(350);
        delay_ms(500);
        Turn(1700,0);
        delay_ms(150);
        Turn(0,400);
        
        while(1);
        
        
        /*
        delay_ms(500);
        Turn(1600,0);
        delay_ms(200);
        Turn(0,400);
        delay_ms(60);
        Turn(0,2000);
        delay_ms(150);
        Turn(400,0);
        delay_ms(550);
        /*
        
        /*
        Turn(1300,0);
        delay_ms(200);
        Turn(0,400);
        delay_ms(250);
        Turn(0,1250);
        delay_ms(200);
        Turn(400,0);
        delay_ms(250);
        */
      }
    }
    
    
    if (tmp1==2)
    {
      Turn(1700,0);
      delay_ms(200);
      Turn(0,400);
      delay_ms(60);
      Turn(0,2600);
      delay_ms(200);
      Turn(400,0);
      delay_ms(250);
    }
    /*
    for (i=0;i<1;i++)
    {    
    Turn(0,1400);
    delay_ms(200);
    Turn(0,400);
    delay_ms(50);
    Turn(2550,0);
    delay_ms(200);
    Turn(400,0);
    delay_ms(50);
  }
    */
    
    while(1)
    {    
      Turn(1600,0);
      delay_ms(150);
      Turn(0,400);
      delay_ms(200);
      Turn(0,1550);
      delay_ms(150);
      Turn(400,0);
      delay_ms(200);
    } 
    
    
  }
  
  if (mode2==3)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("165 度保持垂直");
    while(1)
    {
      speed_want=0;
      int_to_char(AD);
      LCD12864_gotoXY(2,0);
      LCD12864_sendstr("AD值:");
      LCD12864_sendstr(Result);
      int_to_char(Angle[pt]);
      LCD12864_gotoXY(3,0);
      LCD12864_sendstr("角度:");
      LCD12864_sendstr(Result);     
      LCD12864_gotoXY(4,0);
      LCD12864_sendstr("角速度:");
      if  (GYRO_X<0)
      {
        LCD12864_sendbyte(iDat,'-');
        int_to_char(-GYRO_X);
      }
      else
      {
        LCD12864_sendbyte(iDat,' ');
        int_to_char(GYRO_X);
      }
      LCD12864_sendstr(Result);
      delay_ms(500);
    }
  }
  
  if (mode2==4)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("自由挂起");
    flag_mode4=1;
    
    while (1)
    {
      if ( Angle[pt]>165&&Angle[pt]<195 )
        EN=1;
      else
      {
        if (EN==0)
        {
          Turn(400,0);
          delay_ms(500);
          Turn(1600,0);
          delay_ms(200);
          Turn(0,400);
          delay_ms(60);
          Turn(0,2000);
          delay_ms(150);
          Turn(400,0);
          delay_ms(550);
          /*   
          delay_ms(500);
          Turn(1400,0);
          delay_ms(200);
          Turn(0,400);
          delay_ms(60);
          Turn(0,1900);
          delay_ms(145);
          Turn(400,0);
          delay_ms(550);
          */ 
        }
      }
    }
    
    goto l1;
    
  }
  
  if (mode2==5)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("圆周运动并保持");
    LCD12864_gotoXY(2,0);
    LCD12864_sendstr("1.逆时针");
    LCD12864_gotoXY(3,0);
    LCD12864_sendstr("2.顺时针");
    LCD12864_gotoXY(4,0);
    LCD12864_sendstr("3.方向变换");
    speed_want=0;
    EN=1;
    flag_f=0;
    tmp1=0;
    Turn(400,0);
    while(flag_f==0)
    {
      key_event();
      if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
      {
        key_flag = 0;
        switch(key_val)
        {
        case 0:  flag_f=1;tmp1=1;break;//60
        case 1:  flag_f=1;tmp1=2;break;//圆周 
        case 2:  flag_f=1;tmp1=3;break;//圆周 
        case 4:  flag_f=1;tmp1=4;break;//60
        case 5:  flag_f=1;tmp1=5;break;//圆周 
        case 8:  flag_f=1;tmp1=6;break;//60
        case 9:  flag_f=1;tmp1=7;break;//圆周 
        case 12:  flag_f=1;tmp1=8;break;//60
        case 13:  flag_f=1;tmp1=9;break;//圆周
        default:break;  
        }     
      }
    }
    
    if (tmp1==1)
    {
      while(1)
      {
        EN=1;
        speed_want=100; 
        delay_ms(1000);
        while(1)
        {
          LCD12864_clear();
          LCD12864_gotoXY(1,0);
          LCD12864_sendstr("逆时针");
          LCD12864_gotoXY(2,0);
          LCD12864_sendstr("1.加速");
          LCD12864_gotoXY(3,0);
          LCD12864_sendstr("2.减速");
          LCD12864_gotoXY(4,0);
          LCD12864_sendstr("当前转速:");
          tmp2=speed_want;
          LCD12864_sendbyte(iDat,'0'+tmp2/100);
          LCD12864_sendbyte(iDat,'0'+(tmp2/10)%10);
          LCD12864_sendbyte(iDat,'0'+tmp2%10);
          flag_f=0;
          tmp1=0;
          Turn(400,0);
          while(flag_f==0)
          {
            key_event();
            if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
            {
              key_flag = 0;
              switch(key_val)
              {
              case 0:  flag_f=1;tmp1=1;  if(speed_want<=200) speed_want+=10;break;//60
              case 1:  flag_f=1;tmp1=2; if(speed_want>=50) speed_want-=10;break;//圆周 
              default:break;  
              }     
            }
          }  
        } 
      }
    } 
    
    if (tmp1==2)
    {
      while(1)
      {
        EN=1;
        speed_want=-100; 
        delay_ms(1000);
        while(1)
        {
          LCD12864_clear();
          LCD12864_gotoXY(1,0);
          LCD12864_sendstr("逆时针");
          LCD12864_gotoXY(2,0);
          LCD12864_sendstr("1.加速");
          LCD12864_gotoXY(3,0);
          LCD12864_sendstr("2.减速");
          LCD12864_gotoXY(4,0);
          LCD12864_sendstr("当前转速:");
          tmp2=-speed_want;
          LCD12864_sendbyte(iDat,'0'+tmp2/100);
          LCD12864_sendbyte(iDat,'0'+(tmp2/10)%10);
          LCD12864_sendbyte(iDat,'0'+tmp2%10);
          flag_f=0;
          tmp1=0;
          Turn(400,0);
          while(flag_f==0)
          {
            key_event();
            if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
            {
              key_flag = 0;
              switch(key_val)
              {
              case 0:  flag_f=1;tmp1=1;  if(speed_want>=-200) speed_want-=10;break;//60
              case 1:  flag_f=1;tmp1=2; if(speed_want<=-50) speed_want+=10;break;//圆周 
              default:break;  
              }     
            }
          }  
        } 
      }
    } 
    
    if (tmp1==3)
    {
      while(1)
      {
        EN=1;
        speed_want=-100; 
        delay_ms(10000);
        speed_want=0;
        delay_ms(4000);
        speed_want=100; 
        delay_ms(10000);
        speed_want=0;
        delay_ms(4000);
      }
    }
    
    if (tmp1==4)
    {
      
      while(1)
      {
        EN=1;       
        speed_want=150;
        delay_ms(2000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    }
    
    if (tmp1==5)
    {
      while(1)
      {
        EN=1;
        speed_want=-150; 
        delay_ms(2000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    } 
    
    if (tmp1==6)
    {
      while(1)
      {
        EN=1;       
        speed_want=200;
        delay_ms(1000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    }
    if (tmp1==7)
    {
      while(1)
      {
        EN=1;
        speed_want=-200; 
        delay_ms(1000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    }
    
     
    if (tmp1==8)
    {
      while(1)
      {
        EN=1;       
        speed_want=240;
        delay_ms(1000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    }
    if (tmp1==9)
    {
      while(1)
      {
        EN=1;
        speed_want=-240; 
        delay_ms(1000);
        while(1)
        { 
          speed_want=speed_want*9/10;
          delay_ms(100);
        }
      }
    }
    
  }
  
  
  
  if (mode2==6)
  {
    LCD12864_clear();
    LCD12864_gotoXY(1,0);
    LCD12864_sendstr("挂起并保持圆周");
    flag_mode4=1;
    LCD12864_gotoXY(2,0);
    LCD12864_sendstr("1.逆时针");
    LCD12864_gotoXY(3,0);
    LCD12864_sendstr("2.顺时针");
    
    flag_f=0;
    tmp1=0;
    Turn(400,0);
    while(flag_f==0)
    {
      key_event();
      if(key_flag) //如果捕获到有按键事件的话，刷新键值，显示
      {
        key_flag = 0;
        switch(key_val)
        {
        case 0:  flag_f=1;tmp1=1;Turn(400,0);break;//60
        case 1:  flag_f=1;tmp1=2;Turn(0,400);break;//圆周 
        default:break;  
        }     
      }
    }
    
    if (tmp1==1)
    {
      while (1)
      {
        if ( Angle[pt]>165&&Angle[pt]<195 )
          EN=1;
        else
        {
          if (EN==0)
          {
            delay_ms(500);
            Turn(1600,0);
            delay_ms(180);
            Turn(0,400);
            delay_ms(60);
            Turn(0,2100);
            delay_ms(150);
            Turn(400,0);
            delay_ms(550);
            speed_want=100; 
          }
        }
      }
    }
    
    if (tmp1==2)
    {
      while (1)
      {
        if ( Angle[pt]>165&&Angle[pt]<195 )
          EN=1;
        else
        {
          if (EN==0)
          {
            delay_ms(500);
            Turn(0,1600);
            delay_ms(180);
            Turn(400,0);
            delay_ms(60);
            Turn(2100,0);
            delay_ms(160);
            Turn(0,400);
            delay_ms(550);
            speed_want=-100; 
          }
        }
      }
    }
    
    
    
  }
  
  
  
  
  while (flag_rst);
  
  goto l1;
  
  
}

void timer_ini()
{
  
  SetVCore(PMMCOREV_1);                     // Set VCore = 1.6V for 12MHz clock                            
  
  UCSCTL3 |= SELREF_2;                      // Set DCO FLL reference = REFO
  UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
  
  __bis_SR_register(SCG0);                  // Disable the FLL control loop
  UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
  UCSCTL1 = DCORSEL_5;                      // Select DCO range 24MHz operation
  UCSCTL2 = FLLD_1 + 374;                   // Set DCO Multiplier for 12MHz
  // (N + 1) * FLLRef = Fdco
  // (374 + 1) * 32768 = 12MHz
  // Set FLL Div = fDCOCLK/2
  __bic_SR_register(SCG0);                  // Enable the FLL control loop
  
  // Worst-case settling time for the DCO when the DCO range bits have been
  // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 6xx
  // UG for optimization.
  // 32 x 32 x 12 MHz / 32,768 Hz = 375000 = MCLK cycles for DCO to settle
  __delay_cycles(375000);
  
  // Loop until XT1,XT2 & DCO fault flag is cleared
  do
  {
    UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
    // Clear XT2,XT1,DCO fault flags
    SFRIFG1 &= ~OFIFG;                      // Clear fault flags
  }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
  
  
}

uint16_t SetVCore (uint8_t level)
{
  uint16_t actlevel;
  uint16_t status = 0;
  level &= PMMCOREV_3;                       // Set Mask for Max. level
  actlevel = (PMMCTL0 & PMMCOREV_3);         // Get actual VCore
  
  while (((level != actlevel) && (status == 0)) || (level < actlevel))		// step by step increase or decrease
  {
    if (level > actlevel)
      status = SetVCoreUp(++actlevel);
  }
  return status;
}

static uint16_t SetVCoreUp (uint8_t level)
{
  uint16_t PMMRIE_backup,SVSMHCTL_backup;
  
  // Open PMM registers for write access
  PMMCTL0_H = 0xA5;
  
  // Disable dedicated Interrupts to prevent that needed flags will be cleared
  PMMRIE_backup = PMMRIE;
  PMMRIE &= ~(SVSMHDLYIE | SVSMLDLYIE | SVMLVLRIE | SVMHVLRIE | SVMHVLRPE);
  SVSMHCTL_backup = SVSMHCTL;
  PMMIFG &= ~(SVMHIFG | SVSMHDLYIFG);
  // Set SVM highside to new level and check if a VCore increase is possible
  SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);    
  // Wait until SVM highside is settled
  while ((PMMIFG & SVSMHDLYIFG) == 0);  
  // Check if a VCore increase is possible
  if ((PMMIFG & SVMHIFG) == SVMHIFG){       //-> Vcc is to low for a Vcore increase
    // recover the previous settings
    PMMIFG &= ~SVSMHDLYIFG;
    SVSMHCTL = SVSMHCTL_backup;
    // Wait until SVM highside is settled
    while ((PMMIFG & SVSMHDLYIFG) == 0);
    // Clear all Flags
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
    // backup PMM-Interrupt-Register
    PMMRIE = PMMRIE_backup;
    
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
    return PMM_STATUS_ERROR;            // return: voltage not set
  }
  // Set also SVS highside to new level	    //-> Vcc is high enough for a Vcore increase
  SVSMHCTL |= (SVSHRVL0 * level);
  // Set SVM low side to new level
  SVSMLCTL = SVMLE | (SVSMLRRL0 * level);
  // Wait until SVM low side is settled
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  // Clear already set flags
  PMMIFG &= ~(SVMLVLRIFG | SVMLIFG);
  // Set VCore to new level
  PMMCTL0_L = PMMCOREV0 * level;
  // Wait until new level reached
  if (PMMIFG & SVMLIFG)
    while ((PMMIFG & SVMLVLRIFG) == 0);
  // Set also SVS/SVM low side to new level
  PMMIFG &= ~SVSMLDLYIFG;
  SVSMLCTL |= SVSLE | (SVSLRVL0 * level);
  // wait for lowside delay flags
  while ((PMMIFG & SVSMLDLYIFG) == 0);
  
  // Disable SVS/SVM Low
  // Disable full-performance mode to save energy
  SVSMLCTL &= ~(_HAL_PMM_SVSLE + _HAL_PMM_SVMLE + _HAL_PMM_SVSFP + _HAL_PMM_SVMFP);
  SVSMHCTL &= ~(_HAL_PMM_SVSFP + _HAL_PMM_SVMFP);
  
  // Clear all Flags
  PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
  // backup PMM-Interrupt-Register
  PMMRIE = PMMRIE_backup;
  
  // Lock PMM registers for write access
  PMMCTL0_H = 0x00;
  return PMM_STATUS_OK;                               // return: OK
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  
  flag_rst=0;
  P1IFG =0;
}

// Timer B0 interrupt service routine
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR (void)
{
  P1OUT^=BIT0;
  Get_Angle();
  if (mode2==3)
  {
    if( Angle[pt]>160&&Angle[pt]<200)
    {
      EN=1;
    }
    else
    {
      if (Angle[pt]<150||Angle[pt]>220)
      {
        EN=0;
        Turn(400,0);
      }
    }
  }
  
  if (mode2==4)
  {
    if(EN==0&&Angle[pt]>160&&Angle[pt]<200)
    {
      EN=1;
    }
    else
    {
      if (EN==1&&(Angle[pt]<150||Angle[pt]>220))
      {
        EN=0;
      }
    }
  }
  
  if (mode2==5&&tmp1>0)
  {
    if(EN==0&&Angle[pt]>160&&Angle[pt]<200)
    {
      EN=1;
    }
    else
    {
      if (EN==1&&(Angle[pt]<150||Angle[pt]>220))
      {
        EN=0;
      }
    }
  }
  
  if (mode2==6)
  {
    if(EN==0&&Angle[pt]>160&&Angle[pt]<200)
    {
      EN=1;
    }
    else
    {
      if (EN==1&&(Angle[pt]<150||Angle[pt]>220))
      {
        EN=0;
      }
    }
  }
  
  if (EN==1)
  { 
    zhili_control();
    speed_need();
    
    speed_out=zhili_err-speed_err;
    if (speed_out >0)
    {
      speed_out += 600;
      if (speed_out > speed_max) speed_out=speed_max;
      Turn (0 ,speed_out);
    }
    else
    {
      speed_out = - speed_out;
      speed_out += 700;
      if (speed_out > speed_max) speed_out=speed_max;
      Turn (speed_out ,0 );
    }
    /* if (speed_direct == left_direct)
    {
    speed_out +=  600;
    if (speed_out > speed_max)  speed_out=speed_max;
    Turn (speed_out , 0);
  }
    else {
    speed_out += 700;
    if (speed_out > speed_max)  speed_out=speed_max;
    Turn ( 0 , speed_out);
  }*/
    
  }
  
  
  TBCCR0 += 50000;                          // Add Offset to CCR0 [Cont mode]
}

