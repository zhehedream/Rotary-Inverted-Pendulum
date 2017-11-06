#include "LCD12864.h"
/******************************************
函数名称: LCD12864_portini()
功    能: 12864端口初始化
参    数: 无
返回值  : 无
********************************************/
void LCD12864_portini()
{
    P9DIR = 0xFF;
    P5DIR |= BIT4;
	P5DIR |= BIT5;
    P8DIR |= BIT0;
	P8DIR |= BIT7; 
}
/******************************************
函数名称: LCD12864_sendbyte
功    能: 向12864液晶写入一个字节数据或者指令
参    数: DatCmd--为iDat时是数据，为iCmd时是指令
	  	  dByte--为写入12864的数据或者指令
返回值  : 无
********************************************/
void LCD12864_sendbyte(unsigned char DatCmd, unsigned char dByte)
{
	if (DatCmd == iCmd)		//指令操作
		LCDb_CLR_RS;
	else
		LCDb_SET_RS;
	_delay_us(5);	
	LCDb_CLR_RW;			//写操作
	_delay_us(5);
        LCDb_SET_E;
	_delay_us(5);
        LCDb_DO = dByte;		//写入数据
	_delay_us(40);
	LCDb_CLR_E;
        _delay_us(5);	
}
/*******************************************
函数名称: LCD12864_sendstr
功    能: 向12864液晶写入一个字符串
参    数: ptString--字符串指针
返回值  : 无
********************************************/
void LCD12864_sendstr(unsigned char *ptString)
{
	while((*ptString)!='\0')		 //字符串未结束一直写
	{
		LCD12864_sendbyte(iDat, *ptString++);
	}
}
/*******************************************
函数名称: LCD12864_clear
功    能: 12864液晶清屏
参    数: 无
返回值  : 无
********************************************/
void LCD12864_clear(void)
{
	LCD12864_sendbyte(iCmd,LCDb_CLS);
	_delay_ms(2);// 清屏指令写入后，2ms 的延时是很必要的!!!
}

/*******************************************
函数名称: LCD12864_gotoXY
功    能: 移动到指定位置
参    数: Row--指定的行
	  	  Col--指定的列
返回值  : 无
********************************************/
void LCD12864_gotoXY(unsigned char Row, unsigned char Col)
{	
	switch (Row)		  //选择行
	{
		case 2:
			LCD12864_sendbyte(iCmd, LCDb_L2 + Col); break;	//写入第2行的指定列
		case 3:
			LCD12864_sendbyte(iCmd, LCDb_L3 + Col); break;	//写入第3行的指定列
		case 4:
			LCD12864_sendbyte(iCmd, LCDb_L4 + Col); break;	//写入第4行的指定列	
		default:
			LCD12864_sendbyte(iCmd, LCDb_L1 + Col); break;	//写入第1行的指定列	
	}
}
/*******************************************
函数名称: LCD12864_initial
功    能: 12864液晶初始化
参    数: 无
返回值  : 无
********************************************/
void LCD12864_initial(void)
{
	LCDb_SET_RST;
	_delay_ms(100);				        // 等待内部复位
	LCDb_CLR_RST;
	LCD12864_portini();			        //端口初始化
	LCD12864_sendbyte(iCmd, LCDb_FUNCTION);	        //功能、模式设定
	LCD12864_sendbyte(iCmd, LCDb_ON);		//打开显示
	LCD12864_clear();				//清屏
	LCD12864_sendbyte(iCmd, LCDb_ENTRY);	        // 输入模式设定		
}

void LcdWriteCommand(unsigned char dByte)
{
    LCD12864_sendbyte(iCmd,dByte);
}

void LcdWriteData(unsigned char dByte)
{
    LCD12864_sendbyte(iDat,dByte);
}


//========================================= 
//函数功能:显示bmp图像（GDRAM） 

//========================================= 
void DisplayBMP(unsigned char *bmp)
{
  unsigned char i = 0,j = 0;
  LcdWriteCommand(0x36);   //扩充指令，绘图打开
 
 for(i=0;i<32;i++)
  
{
 
     LcdWriteCommand(0x80+i);//先送垂直地址
  
    LcdWriteCommand(0x80);  //再送水平地址，显示图片的上半部分
        for(j=0;j<16;j++)
         {
             LcdWriteData(*bmp++);
       
  }
 
  }
 
 for(i=0;i<32;i++)
  
{
 
   LcdWriteCommand(0x80+i); //先送垂直地址
     LcdWriteCommand(0x88)






; //显示图片的下半部分
  
  for(j=0;j<16;j++)
    {
        LcdWriteData(*bmp++);
     }
 
 }
 
 
LcdWriteCommand(0x30); 
//基本指令，绘图关闭

}
