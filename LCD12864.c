#include "LCD12864.h"
/******************************************
��������: LCD12864_portini()
��    ��: 12864�˿ڳ�ʼ��
��    ��: ��
����ֵ  : ��
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
��������: LCD12864_sendbyte
��    ��: ��12864Һ��д��һ���ֽ����ݻ���ָ��
��    ��: DatCmd--ΪiDatʱ�����ݣ�ΪiCmdʱ��ָ��
	  	  dByte--Ϊд��12864�����ݻ���ָ��
����ֵ  : ��
********************************************/
void LCD12864_sendbyte(unsigned char DatCmd, unsigned char dByte)
{
	if (DatCmd == iCmd)		//ָ�����
		LCDb_CLR_RS;
	else
		LCDb_SET_RS;
	_delay_us(5);	
	LCDb_CLR_RW;			//д����
	_delay_us(5);
        LCDb_SET_E;
	_delay_us(5);
        LCDb_DO = dByte;		//д������
	_delay_us(40);
	LCDb_CLR_E;
        _delay_us(5);	
}
/*******************************************
��������: LCD12864_sendstr
��    ��: ��12864Һ��д��һ���ַ���
��    ��: ptString--�ַ���ָ��
����ֵ  : ��
********************************************/
void LCD12864_sendstr(unsigned char *ptString)
{
	while((*ptString)!='\0')		 //�ַ���δ����һֱд
	{
		LCD12864_sendbyte(iDat, *ptString++);
	}
}
/*******************************************
��������: LCD12864_clear
��    ��: 12864Һ������
��    ��: ��
����ֵ  : ��
********************************************/
void LCD12864_clear(void)
{
	LCD12864_sendbyte(iCmd,LCDb_CLS);
	_delay_ms(2);// ����ָ��д���2ms ����ʱ�Ǻܱ�Ҫ��!!!
}

/*******************************************
��������: LCD12864_gotoXY
��    ��: �ƶ���ָ��λ��
��    ��: Row--ָ������
	  	  Col--ָ������
����ֵ  : ��
********************************************/
void LCD12864_gotoXY(unsigned char Row, unsigned char Col)
{	
	switch (Row)		  //ѡ����
	{
		case 2:
			LCD12864_sendbyte(iCmd, LCDb_L2 + Col); break;	//д���2�е�ָ����
		case 3:
			LCD12864_sendbyte(iCmd, LCDb_L3 + Col); break;	//д���3�е�ָ����
		case 4:
			LCD12864_sendbyte(iCmd, LCDb_L4 + Col); break;	//д���4�е�ָ����	
		default:
			LCD12864_sendbyte(iCmd, LCDb_L1 + Col); break;	//д���1�е�ָ����	
	}
}
/*******************************************
��������: LCD12864_initial
��    ��: 12864Һ����ʼ��
��    ��: ��
����ֵ  : ��
********************************************/
void LCD12864_initial(void)
{
	LCDb_SET_RST;
	_delay_ms(100);				        // �ȴ��ڲ���λ
	LCDb_CLR_RST;
	LCD12864_portini();			        //�˿ڳ�ʼ��
	LCD12864_sendbyte(iCmd, LCDb_FUNCTION);	        //���ܡ�ģʽ�趨
	LCD12864_sendbyte(iCmd, LCDb_ON);		//����ʾ
	LCD12864_clear();				//����
	LCD12864_sendbyte(iCmd, LCDb_ENTRY);	        // ����ģʽ�趨		
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
//��������:��ʾbmpͼ��GDRAM�� 

//========================================= 
void DisplayBMP(unsigned char *bmp)
{
  unsigned char i = 0,j = 0;
  LcdWriteCommand(0x36);   //����ָ���ͼ��
 
 for(i=0;i<32;i++)
  
{
 
     LcdWriteCommand(0x80+i);//���ʹ�ֱ��ַ
  
    LcdWriteCommand(0x80);  //����ˮƽ��ַ����ʾͼƬ���ϰ벿��
        for(j=0;j<16;j++)
         {
             LcdWriteData(*bmp++);
       
  }
 
  }
 
 for(i=0;i<32;i++)
  
{
 
   LcdWriteCommand(0x80+i); //���ʹ�ֱ��ַ
     LcdWriteCommand(0x88)






; //��ʾͼƬ���°벿��
  
  for(j=0;j<16;j++)
    {
        LcdWriteData(*bmp++);
     }
 
 }
 
 
LcdWriteCommand(0x30); 
//����ָ���ͼ�ر�

}
