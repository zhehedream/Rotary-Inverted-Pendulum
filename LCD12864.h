#include<msp430.h>
#include"delay.h"

#ifndef LCD12864_H
#define LCD12864_H

#define iDat	1		//���ݱ�־
#define iCmd	0		//ָ���־

#define LCDb_RS	    0x20 	//BIT5  	//P5.5
#define LCDb_RW		0x01 	//BIT0  	//P8.0
#define LCDb_E	    0x80	//BIT7		//P8.7
#define LCDb_RST	0x10	//BIT4		//P5.4

#define LCDb_L1		0x80	//��һ�еĵ�ַ
#define LCDb_L2		0x90	//�ڶ��еĵ�ַ
#define LCDb_L3		0x88	//�����еĵ�ַ
#define LCDb_L4		0x98	//�����еĵ�ַ

#define LCDb_SET_RS			P5OUT|=LCDb_RS	//�ĸ����ƹܽŵĿ��Ʋ���
#define LCDb_SET_RW 		P8OUT|=LCDb_RW
#define LCDb_SET_E  		P8OUT|=LCDb_E
#define LCDb_CLR_RST  		P5OUT|=LCDb_RST
#define LCDb_CLR_RS 		P5OUT&=~LCDb_RS
#define LCDb_CLR_RW 		P8OUT&=~LCDb_RW
#define LCDb_CLR_E  		P8OUT&=~LCDb_E
#define LCDb_SET_RST 		P5OUT&=~LCDb_RST

#define LCDb_DO		P9OUT	        //����������߶˿ڶ���

#define LCDb_FUNCTION	0x38   	        // Һ��ģʽΪ8λ��2�У�5*8�ַ�
#define LCDb_BASCMD	0x30		// ����ָ�
#define LCDb_CLS	0x01		// ����
#define LCDb_HOME	0x02		// ��ַ����ԭ�㣬���ı�DDRAM����
#define LCDb_ENTRY 	0x06		// �趨����ģʽ�����ӣ���Ļ���ƶ�
#define LCDb_C2L	0x10		// �������
#define LCDb_C2R	0x14		// �������
#define LCDb_D2L	0x18		// ��Ļ����
#define LCDb_D2R	0x1C		// ��Ļ����
#define LCDb_ON		0x0C		// ����ʾ
#define LCDb_OFF	0x08		// �ر���ʾ

#define GRAPH_ON			0x36		//�򿪻�ͼģʽ
#define GRAPH_OFF			0x34		//�رջ�ͼģʽ

extern void LCD12864_portini();
extern void LCD12864_sendbyte(unsigned char DatCmd, unsigned char dByte);
extern void LCD12864_sendstr(unsigned char *ptString);
extern void LCD12864_clear(void);
extern void LCD12864_gotoXY(unsigned char Row, unsigned char Col);
extern void LCD12864_initial(void);
extern void DisplayBMP(unsigned char *bmp);

#endif
