#include<msp430.h>
#include"delay.h"

#ifndef LCD12864_H
#define LCD12864_H

#define iDat	1		//数据标志
#define iCmd	0		//指令标志

#define LCDb_RS	    0x20 	//BIT5  	//P5.5
#define LCDb_RW		0x01 	//BIT0  	//P8.0
#define LCDb_E	    0x80	//BIT7		//P8.7
#define LCDb_RST	0x10	//BIT4		//P5.4

#define LCDb_L1		0x80	//第一行的地址
#define LCDb_L2		0x90	//第二行的地址
#define LCDb_L3		0x88	//第三行的地址
#define LCDb_L4		0x98	//第四行的地址

#define LCDb_SET_RS			P5OUT|=LCDb_RS	//四个控制管脚的控制操作
#define LCDb_SET_RW 		P8OUT|=LCDb_RW
#define LCDb_SET_E  		P8OUT|=LCDb_E
#define LCDb_CLR_RST  		P5OUT|=LCDb_RST
#define LCDb_CLR_RS 		P5OUT&=~LCDb_RS
#define LCDb_CLR_RW 		P8OUT&=~LCDb_RW
#define LCDb_CLR_E  		P8OUT&=~LCDb_E
#define LCDb_SET_RST 		P5OUT&=~LCDb_RST

#define LCDb_DO		P9OUT	        //输出数据总线端口定义

#define LCDb_FUNCTION	0x38   	        // 液晶模式为8位，2行，5*8字符
#define LCDb_BASCMD	0x30		// 基本指令集
#define LCDb_CLS	0x01		// 清屏
#define LCDb_HOME	0x02		// 地址返回原点，不改变DDRAM内容
#define LCDb_ENTRY 	0x06		// 设定输入模式，光标加，屏幕不移动
#define LCDb_C2L	0x10		// 光标左移
#define LCDb_C2R	0x14		// 光标右移
#define LCDb_D2L	0x18		// 屏幕左移
#define LCDb_D2R	0x1C		// 屏幕又移
#define LCDb_ON		0x0C		// 打开显示
#define LCDb_OFF	0x08		// 关闭显示

#define GRAPH_ON			0x36		//打开绘图模式
#define GRAPH_OFF			0x34		//关闭绘图模式

extern void LCD12864_portini();
extern void LCD12864_sendbyte(unsigned char DatCmd, unsigned char dByte);
extern void LCD12864_sendstr(unsigned char *ptString);
extern void LCD12864_clear(void);
extern void LCD12864_gotoXY(unsigned char Row, unsigned char Col);
extern void LCD12864_initial(void);
extern void DisplayBMP(unsigned char *bmp);

#endif
