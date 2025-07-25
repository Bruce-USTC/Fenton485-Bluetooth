#ifndef __USUALLY_H
#define __USUALLY_H

//头文件调用
#include "stm32f10x.h"
#include "hardware.h"

//LED引脚定义
//#define LED1 PDout(12)	   	//板子上对应于LD1
//#define LED2 PDout(13)	   	//板子上对应于LD2
//#define LED3 PDout(14)	   	//板子上对应于LD3
//#define LED4 PDout(15)	   	//板子上对应于LD3

//#define KEY_USER  PCin(13) 	//板子上对应于按键USER
#define KEY_S  PCin(0)  	//板子上对应于按键JOY-SEN
#define KEY_D  PCin(1)  	//板子上对应于按键JOY-DOWN
#define KEY_L  PCin(2)  	//板子上对应于按键JOY-LEFT
#define KEY_R  PCin(3)  	//板子上对应于按键JOY-RIGHT
#define KEY_U  PCin(4)  	//板子上对应于按键JOY-UP

//声明一个枚举
enum  						
{							
	 USER=1,				//1
	 SEN,					//2
	 DOWN,					//3
	 LEFT,					//4
	 RIGHT,					//5
	 UP						//6
};
#endif
