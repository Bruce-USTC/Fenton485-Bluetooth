#ifndef __USUALLY_H
#define __USUALLY_H

//ͷ�ļ�����
#include "stm32f10x.h"
#include "hardware.h"

//LED���Ŷ���
//#define LED1 PDout(12)	   	//�����϶�Ӧ��LD1
//#define LED2 PDout(13)	   	//�����϶�Ӧ��LD2
//#define LED3 PDout(14)	   	//�����϶�Ӧ��LD3
//#define LED4 PDout(15)	   	//�����϶�Ӧ��LD3

//#define KEY_USER  PCin(13) 	//�����϶�Ӧ�ڰ���USER
#define KEY_S  PCin(0)  	//�����϶�Ӧ�ڰ���JOY-SEN
#define KEY_D  PCin(1)  	//�����϶�Ӧ�ڰ���JOY-DOWN
#define KEY_L  PCin(2)  	//�����϶�Ӧ�ڰ���JOY-LEFT
#define KEY_R  PCin(3)  	//�����϶�Ӧ�ڰ���JOY-RIGHT
#define KEY_U  PCin(4)  	//�����϶�Ӧ�ڰ���JOY-UP

//����һ��ö��
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
