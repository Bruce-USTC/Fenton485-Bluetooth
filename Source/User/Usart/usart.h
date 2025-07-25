#ifndef __USART_H // ȷ���궨��Ψһ����ֹ�ظ�����
#define __USART_H

#include "stm32f10x.h" // ���STM32��׼�����ͷ�ļ����������оƬϵ�п��ܲ�ͬ (���� stm32f4xx.h)
#include <stdio.h>    // ������ͷ�ļ���֧�� fputc ������ FILE �ṹ

// --- LED ���ƺ궨�� ---
// ������ Init_LED ������ GPIOD �������ƶ�
// ��������ʵ��Ӳ�����ӽ��е���
// LED1 ��Ӧ GPIOD_Pin_14
#define LED1_GPIO_PORT GPIOD
#define LED1_PIN       GPIO_Pin_12
#define LED1(a)        ((a) ? GPIO_SetBits(LED1_GPIO_PORT, LED1_PIN) : GPIO_ResetBits(LED1_GPIO_PORT, LED1_PIN))

// LED2 ��Ӧ GPIOD_Pin_12��Ҳ���� USART2 (RS485) ����ָʾ
#define LED2_GPIO_PORT GPIOD
#define LED2_PIN       GPIO_Pin_13
#define LED2(a)        ((a) ? GPIO_SetBits(LED2_GPIO_PORT, LED2_PIN) : GPIO_ResetBits(LED2_GPIO_PORT, LED2_PIN))
#define LED2_TOGGLE    GPIO_ReadOutputDataBit(LED2_GPIO_PORT, LED2_PIN) ? GPIO_ResetBits(LED2_GPIO_PORT, LED2_PIN) : GPIO_SetBits(LED2_GPIO_PORT, LED2_PIN)

// LED3 ��Ӧ GPIOD_Pin_13��Ҳ���� USART3 (����) ����ָʾ
#define LED3_GPIO_PORT GPIOD
#define LED3_PIN       GPIO_Pin_14
#define LED3(a)        ((a) ? GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN) : GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN))
#define LED3_TOGGLE    GPIO_ReadOutputDataBit(LED3_GPIO_PORT, LED3_PIN) ? GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN) : GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN)


// --- RS485 �շ��������ź궨�� ---
// DIR ��Ӧ GPIOA_Pin_1
#define DIR_GPIO_PORT GPIOA
#define DIR_PIN       GPIO_Pin_1
#define DIR(a)         ((a) ? GPIO_SetBits(DIR_GPIO_PORT, DIR_PIN) : GPIO_ResetBits(DIR_GPIO_PORT, DIR_PIN))


// --- �������ջ�������С���� ---
#define BT_RX_BUFFER_SIZE 64 // ��������ָ�������С���ɸ���ʵ��ָ��ȵ���

// --- �ⲿ�������� (��Щ������ usart.c �ж���) ---
extern volatile char bluetoothRxBuffer[BT_RX_BUFFER_SIZE];
extern volatile uint8_t bluetoothRxIndex;
extern volatile uint8_t bluetoothDataReady;


//extern uint8_t Rs485Buffer[100] = {0};
//// uint8_t Rs485Txflag = 0; // �����־���Ա����������ÿ���֡�жϴ���
//extern uint8_t index_i = 0; // RS485����������

//// ����һ����־λ����ʾRS485�������һ֡����
//volatile uint8_t Rs485_Frame_Ready = 0;

// --- �������� ---
void Init_Usart(void);
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate);
void RS485_Send_Byte(uint16_t Data);

#endif // __USART_H
