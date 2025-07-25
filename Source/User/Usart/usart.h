#ifndef __USART_H // 确保宏定义唯一，防止重复包含
#define __USART_H

#include "stm32f10x.h" // 你的STM32标准外设库头文件，根据你的芯片系列可能不同 (例如 stm32f4xx.h)
#include <stdio.h>    // 包含此头文件以支持 fputc 函数和 FILE 结构

// --- LED 控制宏定义 ---
// 根据你 Init_LED 函数中 GPIOD 的引脚推断
// 请根据你的实际硬件连接进行调整
// LED1 对应 GPIOD_Pin_14
#define LED1_GPIO_PORT GPIOD
#define LED1_PIN       GPIO_Pin_12
#define LED1(a)        ((a) ? GPIO_SetBits(LED1_GPIO_PORT, LED1_PIN) : GPIO_ResetBits(LED1_GPIO_PORT, LED1_PIN))

// LED2 对应 GPIOD_Pin_12，也用于 USART2 (RS485) 接收指示
#define LED2_GPIO_PORT GPIOD
#define LED2_PIN       GPIO_Pin_13
#define LED2(a)        ((a) ? GPIO_SetBits(LED2_GPIO_PORT, LED2_PIN) : GPIO_ResetBits(LED2_GPIO_PORT, LED2_PIN))
#define LED2_TOGGLE    GPIO_ReadOutputDataBit(LED2_GPIO_PORT, LED2_PIN) ? GPIO_ResetBits(LED2_GPIO_PORT, LED2_PIN) : GPIO_SetBits(LED2_GPIO_PORT, LED2_PIN)

// LED3 对应 GPIOD_Pin_13，也用于 USART3 (蓝牙) 接收指示
#define LED3_GPIO_PORT GPIOD
#define LED3_PIN       GPIO_Pin_14
#define LED3(a)        ((a) ? GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN) : GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN))
#define LED3_TOGGLE    GPIO_ReadOutputDataBit(LED3_GPIO_PORT, LED3_PIN) ? GPIO_ResetBits(LED3_GPIO_PORT, LED3_PIN) : GPIO_SetBits(LED3_GPIO_PORT, LED3_PIN)


// --- RS485 收发控制引脚宏定义 ---
// DIR 对应 GPIOA_Pin_1
#define DIR_GPIO_PORT GPIOA
#define DIR_PIN       GPIO_Pin_1
#define DIR(a)         ((a) ? GPIO_SetBits(DIR_GPIO_PORT, DIR_PIN) : GPIO_ResetBits(DIR_GPIO_PORT, DIR_PIN))


// --- 蓝牙接收缓冲区大小定义 ---
#define BT_RX_BUFFER_SIZE 64 // 蓝牙接收指令缓冲区大小，可根据实际指令长度调整

// --- 外部变量声明 (这些变量在 usart.c 中定义) ---
extern volatile char bluetoothRxBuffer[BT_RX_BUFFER_SIZE];
extern volatile uint8_t bluetoothRxIndex;
extern volatile uint8_t bluetoothDataReady;


//extern uint8_t Rs485Buffer[100] = {0};
//// uint8_t Rs485Txflag = 0; // 这个标志可以保留，或者用空闲帧中断代替
//extern uint8_t index_i = 0; // RS485缓冲区索引

//// 新增一个标志位，表示RS485接收完成一帧数据
//volatile uint8_t Rs485_Frame_Ready = 0;

// --- 函数声明 ---
void Init_Usart(void);
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate);
void RS485_Send_Byte(uint16_t Data);

#endif // __USART_H
