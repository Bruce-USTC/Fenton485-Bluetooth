/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 串口底层函数驱动
** 版本：Rev1.0
** 日期：2018-6-20
** 功能：提供USART2(RS485)和USART3(蓝牙)的底层驱动
** printf重定向到USART3
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//头文件调用
#include "usually.h" // 包含常用的宏定义和类型，可能包含 u8 等
#include "usart.h"   // 包含USART相关的宏、函数声明及蓝牙缓冲区定义

#include "stm32f10x.h" // STM32标准外设库头文件
#include <stdio.h>     // 用于 fputc 函数
#include <string.h>    // 用于 memset


// --- printf 函数重定向到 USART3 (蓝牙) ---
// 该部分代码支持printf函数，而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting) // 禁用半主机模式

// 标准库需要的支持函数
struct __FILE
{
    int handle;
};
FILE __stdout;

// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x; // 避免编译器警告，实际无操作
}

// 重定义 fputc 函数，使其通过 USART3 发送字符
int fputc(int Data, FILE *f)
{
    // 等待 USART3 的发送数据寄存器为空，确保上一个数据已发送完成
    while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
    // 通过 USART3 发送当前字符
    USART_SendData(USART3, Data);
    return Data;
}
#endif // #if 1


// --- 蓝牙接收缓冲区及相关变量定义 ---
// 这些变量在 usart.h 中声明为 extern
volatile char bluetoothRxBuffer[BT_RX_BUFFER_SIZE];
volatile uint8_t bluetoothRxIndex = 0;
volatile uint8_t bluetoothDataReady = 0; // 0: 未就绪, 1: 已就绪


// --- RS485 接收缓冲区及相关变量定义 ---
// 这些变量在 usart.h 中声明为 extern
//uint8_t Rs485Buffer[100] = {0};
//uint8_t Rs485Txflag = 0;
//uint8_t index_i = 0; // RS485缓冲区索引

uint8_t Rs485Buffer[100] ;
// uint8_t Rs485Txflag = 0; // 这个标志可以保留，或者用空闲帧中断代替
uint8_t index_i ; // RS485缓冲区索引

// 新增一个标志位，表示RS485接收完成一帧数据
volatile uint8_t Rs485_Frame_Ready ;
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: RS485_Send_Byte
** 功能描述: 通过 USART2 发送一个字节 (RS485)
** 参数描述：Data 要发送的数据
** 作    者: Dream
** 日    期: 2018年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void RS485_Send_Byte(uint16_t Data)
{
    // 等待 USART2 的发送数据寄存器为空
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    // 发送一个字节
    USART_SendData(USART2, Data);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Init_Usart
** 功能描述: 串口引脚初始化 (USART2用于RS485，USART3用于蓝牙)
** 参数描述: 无
** 作    者: Dream
** 日    期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Init_Usart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // --- 蓝牙 (USART3) GPIO 配置 ---
    // 使能 GPIOC, USART3, AFIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // 引脚重映射：USART3使用PC10 (TX) 和 PC11 (RX)
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    // TX3: PC10 配置为复用功能推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // RX3: PC11 配置为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    // --- RS485 (USART2) GPIO 配置 ---
    // 使能 GPIOA, USART2, AFIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // TX2: PA2 配置为复用功能推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX2: PA3 配置为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RS485 收发切换引脚 (DIR): PA1 配置为通用推挽输出
    GPIO_InitStructure.GPIO_Pin = DIR_PIN; // 使用宏定义中的引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStructure); // 使用宏定义中的端口
    DIR(0); // 初始设置为接收模式 (低电平)
}


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
** 函数名称: Usart_Configuration
** 功能描述: 串口通用配置函数
** 参数描述: USARTx 串口号, BaudRate 波特率
** 作    者: Dream
** 日    期: 2011年6月20日
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate          = BaudRate;
    USART_InitStructure.USART_WordLength        = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits          = USART_StopBits_1;
    USART_InitStructure.USART_Parity            = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode              = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USARTx, &USART_InitStructure); // 初始化串口外设
    
    // !!! 修改这里，使能接收中断和空闲帧中断 !!!
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);   // 使能串口接收中断
    USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);   // 使能串口空闲帧中断 (关键)
    
    __nop(); __nop(); // 短暂延时，确保配置生效
    USART_Cmd(USARTx, ENABLE); // 使能串口外设
}


///*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//** 函数名称: USART3_IRQHandler
//** 功能描述: USART3 串口中断处理函数 (蓝牙数据接收)
//** 参数描述: 无
//** 作    者: Dream
//** 日    期: 2011年6月20日
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//void USART3_IRQHandler(void)
//{
//    uint8_t ReceiveData;
//    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        // LED3_TOGGLE; // 接收到蓝牙数据时翻转LED3作为调试指示 (可以保留)
//        ReceiveData = USART_ReceiveData(USART3);

//        if (bluetoothRxIndex < (BT_RX_BUFFER_SIZE - 1))
//        {
//            bluetoothRxBuffer[bluetoothRxIndex++] = (char)ReceiveData;
//        }

//        // 如果接收到换行符('\n')或缓冲区即将满，则认为一条指令接收完成
//        // 注意：多数蓝牙串口助手发送的指令末尾包含 '\r\n'
//        // 或者，如果你希望实时转发，可以去除此判断，每次收到一个字节就设置标志，但在主循环中处理时需要更小心
//        if (ReceiveData == '\n' || bluetoothRxIndex == (BT_RX_BUFFER_SIZE - 1))
//        {
//            bluetoothRxBuffer[bluetoothRxIndex] = '\0'; // 添加字符串结束符
//            bluetoothDataReady = 1; // 设置数据就绪标志，通知主循环处理
//            bluetoothRxIndex = 0;   // 重置索引，准备接收下一条指令
//        }
//        // else if (ReceiveData == '\r') { // 如果需要，可以单独处理回车符
//        //     // 忽略回车符，或者在这里也设置 bluetoothDataReady = 1;
//        // }

//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//    }
//}
///*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//** 函数名称: USART2_IRQHandler
//** 功能描述: USART2 串口中断处理函数 (RS485数据接收)
//** 参数描述: 无
//** 作    者: Dream
//** 日    期: 2011年6月20日
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
//void USART2_IRQHandler(void) // 添加void以规范函数签名
//{
//    uint8_t ReceiveData;

//    // 检查空闲帧中断标志位
//    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
//    {
//        // 软件序列清除 IDLE 标志: 先读 SR，再读 DR
//        USART_GetFlagStatus(USART2, USART_FLAG_IDLE); // 清除 IDLE 标志位（读SR）
//        ReceiveData = USART_ReceiveData(USART2);      // 清除 IDLE 标志位（读DR，这个值是空闲前最后一个字节）

//        // 如果缓冲区有数据（index_i > 0），则表示一帧数据接收完成
//        if (index_i > 0) {
//            Rs485_Frame_Ready = 1; // 设置帧就绪标志
//        }
//        // 不需要清空缓冲区或重置索引，这在主循环处理后进行
//    }

//    // 检查接收数据寄存器非空中断标志位
//    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//    {
//        LED2_TOGGLE; // 接收到RS485数据时翻转LED2作为调试指示
//        ReceiveData = USART_ReceiveData(USART2); // 读取接收到的数据

//        // 将接收到的数据存入RS485缓冲区
//        if (index_i < sizeof(Rs485Buffer)) // 确保缓冲区不溢出
//        {
//            Rs485Buffer[index_i++] = ReceiveData;
//        }
//        else {
//            // 缓冲区溢出处理：可以丢弃后续数据，或者立即设置 Rs485_Frame_Ready = 1;
//            // 这里简单处理为如果溢出，就认为当前帧结束，并设置标志。
//            Rs485_Frame_Ready = 1;
//        }
//        // 清除接收中断标志位
//        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//    }
//}
///**
//  * @brief  This function handles USART1 global interrupt request.
//  * @param  None
//  * @retval None
//  */
//void USART1_IRQHandler(void)
//{
//  /* 检查中断标志位 */
//  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//  {
//    /* 接收缓冲区非空中断处理 */
//    uint8_t data = USART_ReceiveData(USART1); // 读取接收到的数据
//    
//    /* 处理接收到的数据 */
//    // ...
//    
//    /* 清除中断标志 */
//    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//  }
//  
//  /* 其他可能的中断处理 */
//  // ...
//}
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
End:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D:-D
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
