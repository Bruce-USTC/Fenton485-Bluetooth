// STM32标准外设库头文件
#include "stm32f10x.h"
#include <string.h>

// RS485 方向控制引脚定义
#define DIR_GPIO_PORT   GPIOA
#define DIR_PIN         GPIO_Pin_1

// 函数声明，确保在使用前可见
static __inline void DIR_Set(uint8_t a);

// 环形缓冲区大小
#define RX_BUFFER_SIZE 256

// 蓝牙接收环形缓冲区
uint8_t bt_rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t bt_rx_read_ptr = 0;
volatile uint16_t bt_rx_write_ptr = 0;

// RS485 接收环形缓冲区
uint8_t rs485_rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rs485_rx_read_ptr = 0;
volatile uint16_t rs485_rx_write_ptr = 0;

// SysTick 计数器变量
static volatile uint32_t msTicks = 0;

// ======================= 全局声明区开始 =======================
// 定义发送到RS485的数据帧的帧头
// 注意：这个帧头是由STM32在软件中添加到RS485数据包前的，
// 蓝牙模块发送原始数据时不需要包含此帧头。
#define RS485_FRAME_HEADER 0xAA
// ======================= 全局声明区结束 =======================

// 函数声明 (全部提前，以便编译器知道它们的存在)
int16_t get_data_from_buffer(uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size);
int8_t put_data_to_buffer(uint8_t data, uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size);
void Init_Usart(void);
void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate);
void Init_NVIC(void);
void Delay_Ms(uint32_t time);
void RS485_Send_Byte(uint8_t Data);
void Bluetooth_Send_Byte(uint8_t Data);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void Hardware_Init(void);
void Process_Uart_Data(void);
void SysTick_Handler(void);
uint16_t CaculateCRC(uint8_t *Data, uint16_t len);


// ======================= 函数实现 =======================

static __inline void DIR_Set(uint8_t a) {
    if (a) {
        GPIO_SetBits(DIR_GPIO_PORT, DIR_PIN);
    } else {
        GPIO_ResetBits(DIR_GPIO_PORT, DIR_PIN);
    }
}

int16_t get_data_from_buffer(uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size) {
    int16_t data = -1;
    __disable_irq();
    if (*read_ptr != *write_ptr) {
        data = buffer[*read_ptr];
        *read_ptr = (*read_ptr + 1) % size;
    }
    __enable_irq();
    return data;
}

int8_t put_data_to_buffer(uint8_t data, uint8_t *buffer, volatile uint16_t *read_ptr, volatile uint16_t *write_ptr, uint16_t size) {
    uint16_t next_write_ptr = (*write_ptr + 1) % size;
    __disable_irq();
    if (next_write_ptr == *read_ptr) {
        __enable_irq();
        return -1;
    } else {
        buffer[*write_ptr] = data;
        *write_ptr = next_write_ptr;
        __enable_irq();
        return 0;
    }
}

// CRC 计算函数实现 (返回原始计算结果，不交换高低字节)
uint16_t CaculateCRC(uint8_t *Data, uint16_t len)
{
    uint16_t n, i;
    uint16_t crc = 0xffff;
    for (n = 0; n < len; n++)
    {
        crc ^= Data[n];
        for (i = 0; i < 8; i++)
        {
            if ((crc & 1) == 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc; // 返回计算得到的CRC码
}


void SysTick_Handler(void) {
    msTicks++;
}

void Delay_Ms(uint32_t time) {
    uint32_t current_ticks = msTicks;
    while ((msTicks - current_ticks) < time);
}


void RS485_Send_Byte(uint8_t Data)
{
    DIR_Set(1);
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
    USART_SendData(USART2, Data);
    while(!USART_GetFlagStatus(USART2, USART_FLAG_TC));
    DIR_Set(0);
}

void Bluetooth_Send_Byte(uint8_t Data)
{
    while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
    USART_SendData(USART3, Data);
}


void Init_Usart(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_GPIO_PORT, &GPIO_InitStructure);
    DIR_Set(0);
}


void Usart_Configuration(USART_TypeDef* USARTx, uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate              = BaudRate;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

    USART_Cmd(USARTx, ENABLE);
}

void USART3_IRQHandler(void)
{
    uint8_t ReceiveData;
    if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART3);
    }

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        ReceiveData = USART_ReceiveData(USART3);
        put_data_to_buffer(ReceiveData, bt_rx_buffer, &bt_rx_read_ptr, &bt_rx_write_ptr, RX_BUFFER_SIZE);
    }
    USART_ClearITPendingBit(USART3, USART_IT_FE);
    USART_ClearITPendingBit(USART3, USART_IT_NE);
    USART_ClearITPendingBit(USART3, USART_IT_PE);
}

void USART2_IRQHandler(void)
{
    uint8_t ReceiveData;
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        USART_ReceiveData(USART2);
    }

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        ReceiveData = USART_ReceiveData(USART2);
        put_data_to_buffer(ReceiveData, rs485_rx_buffer, &rs485_rx_read_ptr, &rs485_rx_write_ptr, RX_BUFFER_SIZE);
    }
    USART_ClearITPendingBit(USART2, USART_IT_FE);
    USART_ClearITPendingBit(USART2, USART_IT_NE);
    USART_ClearITPendingBit(USART2, USART_IT_PE);
}


void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    USART_ClearITPendingBit(USART1, USART_IT_ORE);
    USART_ClearITPendingBit(USART1, USART_IT_NE);
    USART_ClearITPendingBit(USART1, USART_IT_FE);
    USART_ClearITPendingBit(USART1, USART_IT_PE);
}


void Hardware_Init(void)
{
    SystemInit();
    if (SysTick_Config(SystemFrequency / 1000)) {
        while (1);
    }

    Init_NVIC();
    Init_Usart();

    Usart_Configuration(USART2, 9600);
    Usart_Configuration(USART3, 9600);
}

// Process_Uart_Data 函数 (STM32 软件添加帧头)
void Process_Uart_Data(void)
{
    static uint8_t bt_packet_buffer[6]; // 用于暂存蓝牙接收到的6个数据字节
    static uint8_t bt_packet_idx = 0;   // 蓝牙数据包当前索引

    int16_t bt_data;
    int16_t rs485_data;
    uint16_t crc_value;
    uint8_t i; // 声明循环变量 i

    // 处理蓝牙接收数据
    bt_data = get_data_from_buffer(bt_rx_buffer, &bt_rx_read_ptr, &bt_rx_write_ptr, RX_BUFFER_SIZE);

    if (bt_data != -1) // 有蓝牙数据可用
    {
        // STM32 蓝牙接收端不再进行帧同步。
        // 它假设蓝牙模块会发送准确的 6 个字节作为数据包。
        // 因此，蓝牙发送端需要确保：
        // 1. 每次只发送 6 个字节的有效数据。
        // 2. 数据包之间有足够的间隔，以避免数据粘连或错位。
        // 如果蓝牙发送了多于 6 个字节的数据，多余的字节将被丢弃。

        if (bt_packet_idx < 6) {
            // 尚未接收满6个字节，将当前字节存入缓冲区
            bt_packet_buffer[bt_packet_idx++] = (uint8_t)bt_data;
        } else {
            // 已经接收到6个字节，后续数据将被丢弃
            // (例如，如果蓝牙发送 "01 06 00 00 00 00 89 CA"，那么 '89' 和 'CA' 将被丢弃)
        }

        // 当接收到足够的6个字节时，处理并发送
        if (bt_packet_idx == 6) {
            // 计算前6个字节的CRC校验码
            crc_value = CaculateCRC(bt_packet_buffer, 6);

            // --- 关键修改：在发送给 RS485 之前，先发送帧头 ---
            RS485_Send_Byte(RS485_FRAME_HEADER); // 先发送帧头 (0xAA)

            // 将6个数据字节发送到RS485
            for (i = 0; i < 6; i++) {
                RS485_Send_Byte(bt_packet_buffer[i]);
            }
            // 按照 Modbus CRC16 标准，先发送低字节，再发送高字节
            RS485_Send_Byte((uint8_t)(crc_value & 0x00FF));       // 发送 CRC 低字节 (例如 0x48)
            RS485_Send_Byte((uint8_t)((crc_value >> 8) & 0x00FF)); // 发送 CRC 高字节 (例如 0x0A)

            // 发送完成后，重置索引，准备接收下一个6字节的包
            bt_packet_idx = 0;
            // 在这里添加一个小的延时（可选，但推荐），给RS485接收端足够的处理时间，
            // 尤其是在连续发送多个包时，有助于提高稳定性。
            // Delay_Ms(10); // 例如，延时10毫秒，根据实际情况调整
        }
    }

    // 处理RS485接收数据 (这部分逻辑保持不变)
    rs485_data = get_data_from_buffer(rs485_rx_buffer, &rs485_rx_read_ptr, &rs485_rx_write_ptr, RX_BUFFER_SIZE);
    if (rs485_data != -1)
    {
        Bluetooth_Send_Byte((uint8_t)rs485_data);
    }
}


int main(void)
{
    Hardware_Init();

    while(1)
    {
        Process_Uart_Data();
    }
}

void Init_NVIC(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    #ifdef  VECT_TAB_RAM
      NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
    #else
      NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
    #endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
